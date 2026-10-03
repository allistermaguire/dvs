# DVS G3 RS485 Sniffer

Passive RS485 bus sniffer for a **DVS G3 (DVS-G3 BC220)** home ventilation system,
built on a **D1 Mini ESP32** and a **MAX3485** transceiver.

It listens (receive-only) on the DVS bus *in parallel with the existing wall
controller*, groups the serial bytes into frames and verifies the CRC. It can
decode every frame to the USB console for debugging (off by default — see
`trace` below), and **connect to Wi-Fi and MQTT** to expose the fan as a
controllable device — turn it on/off, set the fan speed, or hand control to an
on-device summer/winter profile — while publishing the sniffed roof-space
temperature and actual fan speed.

Protocol work is based on Ian Rees' write-up,
[*Reverse engineering the DVS bus*](https://ianrees.nz/tech/2018-07-11-reverse-engineering-the-dvs-bus.html)

> **Status:** the bus protocol is reverse-engineered and the control path is
> **confirmed working on hardware** — the ESP32 drives fan power/speed over MQTT
> and runs the summer/winter auto-ventilation controller. The firmware runs
> quiet by default (per-frame tracing off; lifecycle/error logs and MQTT stay
> on).
>
> ⚠️ **Transmitting is active and shares the bus with the wall controller.**
> Frame collisions are still possible. Transmit is **off by default**
> (listen-only); only enable control on a system you can recover, and keep the
> wall controller handy. In listen-only mode the firmware is a pure passive
> sniffer and never drives the bus.

## The DVS bus protocol

Established facts used by this firmware:

| Property     | Value                                                              |
| ------------ | ------------------------------------------------------------------ |
| Physical     | RS485 differential pair                                            |
| Baud / format| 4800 baud, LSb first (standard UART), assumed 8N1                  |
| Frame layout | `[addr:1] [register:1] [payload:0+] [crc16:2]`                     |
| Checksum     | CRC-16/AUG-CCITT (init `0x1D0F`), big-endian on the wire           |
| Flow         | Controller polls a peripheral; the peripheral responds             |

The air-handler's 5-pin 5.08 mm connector:

| Pin | Signal               |
| --- | -------------------- |
| A   | Ground               |
| B   | RS485 **A**          |
| C   | RS485 **B**          |
| D   | +30 VDC (approx)     |
| E   | +12 VDC              |

At the **wall controller** the same bus appears as a 4-wire connector (pins A-D,
no +12 V rail). The pin letters match the table above, but the wire **colours are
counter-intuitive** - verified by multimeter on one unit:

| Pin | Signal               | Wire colour |
| --- | -------------------- | ----------- |
| A   | Ground               | **Red**     |
| B   | RS485 **A**          | White       |
| C   | RS485 **B**          | Blue        |
| D   | +30 VDC (approx)     | **Black**   |

> ⚠️ Note the reversal: **Red is ground and Black is the +30 V rail** on this
> harness - the opposite of the usual convention. Confirm with a meter before
> wiring, and keep the +30 V (Black) wire away from the ESP32/MAX3485. Colours may
> differ between units, so treat this as a guide, not a guarantee.

### Known registers

| Register | Meaning                                                          |
| -------- | ---------------------------------------------------------------- |
| `0x03`   | Deadman (per Ian Rees) - **not observed** in live control captures; this firmware does not use |
| `0x04`   | Set address - one payload byte becomes the peripheral address    |
| `0x05`   | Stop (per Ian Rees) - **not observed**; "off" is really speed 0 via `0x15` |
| `0x10`   | Roof-space temperature - raw count is **little-endian across the payload** (exceeds one byte above ~16 °C); linear over 11-20 °C: `temp_C = 0.06807 * raw - 1.039` (≈14.7 counts/°C) |
| `0x15`   | Fan speed - 1-byte payload = commanded/target (steps instantly on switch-on), 2-byte payload = actual speed (ramps up, soft-start) |

## Control model

Reading the G3 operating guide, effectively **every** homeowner-facing feature
reduces to a single control loop: **airflow (fan speed) as a function of two
temperatures** - a roof-space sensor and a controller (in-home) sensor:

$$\text{speed} = \text{clamp}\big(\text{profile}(T_{roof},\, T_{house}) + \text{airflowOffset},\, \text{cutoffs}\big)$$

- **Retain Home Heat** - reduce speed when `roof < house`.
- **Solar Heat Boost** - increase speed when `roof > house` (the 0-4 on-screen
  arrows scale with the delta).
- **Fan Profiles** (Winter / Summer / Uniform / Auto Summer Vent) - each is just a
  different `profile()` curve mapping the two temps to a speed.
- **Airflow ± / calibration `0`** - a global offset added to the profile output.
- **High/low temp cut-outs** - hard limits that clamp the result (still just
  speed-vs-temp thresholds).

Everything else (Power ON/OFF/TIMER, Presets, reminders, child lock, contrast,
clock) is UI/bookkeeping around that loop - it stores or gates parameters but is
not part of the temperature->speed computation. The only non-temp override is
**Fan Boost** (max speed for a fixed time).

The testable consequence for this project: if you log `0x10` (temps in) against
`0x15` (speed out) over time, **fan speed should be fully predictable from the two
temperatures plus the selected profile**.

**Two-sensor communication (resolved by capture):** only *one* temperature appears
on the bus - register `0x10` from the fan (addr `0x02`) reports the **roof-space**
reading. The **in-home** sensor is wired directly to the wall controller and read
locally, so it never appears on RS485. The two sensors are therefore reported by
different means, not two bus registers.

**Temperature scaling.** The `0x10` payload is a raw roof-space sensor count read
**little-endian across the payload** (above ~16 °C it exceeds one byte, so byte 0
alone wraps - read the whole payload). A least-squares fit to 8 live points over
11-20 °C is linear (residuals < 0.75 °C, limited by the controller's integer
display): `temp_C = 0.06807 * raw - 1.039` (≈ 14.7 counts/°C). Readings at
temperature extremes are still needed to confirm the fit holds outside that range.

**Fan-speed map (measured).** Fan speed (`0x15`) runs **0 (off) to 84 (max/boost,
all profiles)**. Sweeping profile × airflow offset × heat-recovery gives the
commanded `speed_target` values below - i.e. the `profile()` output for the
temperature conditions at capture time:

| Profile   | Offset | HR ON | HR OFF |
| --------- | -----: | ----: | -----: |
| Winter 1  |    +10 |    62 |     66 |
| Winter 1  |      0 |    53 |     56 |
| Winter 1  |    -10 |    43 |     46 |
| Summer    |    +10 |    62 |     66 |
| Summer    |      0 |    52 |     56 |
| Summer    |    -10 |    43 |     46 |
| Uniform 1 |    +10 |    31 |     33 |
| Uniform 1 |      0 |    27 |     27 |
| Uniform 1 |    -10 |    27 |     23 |
| General 1 |    +10 |    49 |     54 |
| General 1 |      0 |    42 |     45 |
| General 1 |    -10 |    34 |     37 |

Observations:
- **Airflow offset** adds ~linearly but with a per-profile gain: ≈ 1.0 speed/unit
  for Winter/Summer, ≈ 0.85 for General, ≈ 0.5 for Uniform.
- **Retain Home Heat (HR ON)** trims speed by ~3-5 vs OFF, consistent with the
  roof/house delta at capture time.
- **Uniform 1 is anomalous** (HR ON = OFF at centre; HR ON > OFF at -10),
  suggesting a floor around 27.

These are single-condition snapshots; the HR delta in particular will vary with
the live roof-vs-house temperature difference.

## Hardware & wiring

By default the firmware only receives: the MAX3485 driver stays disabled (`DE` +
`RE` tied together and held **LOW**) and only the receiver is active. When the
MQTT/console control path sends a command it briefly drives that same pin **HIGH**
to transmit, then returns it LOW to keep listening.

| MAX3485 | Board label | Connects to                                                     |
| ------- | ----------- | --------------------------------------------------------------- |
| A       | A           | DVS pin **C** - (RS485 A)                                       |
| B       | B           | DVS pin **B** - (RS485 B)                                       |
| GND     | GND         | DVS pin **A** - **common ground with the ESP32**                |
| RO      | **TXD**     | ESP32 **GPIO16** (UART2 RX)                                     |
| DI      | **RXD**     | ESP32 **GPIO17** (UART2 TX - unused while listening)            |
| DE + RE | **EN**      | ESP32 **GPIO4** (driven LOW = receive-only)                     |
| VCC     | VCC         | ESP32 **3.3V**                                                  |

> **Board labels are from the module's perspective:** its `TXD` (data the module
> outputs to the MCU) is the transceiver's `RO` and connects to the ESP32 **RX**
> (GPIO16); its `RXD` is `DI` and connects to the ESP32 **TX** (GPIO17). If you
> get no frames, this RX/TX pairing is the first thing to swap.

Power the ESP32 from your computer's USB port during debugging. **A common ground
between the ESP32 and the DVS bus is required** for the RS485 signalling to be valid.

Pin assignments live in [`include/config.h`](include/config.h).

> ⚠️ The DVS connector also carries +12 V and +30 V. Do **not** connect those to the
> ESP32 - only the two RS485 lines and ground go to the MAX3485.

## Project layout

```
platformio.ini                  PlatformIO envs: esp32dev (firmware) + native (tests)
include/
  config.h                      Pins, baud, gap threshold, register + control table, MQTT topics
  crc16.h                       CRC16-CCITT API
  frame.h                       Frame struct, CRC verify, decode + formatter, frame builder
  control.h                     Fan control + auto-ventilation API
  bus.h                         RS485 transmit seam (dvsBusSend / poll-cycle reset)
  parse.h                       Shared argument parsers: on/off/toggle + strict numeric/mode/profile (console + MQTT)
  profile.h                     Pure summer/winter speed-vs-temperature logic + fail-safe freshness gate
  net.h                         Wi-Fi = MQTT bridge API
  secrets.h                     Wi-Fi / MQTT credentials (git-ignored; copy the .example)
  secrets.h.example             Template for secrets.h
src/
  crc16.cpp                     CRC implementation (switchable variant)
  frame.cpp                     Frame decoding / line formatting / frame building
  profile.cpp                   Ventilation profile maths (pure, host-tested)
  control.cppp                  Fan control + auto-ventilation policy (Arduino only)
  net.cpp                       Wi-Fi + MQTT client (esp32 only)
  main.cpp                      Sniffer loop + USB console + RS485 bus/transmit (Arduino only)
test/
  test_crc/test_crc.cpp         Host-side CRC unit tests (Unity)
  test_profile/test_profile.cpp Host-side profile-logic + fail-safe tests (Unity)
  test_parse/test_parse.cpp     Host-side argument-parser tests (Unity)
```

## Building & flashing

PlatformIO does the work on a machine that has it installed (this project is
authored without a local toolchain).

```sh
# Run the host-side unit tests (CRC, frame, profile, parsers) - no hardware needed
pio test -e native

# Provide Wi-Fi + MQTT credentials (git-ignored) before building the firmware
cp include/secrets.h.example include/secrets.h
# ...then edit include/secrets.h with your SSID, password and broker

# Build and flash the firmware
pio run
pio run -t upload

# Watch the console (frames are quiet by default; type `trace on` to stream them)
pio device monitor
```

## Using the sniffer

Open the serial monitor at **115200 baud**. On boot you'll see a banner and the
current configuration. The firmware runs **quiet by default** - lifecycle and
error events (`[wifi]`, `[mqtt]`, `[err]`, `[auto]`) still log, but individual
frames are not printed. Type `trace on` to stream every captured frame as a
single line, for example:

```
1234567us len=7 01 10 34 12 00 9A F1 CRC=OK addr=0x01 reg=0x10 (temperature) temp_raw=4660
```

Fields: timestamp (µs), byte count, raw hex, CRC result, address, register (name),
and any register-specific decode.

### Console commands

Type these into the serial monitor to tune capture and confirm the protocol from
live data:

| Command       | Effect                                                          |
| ------------- | --------------------------------------------------------------- |
| `gap <us>`    | Inter-frame idle gap that marks a frame boundary                |
| `trace [on\|off]` | Stream decoded frames to the console (off by default = quiet) |
| `decode`         | Toggle register decoding vs. raw hex only (when tracing) |
| `badonly [on\|off\|reset]`  | Show only CRC=BAD frames / reset the counter      |
| `tx [on\|off]` | Enable/disable transmit – `off` => listen-only, `on` => listen+send |
| `on`          | Power the fan on (last used / default speed) – transmits        |
| `off`         | Power the fan off (stop) – transmits                            |
| `speed <n>`   | Set target fan speed `0..84` – transmits                        |
| `mode [manual\|auto]` | Manual control vs. automatic summer/winter profiles |
| `profile [summer\|winter\|auto]` | Select the ventilation profile (`auto` picks by inside temp) |
| `heatrecovery [on\|off]` | Winter warm-roof speed boost on/off/toggle           |
| `inside <°C>` | Feed the inside temperature used by auto control |
| `status` | Print the current configuration (incl. mode/profile/inside) |
| `help` | List commands |

## Automatic ventilation profiles

Beyond manual control, the firmware can drive the fan itself from temperature —
an on-device **summer/winter profile controller**. The decision logic is pure and
host-tested (see [`include/profile.h`](include/profile.h) /
[`src/profile.cpp`](src/profile.cpp)). This unit is positive-pressure roof air
with **no heat-recovery core**, so the air entering the house is the **raw roof
temperature** — the profiles work directly on `roof` vs `inside`, with no
tempering blend.

**Inputs.** Roof temperature is the sniffed `0x10` reading. The in-home
temperature is **not on the bus**, so you feed it in over MQTT
(`dvs/inside_temp/set`, e.g. from a Home Assistant sensor) or with `inside <°C>`
on the console.

**Profiles.**

- **Winter (moisture-first, optional heat recovery)** - **always** ventilates
  for moisture/condensation control whenever the roof air is above the cold
  cutoff (`minSupply`), at a low baseline speed (`moistureBaseSpeed`). When
  **heat recovery** is on (the default) and the roof is as warm as - or warmer
  than - inside, it ramps the speed up proportionally to harvest that free heat:
  the boost begins around a "similar" differential (`recoverStart`) and reaches
  max once the roof is `recoverFull` (≥ 4 °C) warmer than inside. Once the house
  is warm (`comfortHigh`) the boost is capped and the fan holds the moisture
  baseline. With heat recovery off it just holds that baseline. Only the cold
  cutoff (`minSupply`) stops it.
- **Summer (free cooling)** — ventilates when the roof/night air is cooler than
  inside; blocks hot midday roof air above `maxSupply`, and stops once the house
  is already cool (`comfortLow`).
- **Auto** — picks summer or winter each tick from the inside temperature
  (around `comfortTarget`), for the shoulder seasons. You can instead force the
  profile from a Home Assistant seasonal automation.

**Engaging it.** Auto control only drives the fan when **all** of these hold:

1. `mode auto` — the default is `manual`, and *selecting a profile alone does
   nothing*;
2. `tx on` — in listen-only the controller is inert and the fan is left to the
   wall controller; and
3. a **fresh** roof temperature and an inside temperature received at least once
   - if the sniffed roof reading is older than `DVS_TEMP_STALE_MS`, or no inside
   temperature has ever arrivedd, the controller **fails safe to off**. The inside
   temperature is not age-checked (Home Assistant publishes it on-change only, so
   a steady house legitimately sends no updates) but it must exist - otherwise the
   profile maths would read a missing value as 0 °C and command a bogus boost.

It recomputes every `DVS_CONTROL_TICK_MS`. Example: `winter` at roof 7 °C (below
the 12 °C floor) commands the fan **off**; above 12 °C the moisture baseline
holds `moistureBaseSpeed`, climbing proportionally (heat recovery) as the roof
rises past the inside temperature toward `recoverFull`.

**Setpoints** (boot defaults tuned for Wellington NZ; all in
[`include/config.h`](include/config.h) and tunable live over MQTT/console):

| Setting | Default | Meaning |
| -------- | -------: | -------- |
| `DVS_PROFILE_COMFORT_TARGET_C` | 19 | `auto` summer/winter switch point |
| `DVS_PROFILE_COMFORT_HIGH_C` | 24 | Winter: inside ceiling - caps the heat-recovery boost (moisture baseline still runs above it) |
| `DVS_PROFILE_COMFORT_LOW_C` | 18 | Summer: stop once inside is this cool |
| `DVS_PROFILE_MIN_SUPPLY_C` | 12 | Winter: never supply roof air colder than this |
| `DVS_PROFILE_MAX_SUPPLY_C` | 24 | Summer: never supply roof air hotter than this |
| `DVS_PROFILE_DEADBAND_ON_C` / `_OFF_C` | 1.5 / 0.5 | Summer start / stop hysteresis |
| `DVS_PROFILE_SPAN_C` | 5 | Summer: differential at which speed reaches max |
| `DVS_PROFILE_RECOVER_START_C` / `_FULL_C` | -2 / 4 | Winter heat-recovery ramp (roof - inside): boost start / full-speed differential |
| `DVS_PROFILE_MIN_SPEED` / `_MAX_SPEED` | 20 / 74 | Auto-mode speed range; shared summer floor + winter baseline minimum, with the max held below the 84 hardware boost for headroom |
| `DVS_PROFILE_MOISTURE_SPEED` | 20 | Winter always-on moisture-ventilation baseline (shares the auto minimum) |
| `DVS_HEAT_RECOVERY_DEFAULT` | on | Winter warm-roof speed boost |

> The **Control model** section above describes the *original wall controller*
> (Winter/Summer/Uniform/General, Retain Home Heat ON/OFF, airflow offset) as
> reverse-engineered from captures — it is background, not what this firmware's
> `auto` mode implements.

## Wi-Fi & MQTT control

With `include/secrets.h` filled in, the firmware joins your Wi-Fi (station mode)
and connects to an MQTT broker over **TLS** (Mosquitto with `require_certificate
false`, so no client certificate is needed). It publishes fan state and sniffed
sensor values, and subscribes to command topics so you can drive the fan (e.g.
from Home Assistant). Credentials, broker and the TLS CA live in
`include/secrets.h`; the topic tree and timing are in
`include/config.h` (`include/config.h`).

**TLS trust:** set `MQTT_CA_CERT` in `secrets.h` to the broker's CA certificate
to verify the server. Left empty, the client uses `setInsecure()` – the link is
still encrypted but the server is not authenticated (acceptable only on a trusted
LAN). `MQTT_PORT` defaults to `8883` (the Mosquitto TLS port).

**Listen vs. send:** the firmware boots in the mode set by
`DVS_TX_ENABLED_DEFAULT` (listen-only by default). Flip it at runtime with the
`tx` console command or the `dvs/tx/set` topic. In listen-only mode every control
command is ignored and the bus is never driven – it is a pure sniffer.

The base topic is `dvs`:

| Topic                   | Dir | Payload                      | Meaning                                                                    |
| ----------------------- | --- | ---------------------------- | -------------------------------------------------------------------------- |
| `dvs/status`            | out | `online`/`offline`           | Availability (retained, MQTT LWT)                                          |
| `dvs/tx/set`            | in  | `ON` / `OFF`                 | Enable send (`ON`) or listen-only (`OFF`)                                  |
| `dvs/tx/state`          | out | `ON` / `OFF`                 | Current transmit mode (retained)                                           |
| `dvs/power/set`         | in  | `ON` / `OFF`                 | Turn the fan on or off                                                     |
| `dvs/power/state`       | out | `ON` / `OFF`                 | Commanded power state (retained)                                           |
| `dvs/speed/set`         | in  | `0`..`84`                    | Set target fan speed                                                       |
| `dvs/speed/state`       | out | `0`..`84`                    | Commanded target speed (retained)                                          |
| `dvs/speed/actual`      | out | `0`..`84`                    | Sniffed actual (ramping) speed                                             |
| `dvs/temperature`       | out | e.g. `17.4`                  | Sniffed roof-space temperature (°C)                                        |
| `dvs/inside_temp/set`   | in  | e.g. `18.5`                  | Inside temperature for auto control (feed from HA)                         |
| `dvs/inside_temp/state` | out | e.g. `18.5`                  | Last inside temperature received (retained)                                |
| `dvs/mode/set`          | in  | `manual`/`auto`              | Manual control vs. automatic profiles                                      |
| `dvs/mode/state`        | out | `manual`/`auto`              | Current control mode (retained)                                            |
| `dvs/profile/set`       | in  | `summer`/`winter`/`auto`     | Selected ventilation profile                                               |
| `dvs/profile/state`     | out | `summer`/`winter`/`auto`     | Current profile (retained)                                                 |
| `dvs/heat_recovery/set`     | in  | `ON` / `OFF`             | Winter warm-roof speed boost                                               |
| `dvs/heat_recovery/state`   | out | `ON` / `OFF`             | Heat-recovery state (retained)                                             |
| `dvs/status/reg/NN`     | out | hex bytes                    | Raw payload of status register `0xNN` (`01`, `06`, `12`, `13`)             |

State topics are (re)published immediately when a command is applied and again
every `STATE_PUBLISH_MS` (default 15 s) so clients that reconnect between
commands still see current values.

### Home Assistant discovery

On each MQTT connect the firmware publishes retained [MQTT discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
configs, so Home Assistant auto-creates everything under a single **DVS G3**
device - no YAML required:

- a **fan** entity (on/off + speed as a percentage over the `1..84` range),
- **sensors** for roof temperature and actual fan speed,
- a **switch** for transmit mode (listen-only vs listen+send),
- **selects** for control mode (`manual`/`auto`) and profile (`summer`/`winter`/`auto`),
- a **switch** for winter heat recovery (warm-roof speed boost),
- a **number** for the inside temperature fed into auto control,
- **diagnostic sensors** for each status-block register (`0x01/0x06/0x12/0x13`).

Availability follows the `dvs/status` LWT, so entities show *unavailable* when the
device drops off. Configuration lives in [`include/config.h`](include/config.h):
set `HA_DISCOVERY_ENABLED` to `false` to disable it, change `HA_DISCOVERY_PREFIX`
if your HA uses a non-default discovery prefix, or edit `HA_DEVICE_NAME`. The
PubSubClient buffer is raised to 768 bytes in `netSetup()` because the discovery
JSON exceeds the 256-byte default.


How control maps onto the bus:

- **Speed** -> write register `0x15` (1-byte payload) to the fan address (`0x02`).
- **Power off** -> write register `0x15` with `0`. Live captures show the wall
  controller has **no separate stop register** (`0x05`) -- "off" is just speed 0,
  and the fan ramps down on its own.
- **Power on** -> re-send the last non-zero speed (or `DVS_DEFAULT_SPEED`).
- **Keep-alive / polling** -> in transmit mode the firmware runs a response-driven
  **poll cycle**: it alternates a `0x15` write (the keepalive -- "off" is just
  target `0`; there is **no `0x03` deadman**) and a `0x10` temperature read, one
  request outstanding at a time, resending once if no reply arrives (the
  controller's doubled frame) and advancing after the reply or a timeout. Keeping
  the two conversations 1:1 is what stops the fan locking onto the keepalive and
  ignoring the temperature query. Timing lives in `DVS_POLL_RESEND_MS`,
  `DVS_POLL_REPLY_TIMEOUT_MS` and `DVS_POLL_GAP_MS`.

The same actions are available over USB via the `on`, `off` and `speed <n>`
console commands, which is the easiest way to test the transmit path before
wiring up MQTT.

> Confirmed by capture: on/off/speed all go through register `0x15`; there is no
> `0x03` deadman or `0x05` stop in the observed controller traffic. A periodic
> burst of `0x01` / `0x06` / `0x12` / `0x13` (~every 30 s) is an as-yet undecoded
> status/config poll.

### CRC variant (confirmed)

The DVS bus uses **CRC-16/AUG-CCITT**, confirmed by brute-forcing captured
frames:

| Parameter   | Value                      |
| ----------- | -------------------------- |
| Polynomial  | `0x1021`                   |
| Init        | `0x1D0F`                   |
| Reflection  | none (in or out)           |
| Xor-out     | `0x0000`                   |
| CRC on wire | **big-endian** (MSB first) |

These are the firmware defaults, so frames should show `CRC=OK` out of the box.
The varient is now baked in as a fixed constant; the runtime CRC-sweep commands
used to derive it have been removed. To re-derive it from a fresh capture,
temporarily make `g_crcParams` / `g_crcBigEndian` adjustable again.

## Capturing commands

1. Connect the sniffer and open the monitor.
2. Change settings on the wall controller (fan speed, modes, etc.).
3. Record which frames appear for each action - the `addr`/`reg`/payload reveal the
   command; `CRC=OK` confirms correct framing.
4. For status registers, use the fan's responses: warm the sensor (e.g. a hair
   dryer) to watch `0x10` change, and vary speed to watch `0x15`.
