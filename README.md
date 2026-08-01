# DVS G3 RS485 Sniffer

Passive RS485 bus sniffer for a **DVS G3 (DVS-G3 BC220)** home ventilation system,
built on a **D1 Mini ESP32** and a **MAX3485** transceiver.

It listens (receive-only) on the DVS bus *in parallel with the existing wall
controller*, groups the serial bytes into frames, verifies the CRC and prints a
decoded summary of every frame over USB. It can also **connect to Wi-Fi and MQTT**
to expose the fan as a controllable device - turn it on/off and set the fan speed
- while publishing the sniffed roof-space temperature and actual fan speed.

Protocol work is based on Ian Rees' write-up,
[*Reverse engineering the DVS bus*](https://ianrees.nz/tech/2018-07-11-reverse-engineering-the-dvs-bus.html).

> **Status:** listen-only sniffer + decoder, plus an **experimental control path**
> (Wi-Fi + MQTT) that transmits on the bus to command power and fan speed.
>
> ⚠️ **Transmitting is active and shares the bus with the wall controller.** Frame
> collisions are possible and the exact payloads for some registers (deadman, stop)
> are still inferred, not confirmed. Only enable control on a system you can
> recover, and keep the wall controller handy. The sniffer half remains passive.

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
| `0x03`   | Deadman - write occasionally to keep the fan spinning            |
| `0x04`   | Set address - one payload byte becomes the peripheral address    |
| `0x05`   | Stop                                                             |
| `0x10`   | Roof-space temperature - payload byte 0 is a raw count; linear over 11-20 °C `temp_C = 0.06807·raw - 1.039` (≈14.7 counts/ °C) |
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

**Temperature scaling.** Payload byte 0 of `0x10` is a raw roof-space sensor
count. A least-squares fit to 8 live points over 11–20 °C is linear (residuals
< 0.75 °C, limited by the controller's integer display):
`temp_C = 0.06807·raw - 1.039` (≈ 14.7 counts/°C). Readings at temperature
extremes are still needed to confirm the fit holds outside that range.

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
platformio.ini             PlatformIO envs: esp32dev (firmware) + native (tests)
include/
  config.h                 Pins, baud, gap threshold, register + control table, MQTT topics
  crc16.h                  CRC16-CCITT API
  frame.h                  Frame struct, CRC verify, decode + formatter, frame builder
  control.h                Fan control API (power / speed)
  net.h                    Wi-Fi = MQTT bridge API
  secrets.h                Wi-Fi / MQTT credentials (git-ignored; copy the .example)
  secrets.h.example        Template for secrets.h
src/
  crc16.cpp                CRC implementation (switchable variant)
  frame.cpp                Frame decoding / line formatting / frame building
  net.cpp                  Wi-Fi + MQTT client (esp32 only)
  main.cpp                 Sniffer loop + USB console + RS485 transmit/control (Arduino only)
test/
  test_crc/test_crc.cpp    Host-side unit tests (Unity)
```

## Building & flashing

PlatformIO does the work on a machine that has it installed (this project is
authored without a local toolchain).

```sh
# Run the host-side CRC / frame tests - no hardware needed
pio test -e native

# Provide Wi-Fi + MQTT credentials (git-ignored) before building the firmware
cp include/secrets.h.example include/secrets.h
# ...then edit include/secrets.h with your SSID, password and broker

# Build and flash the firmware
pio run
pio run -t upload

# Watch captured frames (timestamps enabled via monitor_filters)
pio device monitor
```

## Using the sniffer

Open the serial monitor at **115200 baud**. On boot you'll see a banner and the
current configuration. Each captured frame is printed as a single line, for example:

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
| `decode`      | Toggle register decoding vs. raw hex only                       |
| `badonly [on\|off\|reset]`  | Show only CRC=BAD frames / reset the counter      |
| `tx [on\|off]` | Enable/disable transmit – `off` => listen-only, `on` => listen+send |
| `on`          | Power the fan on (last used / default speed) – transmits        |
| `off`         | Power the fan off (stop) – transmits                            |
| `speed <n>`   | Set target fan speed `0..84` – transmits                        |
| `status`      | Print the current configuration                                 |
| `help`        | List commands                                                   |

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

| Topic | Dir | Payload | Meaning |
| :--- | :--- | :--- | :--- |
| `dvs/status` | out | `online`/`offline` | Availability (retained, MQTT LWT) |
| `dvs/tx/set` | in | `ON`/`OFF` | Enable send (`ON`) or listen-only (`OFF`) |
| `dvs/tx/state` | out | `ON`/`OFF` | Current transmit mode (retained) |
| `dvs/power/set` | in | `ON`/`OFF` | Turn the fan on or off |
| `dvs/power/state` | out | `ON`/`OFF` | Commanded power state (retained) |
| `dvs/speed/set` | in | `0..84` | Set target fan speed |
| `dvs/speed/state` | out | `0..84` | Commanded target speed (retained) |
| `dvs/speed/actual` | out | `0..84` | Sniffed actual (ramping) speed |
| `dvs/temperature` | out | e.g. `17.4` | Sniffed roof-space temperature (°C) |
| `dvs/status/reg/NN` | out | hex bytes | Raw payload of status register `0xNN` (`01`, `06`, `12`, `13`) |

State topics are (re)published immediately when a command is applied and again
every `STATE_PUBLISH_MS` (default 15 s) so clients that reconnect between
commands still see current values.

### Home Assistant discovery

On each MQTT connect the firmware publishes retained [MQTT discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
configs, so Home Assistant auto-creates everything under a single **DVS G3**
device – no YAML required:

- a **fan** entity (on/off + speed as a percentage over the `1..84` range),
- **sensors** for roof temperature and actual fan speed,
- a **switch** for transmit mode (listen-only vs listen+send),
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
