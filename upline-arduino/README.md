# Upline (Arduino)

**The simplest serial protocol.** Newline-delimited ASCII key/value records for microcontrollers.

```text
^tempf|78.35~rgb|16711680^
```

Publish state, accept commands, and describe yourself — over UART, USB, or Bluetooth serial.
The full wire format is the protocol spec at the root of this repo: [`../README.md`](../README.md).

## Install

Copy this folder into your Arduino `libraries/` directory (rename it `Upline` if you like — the
IDE shows the `name` from `library.properties` either way), then:

```cpp
#include <upline.hpp>
```

Header-only. No dependencies, no allocation, no libc, no floating point.

## Quick start

```cpp
#include <upline.hpp>

UPLINE_SCHEMA(mySchema,
  "uuid|bench-1~name|Thermostat~desc|Bench rig~ver|1"
  "~temp|fix2|r||-40.00|125.00"
  "~fan|bool|rw|0");

Upline upline(Serial, mySchema);

void uplineOnKeyValPair(const char* key, const char* value, bool isFlag) {
  if (!strcmp(key, "fan")) digitalWrite(FAN_PIN, value[0] == '1');
}

void setup() { Serial.begin(115200); upline.onPair(uplineOnKeyValPair); }

void loop() {
  upline.poll();                       // reads, answers "?", heartbeats every 2.5 s
  if (timeForAReading()) {
    upline.beginRecord();
    upline.addFixed("temp", degreesTimes100(), 2);
    upline.addBool("fan", fanIsOn());
    upline.endRecord();
  }
}
```

A host opens the port, waits for a heartbeat, sends `^?^`, and gets back every key with its
type, access, default, and range — enough to build a whole interface for a board it has never seen.

## Examples

| Example | Board | Shows |
|---|---|---|
| `Basic` | any, including Uno | Counter out, LED in, and a `reset` command. The smallest useful device — start here. |
| `ATtiny85_ServoLedTemp` | ATtiny85 @ 16 MHz PLL | Rate-limited servo that releases when parked, plus an LED in and uncalibrated die temperature out. 8 KB flash, 512 B RAM, no hardware UART — and it still self-describes. Also worked examples of the things that bite on a part this small: fuses, oscillator trim for a bit-banged UART at 115200, and brown-out choices under a servo load. |
| `ItsyBitsyM4_RgbAndTemp` | Adafruit ItsyBitsy M4 | Onboard RGB DotStar as a writable 24-bit `int`, SAMD51 die temperature as a read-only `fix2` interpolated from the factory calibration row. |

All three echo a command back immediately with just the key it touched, so a host never waits on
the next telemetry tick to see its setting take effect.

Built sizes: `Basic` 3,674 B (11%) on an Uno · `ATtiny85_ServoLedTemp` 4,114 B / 262 B
(50% / 51%) · `ItsyBitsyM4_RgbAndTemp` 13,292 B (2%) on the M4.

## Size

A complete device — codec, `fixN`, base64url, descriptor, heartbeat, telemetry, 128-byte
receive buffer:

| Board | Flash | SRAM |
|---|---|---|
| Uno / ATmega328P | 3,900 B (12%) | 364 B (17%) |
| Duemilanove / ATmega168 | 3,900 B (27%) | 364 B (35%) |
| Arduino NG / ATmega8 | 3,684 B (51%) | 364 B (35%) |
| ATtiny85 | 3,262 B (39%) | 259 B (50%) |
| ATtiny85, transmit-only | 2,644 B (32%) | 126 B (24%) |

Anything you never call is never linked, so you do not have to disable features you are not
using. See the configuration block at the top of `src/upline.hpp` for the knobs that do change
the build — chiefly `UPLINE_TRANSMIT_ONLY` and `UPLINE_RX_BUFFER_SIZE`.

## Platforms

Compiled clean on AVR (ATmega328P, ATmega168, ATmega8, ATtiny85, ATtiny1614), SAMD21/SAMD51,
RP2040, RP2350 (ARM and RISC-V), ESP32 / S3 / C3, and Teensy 4.1.

Off Arduino, supply a port with `available()`, `read()`, `write(uint8_t)` and define
`UPLINE_MILLIS()`.

## Licence

CC0 1.0 Universal — public domain. See [`LICENSE`](LICENSE).
