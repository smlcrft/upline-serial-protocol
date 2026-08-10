# Upline (Arduino)

**The simplest serial protocol.** Newline-delimited ASCII entries for microcontrollers.

```text
^tempf~78.35^rgb~16711680^
```

Publish state, accept reads and writes, and describe yourself — over UART, USB, or
Bluetooth serial. The full wire format is the spec at the root of this repo:
[`../README.md`](../README.md).

## Install

Copy this folder into your Arduino `libraries/` directory (rename it `Upline` if you like —
the IDE shows the `name` from `library.properties` either way), then:

```cpp
#include <upline.hpp>
```

Header-only. No dependencies, no allocation, no libc, no floating point.

## Quick start

```cpp
#include <upline.hpp>

UPLINE_SCHEMA(mySchema,
  "_i|bench-1~_n|Thermostat~_d|Bench rig"
  "~temp|r|fix2||-40.00|125.00"
  "~fan|rw|bool|0");

Upline upline(Serial, mySchema);

void uplineOnEntry(const UplineEntry& entry) {
  if (entry.isRead()) return;                       // ^fan^ asks; it does not set
  if (!strcmp(entry.key, "fan")) digitalWrite(FAN_PIN, entry.value(0)[0] == '1');
}

void setup() { Serial.begin(115200); upline.onEntry(uplineOnEntry); }

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

A host opens the port, waits for a heartbeat, sends `^?^`, and gets back the device's
identity, its receive-buffer limit, and every key with its mode, type, default, and range —
enough to build a whole interface for a board it has never seen.

`_v` and `_r` are filled in by the macro from `UPLINE_VERSION` and
`UPLINE_RX_BUFFER_SIZE`, so the advertised buffer can never drift out of sync with the real
one — and a `UPLINE_TRANSMIT_ONLY` build declares `_r|0` by itself.

The `?` response is one constant in flash. Because `_i`, `_n`, `_d`, `_v`, and `_r` only
ever travel device→host, a device answers the whole of `^?^` with a single write and never
implements a read handler for any of them.

## Reads, writes, and commands

One rule covers all three: **an entry with no values is a read, an entry with values is a
write**, and what a key does is whatever it declared (spec §6).

```text
^fan^            read it            -> ^fan~1^
^fan~1^          write it           -> ^fan~1^      the echo is the acknowledgement
^fan~^           write the empty string
^reset^          execute, if `reset` declared mode x
^beep~440~0.5^   execute with arguments
```

A refused or clamped write is echoed with the value **still in force**, which is how a host
tells a rejected write from a lost line.

## Examples

| Example | Board | Shows |
|---|---|---|
| `Basic` | any, including Uno | Counter out, LED in and out, a `reset` command. The smallest useful device — start here. |
| `ATtiny85_ServoLedTemp` | ATtiny85 @ 16 MHz PLL | Rate-limited servo that releases when parked, plus an LED and uncalibrated die temperature. 8 KB flash, 512 B RAM, no hardware UART — and it still self-describes. Also the things that bite on a part this small: fuses, oscillator trim for a bit-banged UART at 115200, and brown-out choices under a servo load. |
| `ItsyBitsyM4_RgbAndTemp` | Adafruit ItsyBitsy M4 | Onboard RGB DotStar as a writable 24-bit `int`, SAMD51 die temperature as a read-only `fix2` interpolated from the factory calibration row. |

All three echo a write back immediately with just the key it touched, so a host never waits
on the next telemetry tick to see its setting take effect.

## Size

Built with `arduino-cli`, current examples:

| Example | Board | Flash | SRAM |
|---|---|---|---|
| `Basic` | Uno / ATmega328P | 4,060 B (12%) | 358 B (17%) |
| `ATtiny85_ServoLedTemp` | ATtiny85 | 4,426 B (54%) | 262 B (51%) |
| `ItsyBitsyM4_RgbAndTemp` | ItsyBitsy M4 | 13,168 B (2%) | — |

Anything you never call is never linked, so you do not have to disable features you are not
using. See the configuration block at the top of `src/upline.hpp` for the knobs that do
change the build — chiefly `UPLINE_TRANSMIT_ONLY`, `UPLINE_RX_BUFFER_SIZE`, and the
`UPLINE_MAX_VALUES` / `UPLINE_MAX_SUBVALUES` pair that bounds how complex a single inbound
entry may be.

## Conformance

The library is checked against the spec's own vectors
([`../vectors/vectors.tsv`](../vectors/vectors.tsv), generated from §13 of the spec). Every vector that is
mechanically checkable on a host passes — framing, escaping, entry shape, the reserved
namespace, and the checked number parser.

## Platforms

Compiled clean on AVR (ATmega328P, ATmega168, ATmega8, ATtiny85, ATtiny1614), SAMD21/SAMD51,
RP2040, RP2350 (ARM and RISC-V), ESP32 / S3 / C3, and Teensy 4.1.

Off Arduino, supply a port with `available()`, `read()`, `write(uint8_t)` and define
`UPLINE_MILLIS()`.

## Licence

CC0 1.0 Universal — public domain. See [`LICENSE`](LICENSE).
