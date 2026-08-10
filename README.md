# Upline

**The simplest serial protocol.** Newline-delimited ASCII entries for microcontrollers.

```text
^tempf~78.35^rgb~16711680^servo~90.00^
```

**Status:** v1 (`_v~1`)

Upline gives any serial-capable device a predictable way to publish state, accept commands, and describe itself — cheaply enough to run on a 20-year-old 8-bit microcontroller, and structured enough that a host can build a complete interface for a device it has never seen before.

---

> **You do not have to implement this yourself.**
>
> **[`upline-arduino/`](upline-arduino/) is a complete, tested implementation** — one header, drop it in, and any Arduino or C++-capable board speaks Upline:
>
> ```cpp
> #include <upline.hpp>
> ```
>
> - **Small.** A complete device — codec, `fixN`, base64url, schema, heartbeat, telemetry, validated numeric input, 128-byte receive buffer — is **4,060 B of flash and 358 B of SRAM** on an Uno, and fits an **ATtiny85** with 8 KB flash and 512 B of RAM. Transmit-only, the same device is 2,100 B and 112 B.
> - **Optimized.** Header-only. No allocation, no libc, no floating point, no `String`. Anything you never call is never linked, so unused features cost nothing.
> - **Tested.** Compiled clean on AVR (ATmega8/168/328P, ATtiny85, ATtiny1614), SAMD21/SAMD51, RP2040, RP2350 (ARM and RISC-V), ESP32 / S3 / C3, and Teensy 4.1. Every conformance vector in §13 that is checkable on a host passes.
> - **Safe with bad input.** A corrupted value is refused, not coerced — `^servo~1a0^` frames perfectly and would otherwise become a setpoint of zero (§8.4).
> - **Not Arduino-only.** Supply any port with `available()`, `read()`, and `write(uint8_t)`, then define `UPLINE_MILLIS()`.
>
> Three worked examples ship with it, including a servo-and-temperature node on an ATtiny85 and the onboard RGB plus die temperature on an ItsyBitsy M4. CC0, like this spec.
>
> **The rest of this document is the wire format** — read it to write your own implementation, to build the host side, or to check a detail. You do not need it to put an Upline device on a bench.

---

## Contents

- [1. At a glance](#1-at-a-glance)
- [2. Transports](#2-transports)
- [3. Framing](#3-framing)
- [4. Escaping](#4-escaping)
- [5. Entries](#5-entries)
- [6. Operations](#6-operations)
  - [6.1 Acknowledging a write](#61-acknowledging-a-write)
- [7. Reserved keys](#7-reserved-keys)
- [8. The schema](#8-the-schema)
  - [8.1 Declarations](#81-declarations)
  - [8.2 Arguments](#82-arguments)
  - [8.3 Types](#83-types)
  - [8.4 Parsing `int` and `fixN`](#84-parsing-int-and-fixn)
  - [8.5 Device-side arithmetic](#85-device-side-arithmetic--outside-the-protocol)
- [9. Heartbeat](#9-heartbeat)
- [10. Limits](#10-limits)
  - [10.1 The drain deadline](#101-the-drain-deadline)
  - [10.2 Baud rates](#102-baud-rates)
  - [10.3 Discovering the rate](#103-discovering-the-rate)
- [11. Building on Upline](#11-building-on-upline)
  - [11.1 Schema to interface](#111-schema-to-interface)
  - [11.2 State, identity, and liveness](#112-state-identity-and-liveness)
  - [11.3 Logging and replay](#113-logging-and-replay)
  - [11.4 What Upline deliberately does not provide](#114-what-upline-deliberately-does-not-provide)
- [12. Footprint](#12-footprint)
- [13. Conformance vectors](#13-conformance-vectors)
- [14. Status](#14-status)
- [Appendix A — why not JSON or MQTT](#appendix-a--why-not-json-or-mqtt)
- [Appendix B — prior art](#appendix-b--prior-art)
- [References](#references)

---

## 1. At a glance

- **A line is a list of independent entries.** Each entry is one operation on one key.
- **No positional rules.** No "first entry is special." An entry means the same thing wherever it sits.
- **Four reserved characters** — `^ ~ | \` — plus the line terminator.
- **Complete escaping, including newline.** Six sequences, no forbidden-character lists.
- **Self-describing.** `^?^` returns identity, limits, and the full schema.
- **Writes are acknowledged by echo** — with the value actually in force, so a refused
  write is distinguishable from a lost one.
- **Liveness built in.** A heartbeat proves the device is alive and speaks Upline.
- **No floating point, anywhere.** Decimals are `fixN`.

**Measured cost of a complete device** — everything on: codec, `fixN`, base64url, schema in
flash, heartbeat, telemetry, validated numeric input, and a 128-byte receive buffer. This is
the bundled `Basic` example, so it is reproducible from this repo with `arduino-cli`:

| Board | Chip | Flash | of | SRAM | of |
|---|---|---|---|---|---|
| Uno / Duemilanove-328 | ATmega328P | 4,060 B | 12% of 32,256 | 358 B | 17% of 2,048 |
| **Duemilanove (2008)** | ATmega168 | **4,060 B** | **28% of 14,336** | 358 B | 34% of 1,024 |
| Arduino NG (2005) | ATmega8 | 3,792 B | 52% of 7,168 | 358 B | 34% of 1,024 |
| ATtiny85 | ATtiny85 | 3,168 B | 38% of 8,192 | 253 B | 49% of 512 |
| ATtiny85 (Tx only) | ATtiny85 | 2,100 B | 25% of 8,192 | 112 B | 21% of 512 |

The last row is the transmit-only profile (§9) at the same workload: no receive path, so the
parser and the receive buffer are gone, the device declares `_r|0` on its own, and it
broadcasts its schema instead of waiting to be asked.

---

## 2. Transports

Upline requires exactly one thing: **an ordered stream of bytes.** It assumes no packet boundaries, no maximum transmission unit, no flow control, no control lines, and no reliability guarantee.

That makes it usable over, among others:

| Transport | Notes |
|---|---|
| **UART / TTL serial** | The baseline. See §10 for timing obligations. |
| **USB CDC-ACM** | Adds a link-layer CRC and retransmission, and real backpressure. |
| **USB-to-serial bridges** (FTDI, CH340, CP210x) | Behave as a bare UART to the device; no backpressure. |
| **Bluetooth Classic SPP / RFCOMM** | A byte stream by definition. Works unmodified. |
| **BLE UART services** (Nordic NUS and equivalents) | Packet-oriented with a small MTU, so lines arrive fragmented. Harmless: reassembly is "accumulate until a line terminator." |
| **RS-232 / RS-422** | Unmodified. |
| **RS-485** | Works point-to-point. On a multi-drop bus see the caveat below. |
| **TCP, PTYs, pipes, files** | A captured Upline stream replays as a valid stream. |

**Fragmentation is a non-issue.** Because lines are terminator-delimited and every entry is bounded by carets on both sides (§3), a receiver never needs to know how bytes were chunked. A line split across three BLE notifications reassembles the same as one delivered whole.

**Loss is survivable; reordering is not.** Delivery is best-effort (§3): a corrupted line loses whatever entries did not arrive whole, the next terminator restores sync, and the heartbeat (§9) bounds how long a receiver waits. But Upline assumes bytes arrive **in order and without interleaving from other senders**. A transport that may reorder or interleave must be wrapped in something that does not.

**Multi-drop buses need arbitration.** Upline is point-to-point by default, and a device transmits unprompted — the heartbeat and any telemetry are not solicited. On a shared bus such as RS-485, either suppress unsolicited transmission and poll instead, or arbitrate the bus above Upline. The protocol defines no addressing and no collision handling.

**8-bit clean is not required.** Every byte Upline emits is printable ASCII or a UTF-8 continuation byte, so it survives transports that mangle control characters, and it stays readable in any terminal or logger along the path.

---

## 3. Framing

```abnf
stream   = *( line eol )
eol      = CRLF / CR / LF          ; any of the three; CRLF is ONE terminator
line     = record / foreign
foreign  = *OCTET                  ; any line whose first byte is not "^"; ignored
record   = "^" [ entry *( "^" entry ) ] "^"
```

A receiver MUST:

1. Split the stream on **LF (0x0A) or CR (0x0D)**. CRLF terminates one line, not two.
   Empty lines are ignored.
2. Ignore any line whose first byte is not `^`. **Not an error** — foreign traffic or
   human-readable debug output sharing the link.
3. Split on **unescaped** `^`. **An entry is the text between two carets, and both must be
   seen before it is dispatched.** The leading caret proves the entry began where the
   receiver thinks it did; the trailing caret proves it arrived whole. Anything with only
   one boundary may be a partial receive and MUST NOT be acted on. Empty entries are
   ignored, so `^^` is a valid line carrying nothing.
4. Bytes after the final caret therefore form an **incomplete entry**. Drop that fragment;
   every fully bounded entry before it still applies. A receiver MAY report the truncation
   (§7) but MUST NOT discard the entries it did receive.

Rule 2 is the same principle at the head of the line: without a leading caret the first
entry's left boundary is unproven, so the line is foreign and none of it is salvaged — even
if the remainder looks like well-formed Upline.

> Accepting all three line endings is deliberate and costs *less*, not more. Measured on
> ATmega328P a universal reader is **98 B against 122 B** for an LF-only reader that strips
> a trailing CR, because there is no trailing-byte special case. The one mode that cannot
> work is "no line ending."

**Entries are independent.** Sending three entries on one line is an efficiency
optimization and is otherwise identical to sending three lines. A malformed entry is
dropped and structurally intact entries in the same line still apply — there is no
cross-entry invariant to protect, so discarding good entries would buy nothing.

**Delivery is best-effort.** Upline has no retransmission, no acknowledgement of receipt,
and no sequence numbers. On a noisy or faulty link a line may be corrupted or lost
outright; the receiver discards what it cannot parse and the next terminator restores sync.
The heartbeat (§9) bounds how long a receiver waits before calling a device stale. Loss is
survivable this way; **reordering and interleaving are not**, so a transport that may
reorder must be wrapped in something that does not.

Two consequences follow from independence and are accepted deliberately:

- **There is no atomicity.** `^servo~0^led~0^` does not guarantee both or neither.
- **Truncation degrades rather than fails.** A line cut anywhere delivers whatever entries
  arrived whole and silently loses the rest — `^a~1^b~2^c~3` yields `a` and `b`. Nothing
  distinguishes that from a device that only ever sent two entries, so a receiver learns
  *that* bytes were lost only when the cut lands mid-entry, and never learns *what* was
  lost. The heartbeat is the backstop, not the framing.

---

## 4. Escaping

Six sequences. **Escapes are symbolic:** `\n` is the two bytes
`\` `n`, not a raw 0x0A — line splitting consumes a raw 0x0A before escape processing runs.

```text
sequence   bytes        decodes to    name
--------   ----------   ----------    ---------------
\\         0x5C 0x5C    0x5C  \       backslash
\^         0x5C 0x5E    0x5E  ^       caret
\|         0x5C 0x7C    0x7C  |       vertical bar
\~         0x5C 0x7E    0x7E  ~       tilde
\n         0x5C 0x6E    0x0A  LF      line feed
\r         0x5C 0x72    0x0D  CR      carriage return
```

> Keep this table as a code block in any copy of this document. GFM rewrites `\|` to `|`
> even inside code spans, which deletes the backslash and makes the table assert the
> opposite of what it means.

The set is **complete over the reserved set** and deliberately not over all 256 byte
values — there is no `\xHH`. Arbitrary bytes travel as `b64`.

**Invalid escapes.** A `\` followed by anything outside `\ ^ | ~ n r`, or appearing as an
entry's final byte, invalidates **that entry**. Drop it and continue with the rest of the
line.

**Encoder obligations.** Escape `\`, `^`, `~`, `|`, LF, and CR wherever they occur in a key
or a value. Never emit raw bytes 0x00–0x1F or 0x7F.

**Why `^`, `~`, and `|`.** They are the three least-used printable ASCII characters in the
text this protocol carries — key names, sensor labels, units, identifiers. A delimiter's
real cost is not its own byte, it is the escaping it forces on everything else. That is what
keeps `:` `,` `/` `"` `'` `{}` `[]` `#` `$` `%` `&` `*` `+` `@` legal raw.

---

## 5. Entries

```abnf
entry    = key *( "~" value )
value    = subvalue *( "|" subvalue )
key      = 1*char                  ; non-empty after unescaping
```

- **`~` separates the key from its values, and values from each other.** Positional.
- **`|` separates sub-values within one value** — an array.

```text
^temp^                    key alone
^temp~23.45^              one value
^beep~440.00~0.50^        two values
^tags~a|b|c^              one value, three sub-values
^plot~1|2|3~red^          an array value and a scalar value
```

Keys are case-sensitive; for interoperability draw them from `[a-z0-9_]`. **Keys beginning
with `_` are reserved** by this specification (§7); applications MUST NOT use them. A
receiver MUST ignore any `_` key it does not define rather than treat it as an error.

---

## 6. Operations

What an entry *does* is determined by its shape and the key's declared **mode** (§8).
There are no positional rules.

| Host → device | Meaning |
|---|---|
| `^key^` | **read** — report the current value. If mode is `x`, **execute** with no arguments. |
| `^key~^` | **write the empty string** — a value is present and it is empty |
| `^key~v^` | **write** |
| `^key~a~b^` | **execute** with arguments, if mode is `x`; otherwise malformed |

| Device → host | Meaning |
|---|---|
| `^key~v^` | report |
| `^?~…^` | schema response (§8) |
| `^_e~text^` | error (§9) |

`key` and `key~` are distinct and both are representable — that is what lets a read and a
write-of-empty coexist without a convention layered on top.

A read of a `w` or `d` key has no answer; the device SHOULD ignore it. A write to an `r` or
`d` key is ignored, and the device MAY report `_e`.

### 6.1 Acknowledging a write

**The echo is the acknowledgement.** There is no separate ack mechanism, no sequence
number, and no correlation (§3) — a device confirms a write by reporting the key back.

A device that receives a write SHOULD **immediately** emit an entry carrying the affected
key and **the value it actually holds afterwards** — not on the next telemetry tick, and
with just the keys the write touched. A device MAY coalesce echoes under a rapid series of
writes.

Crucially, this applies whether the write was accepted, clamped, or refused:

```text
^servo~90^     ->  ^servo~90^      accepted
^servo~200^    ->  ^servo~80^      refused; still at 80
^fan~1^        ->  ^fan~1^         accepted
```

A refused or clamped write is echoed with **the setting still in force**, which is what
lets a host distinguish a rejected write from a lost line (§3). Clamping to the declared
`min`/`max` is normal and expected; the echo is what tells a host the value it sent is not
the value in effect. On a 115200 link an echo costs about a millisecond of airtime.

A host SHOULD treat a value it has sent as **pending, not current**, until that key arrives
from the device, and MUST NOT write back a value it received — that is a feedback loop, not
a change.

There is no idempotency guarantee: a retried `^beep^` beeps twice and a retried `^step~1^`
may step twice. A host SHOULD NOT retransmit what has not been echoed.

A `w` key and a zero-argument `x` have nothing to echo and are fire-and-forget. Devices
SHOULD give an `x` key an observable effect on some declared key where practical, so that
even a command has something to confirm it.

---

## 7. Reserved keys

| Key | Meaning |
|---|---|
| `?` | schema request / response |
| `_i` | device uuid |
| `_n` | device name |
| `_d` | device description |
| `_v` | Upline version spoken |
| `_r` | receive buffer size, in bytes (§10) |
| `_e` | error text, free-form |

`_i` is the stable handle a host keys everything to — cached rate, saved layouts, history.
It MUST survive reboots and reflashes and MUST NOT be shared across units. Prefer a UUID v4
as unpadded base64url (22 chars). Receivers MUST treat it as opaque.

`_e` values are free-form. Receivers MUST NOT branch on them; treat them as text for
logging and display. A line not beginning with `^` is not an error and MUST NOT produce `_e`.

**The reserved keys travel device→host.** `_i`, `_n`, `_d`, `_v`, and `_r` are answers, sent
when there is reason to — in the `?` response (§8), or on their own. A device is **not**
obliged to answer a read of one: `^_i^` is well-formed and a receiver parses it rather than
dropping it, but silence is a conforming reply. A host that wants them asks `^?^`.

That is what lets a device hold its entire `?` response as one constant in flash and answer
with a single write, instead of implementing a read handler per metadata key. The
identity, the limits, and the schema are all one string the device never has to take apart.

---

## 8. The schema

`^?^` requests. The device answers with a single `?` entry whose values are, in order, the
device metadata and then one declaration per non-reserved key.

```text
^?~_i|n4o8IUt-TQWzph4vfI2Qqw~_n|Greenhouse~_d|North bed sensors~_v|1~_r|128
  ~temp|r|fix2|20.00|-40.00|125.00~fan|rw|bool|0~beep|x|freqHz|volume
  ~freqHz|d|fix2|440.00|20.00|20000.00~volume|rw|fix2|0.50|0.00|1.00^
```

(One line on the wire; wrapped here for reading.)

A library may fill `_v` and `_r` in itself rather than asking the device author to repeat
them — they are both known at build time, and deriving them is what stops an advertised
buffer size from drifting away from the real one.

Each value is an array. **Sub-value 1 is the key name**, and a leading `_` marks it as
metadata rather than a declaration — the same reservation that protects the key namespace
doubles as the discriminator, so no ordinary key name is stolen.

### 8.1 Declarations

```abnf
declaration = key "|" mode [ "|" type [ "|" default [ "|" min [ "|" max ] ] ] ]
x-form      = key "|x" *( "|" argkey )
```

**Mode** is sub-value 2 and tells you how to read everything after it:

| Mode | Meaning |
|---|---|
| *(absent)* | same as `r` |
| `r` | read-only |
| `rw` | read and write |
| `w` | write-only — settable but never reported, so never echoed |
| `x` | executable — sub-values 3+ are argument key names |
| `d` | declaration only — no value; cannot be read or written; exists to name, type, and bound a parameter another key references |

`min` and `max` are independent — either may appear without the other. Both are the
validation contract: a host SHOULD refuse to send an out-of-range value rather than rely on
the device to reject it.

A `d` key MUST NOT be rendered as a standalone control. Referencing is not restricted to
`d` keys — a command may name any declared key as an argument, which is how a persistent
setting can double as an overridable parameter.

### 8.2 Arguments

A command's arguments are positional, in declaration order, and their types, ranges, and
defaults come from the referenced keys:

```text
beep|x|freqHz|volume        declared
^beep^                      no arguments
^beep~880.00^               first argument only
^beep~880.00~1.00^          both
```

A host SHOULD send the complete argument list, filling omissions from declared defaults, so
partial invocation stays rare. What a device does with a partial invocation is the
function's own business — but a device that declines MUST make that observable, via `_e` or
an effect on a declared key. Silent non-execution is indistinguishable from a lost line.

### 8.3 Types

The set is closed; extend with the `x-` prefix. A receiver meeting an unknown type MUST
treat it as `str`.

| Type | Wire form |
|---|---|
| `str` | UTF-8 text |
| `int` | Optionally-signed decimal integer |
| `fix1`…`fix9` | Optionally-signed decimal, **N implied fractional digits** |
| `bool` | `0` or `1` |
| `b64` | base64url (RFC 4648 §6), **no padding** |
| `strs` / `ints` / `fix1s`…`fix9s` | Sub-values, each of the element type |
| `b64json` | **OPTIONAL.** base64url of UTF-8 JSON text |
| `x-*` | Vendor extension |

**`fixN` is the only decimal type.** The wire form is an ordinary decimal string, never a
bare scaled integer: the stream stays readable, a terminal remains a usable debugger, and a
scale mismatch degrades gracefully instead of by 10×. Encoders SHOULD emit exactly N
fractional digits; receivers MUST accept fewer and none at all.

**`b64json` is optional and discouraged on small parts** — it reintroduces the multi-kilobyte
parser this protocol exists to avoid. It is in the set anyway because that constraint is not
universal: on a part with hundreds of kilobytes a JSON parser is a rounding error, and
`b64json` is then the natural fallback for a payload no scalar type can express. A device
MUST NOT be considered non-compliant for rejecting it.

### 8.4 Parsing `int` and `fixN`

**A numeric parser MUST reject what it cannot fully consume.** This is the one place where
best-effort delivery (§3) reaches past framing: a flipped byte inside a value leaves the
line structurally perfect, so `^servo~1a0^` arrives looking valid. A parser that quietly
returns `0` for `""` and for `"abc"` — the common shape, and what v1 shipped — turns that
into a setpoint. Dropping the entry is the correct outcome; silently commanding zero is not.

Integer arithmetic only, no libc, no floating point. `int` is scale 0, so one function
serves both types.

```c
/* Append one digit, refusing anything that would exceed INT32_MAX. */
static int up_acc(uint32_t *v, uint8_t d) {
    if (*v > 214748364UL) return -1;                   /* *10 would overflow */
    *v = *v * 10UL + d;
    return (*v > 2147483647UL) ? -1 : 0;               /* the digit pushed it over */
}

/* Text -> scaled integer. Returns 0 on success and writes *out; -1 if the input is
   not a complete, well-formed number, or does not fit int32_t. Accepts a leading
   sign, a missing integer part (".5"), and a short or absent fraction ("23" at
   scale 2 -> 2300). Excess fractional digits round half up. Rejects an empty string,
   a bare sign or dot, any character outside [+-.0-9], and ANY trailing byte —
   "2 3" and "1.5e3" both fail. */
int up_num_parse(const char *s, uint8_t scale, int32_t *out) {
    uint32_t v = 0;
    uint8_t neg = 0, digits = 0;

    if (*s == '-') { neg = 1; s++; } else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') {
        if (up_acc(&v, (uint8_t)(*s++ - '0'))) return -1;
        digits++;
    }
    if (*s == '.') {
        s++;
        while (scale && *s >= '0' && *s <= '9') {
            if (up_acc(&v, (uint8_t)(*s++ - '0'))) return -1;
            digits++; scale--;
        }
        if (*s >= '5' && *s <= '9') {                  /* round half up */
            if (v == 2147483647UL) return -1;
            v++;
        }
        while (*s >= '0' && *s <= '9') s++;            /* discard excess digits */
    }
    if (!digits || *s) return -1;                      /* no digits, or trailing junk */
    while (scale--) {                                  /* pad a short fraction */
        if (up_acc(&v, 0)) return -1;
    }
    *out = neg ? -(int32_t)v : (int32_t)v;
    return 0;
}
```

`up_acc` is what makes the parser *checked* rather than merely strict. Without it a long
input wraps `int32_t` silently, which is the same class of failure the function exists to
prevent — and a digit-count cap is not a substitute, because the honest limit is a value,
not a length: `21474836.47` is nine significant digits *and* the largest `fix2` there is,
while `3` at scale 9 is one digit and does not fit. The two comparisons per digit cost no
division, so this stays affordable on AVR.

`INT32_MIN` is not reachable on input — the accumulator is unsigned and capped at
`INT32_MAX`, so `-21474836.48` is refused while `-21474836.47` is accepted. The asymmetry
is one count at the extreme of the range and costs a branch to remove.


### 8.5 Device-side arithmetic — outside the protocol

**Nothing here is part of the wire format.** The protocol only ever sees decimal text, and the conversion at the boundary is textual: formatting places a `.` among the digits and `up_num_parse` (§8.4) accumulates them, with no multiply or divide in either normal path. But the scaled integer a device holds *between* those boundaries is ordinary fixed-point arithmetic, and two operations are easy to get wrong on a first implementation.

The scaled `int32_t` **is** the value — `2345` is 23.45 at `fix2` — and the scale is compile-time knowledge, since the device declares it. **Addition, subtraction, comparison, clamping, and scaling by a plain integer all work directly** on the scaled integers with no adjustment.

**Multiplication doubles the scale; division cancels it.** Both need a correction:

```c
/* fix2 × fix2 → fix2.  23.45 × 2.00: (2345 * 200) / 100 = 4690 */
int32_t mul = (int32_t)(((int64_t)a * b) / 100);

/* fix2 ÷ fix2 → fix2.  46.90 ÷ 2.00: (4690 * 100) / 200 = 2345 */
int32_t div = (int32_t)(((int64_t)a * 100) / b);
```

**Intermediates overflow far earlier than results do, and the threshold is lower than it looks.** `int32_t` at `fix2` holds values up to ±21,474,836.47, but the *raw product* `a * b` inside a multiply overflows once the scaled operands reach 46,341 — **just 463.40 at `fix2`, or 46.340 at `fix3`.** Two ordinary sensor readings can cross it. That is why the casts above are `int64_t`; without them the failure is a silent wrap, not a diagnostic — the same class of failure §8.4 refuses at the parsing boundary.

On AVR 64-bit arithmetic is expensive, so where a hot path cannot afford it, restructure instead — divide before multiplying, hold accumulators at a lower scale, or keep a running total in a wider unit and convert once at emit time.

These are the reasons `fixN` caps at scale 9: past that, an `int32_t` holds less than a single integer unit.

---

## 9. Heartbeat

A device MUST emit at least one valid line every **2500 ms**. `^^` — no entries — is the
canonical no-op: 3 bytes including LF, 1.2 B/s. **Any valid line resets the timer,** so a
device streaming telemetry needs no separate ping.

A receiver:

- MUST wait for at least one valid line before transmitting anything.
- SHOULD treat the device as stale after **3× the interval (7500 ms)**.
- SHOULD send `^?^` as its first transmission once liveness is established.

Receive-before-transmit avoids poking an unknown device that speaks some other protocol,
and absorbs reset-on-connect: opening a port often asserts DTR and resets the
microcontroller, so a second or two of bootloader silence follows.

**Transmit-only devices** declare `_r|0` and broadcast their `?` response unprompted at
least every 5000 ms. A host MUST accept an unsolicited `?` response exactly as it accepts a
solicited one. Because `_r|0` says so outright, a host learns the device is one-way from the
first schema it sees rather than by waiting out a timeout.

---

## 10. Limits

The constraint is asymmetric.

**Device → host.** No protocol limit. A device streams a line out byte by byte and needs no
buffer of its own. Hosts MUST accept lines of at least 4096 bytes and SHOULD accept far
more.

**Host → device.** `_r` declares, in bytes, the longest **line** the device can receive —
framing carets included, terminator excluded. It bounds the line, not the entry: a host
batching entries onto one line MUST split so that each line fits, or the efficiency
optimization becomes the overflow.

- `_r|0` means transmit-only; the device has no receive path.
- Devices with a receive path MUST accept at least **32** bytes, so `^?^` and short reads
  always fit.
- If `_r` is absent from a schema response, a host SHOULD assume 32.

Note that `^?^` is 4 bytes with its terminator, so the only thing a host sends before it
knows `_r` is always safe.

A receiver that drops an over-long line MUST also discard the remainder up to the next
terminator, not merely reset its buffer — the tail can itself begin with `^`.

Implementations in both directions MUST parse incrementally, or drain the transport while
assembling. There is no protocol-level checksum: where the transport provides integrity
(USB, BLE, TCP) one would be redundant, and §3's framing already catches truncation.



### 10.1 The drain deadline

On a receive buffer of size *B* bytes at *R* bytes per second, a reader has *B / R* seconds of slack before bytes are lost. The Arduino AVR core's 64-byte RX ring at 115200 baud 8N1 (11,520 B/s) gives 5.56 ms:

> **The read loop MUST NOT go longer than the buffer's drain time without consuming available bytes.**

The ring is *not* a line-length cap — a sketch can assemble a much longer line in its own buffer, and `_r` is what declares that limit. What the ring bounds is how far consumption may lag the wire.

On AVR, overflow is silent. The core's RX ISR is `if (i != _rx_buffer_tail) { … }` with **no `else`** — a full buffer discards the byte and exposes neither that condition nor the hardware `DOR0` overrun bit. On parts where `RAMEND - RAMSTART < 1023` the rings drop to **16 bytes**, cutting the deadline to **1.4 ms**. A blocking delay, a slow sensor read, or a long blocking write in the same loop is what breaks this — not line length.

A dropped byte is not a corrupted value. It removes bytes from the middle of a line, so the line either fails to end in a caret and loses its trailing entry (§3), or loses a whole entry cleanly. Either way the next terminator restores sync — but the loss is silent, which is why the deadline is a MUST rather than a suggestion.

### 10.2 Baud rates

Baud is **irrelevant on USB CDC-ACM, Bluetooth SPP/RFCOMM, BLE, and TCP** — the rate a host requests is a no-op the device may ignore. It matters only for a real UART: direct TTL/RS-232, or a USB-to-serial bridge (FTDI, CH340, CP210x) that clocks a physical line.

Divisor error for common AVR clocks, best of normal and double-speed mode. UART 8N1 tolerates roughly ±2% per end, and oscillator error adds on top:

```text
   baud |   1 MHz int |   8 MHz int |  16 MHz xtal |  20 MHz xtal |  12 MHz xtal
   9600 |      +0.16% |      +0.16% |       +0.16% |       +0.16% |       +0.16%
 115200 |      +8.51% |      -3.55% |       +2.12% |       -1.36% |       +0.16%
 250000 |   too fast  |      +0.00% |       +0.00% |       +0.00% |       +0.00%
```

**250000 is exact on every clock that is a multiple of 4 MHz**, because 250000 × 16 = 4,000,000 divides evenly. 115200 × 16 = 1,843,200 divides evenly into almost nothing, which is why it is the least accurate common rate — out of budget at 8 MHz before the oscillator is even considered.

| Rate | Requirement | When |
|---|---|---|
| **9600** | **MUST** be supported | Universal fallback. Always available, on every device. |
| **115200** | **SHOULD** be the default | Interoperable everywhere; `B115200` exists on every host platform. |
| **250000** | **MAY** be used | Exact divisor, 2.17× faster — but a non-standard host rate (§10.3). |

- A device whose clock is an **internal RC oscillator below ~12 MHz SHOULD default to 9600**, not 115200. At 8 MHz internal, 115200 is −3.55% from the divisor alone, and ±1–2% of uncalibrated oscillator drift lands on top of that.
- A device with **no hardware UART** (software serial, e.g. ATtiny85) SHOULD use 9600. At 8 MHz that is 833 cycles per bit and bit-bangable; 250000 is 32 cycles per bit and is not.
- **250000 is not a standard termios rate.** Linux has no `B250000` and macOS needs `IOSSIOSPEED`. Modern libraries handle custom rates, but `screen`, `minicom`, and some tooling will not — which is why 115200 rather than 250000 is the recommended default despite the worse arithmetic.

### 10.3 Discovering the rate

A host opening a raw UART cold does not know the device's rate, and a standard serial API cannot measure one — it can only set a rate and read bytes. Probing is therefore the only portable method, and **Upline needs no dedicated autobaud mechanism, because the `?` exchange already validates a rate in both directions.**

For each candidate rate, in order:

1. **Open the port and send nothing.** §9's receive-before-transmit rule makes probing inherently safe: a wrong-rate probe cannot poke a device that speaks some other protocol.
2. **Listen for a line that parses** (§3). Timeout **6 s** — worst case is a DTR-triggered reset (~2 s of bootloader) plus one heartbeat interval (2.5 s). If the host **suppresses DTR on open**, no reset occurs and **3 s** suffices.
3. On seeing one, **send `^?^`.**
4. **A well-formed `?` response carrying `_i` confirms the rate** (§8). Anything else — garbage, silence, a malformed response — means try the next rate.

An **unsolicited** `?` response confirms the rate just as well, and is the only confirmation a transmit-only device can offer. Such a device broadcasts its schema at least every 5000 ms (§9), so the 6 s window catches one without the host having to ask — and `_r|0` in that response tells the host not to send at all.

Step 4 is what makes this robust. Noise at a wrong rate will occasionally frame as something line-shaped, but it will essentially never produce a valid schema response *to a `?` the host just sent*. The round trip proves both directions at once.

**Recommended probe order: 115200, then 9600, then 250000** — most likely first. On USB CDC and Bluetooth the first attempt always succeeds, since the rate is ignored, so most modern devices never pay the cost at all.

**Cache the result against `_i`.** The stable identifier makes probing a once-per-device event rather than once-per-connection; a host that has seen a device before opens directly at the remembered rate and falls back to probing only if the exchange fails.

---

## 11. Building on Upline

The point of the schema is that **a host needs no device-specific code.** One `^?^` yields identity, the receive limit, and every key with its mode, type, default, and range — enough to render a complete interface, validate input before sending it, and label a data series correctly.

### 11.1 Schema to interface

**Mode picks the kind of control; type picks the widget.** Read mode first (§8.1):

| Mode | Renders as |
|---|---|
| `r` | A readout, or a plot over time. No input. |
| `rw` | The interactive control below, with a pending state until the echo arrives (§6.1) |
| `w` | The same control, but showing no current value and no pending state — a `w` key is never reported |
| `x` | A button. Its arguments, if any, become a small form built from the referenced keys |
| `d` | Nothing on its own. It exists to type and bound another key's argument |

Then type picks the widget:

| Type | Natural control | Notes |
|---|---|---|
| `bool` | Toggle / checkbox | `default` sets the initial position |
| `fixN` with `min`/`max` | Slider | Step is 10⁻ᴺ; the declared scale is the display precision |
| `fixN` without range | Numeric field | Validate as a decimal, N places |
| `int` with `min`/`max` | Stepper or slider | |
| `int` without range | Numeric field | |
| `str` | Text field | |
| `strs` / `ints` / `fixNs` | List or multi-series plot | Sub-values are the elements (§5) |
| `b64` | Binary blob — download, hex view, or decode by convention | |
| `b64json` | Structured payload — decode and render as a tree | |
| `x-*` | Falls back to a text field | Unknown types are `str` (§8.3) |

`min` and `max` are also the validation contract: a host SHOULD refuse to send an out-of-range value rather than rely on the device to reject it, since §7 makes error reporting optional.

**A host reads with `^key^` and writes with `^key~value^`** (§6), so a control that has never received a value can ask for one rather than waiting for the device to volunteer it.

### 11.2 State, identity, and liveness

- **State** is the most recent value seen per key. Every line is a partial update; keys not present are unchanged. Entries are independent, so within one line the last occurrence of a repeated key wins (§3).
- **Identity** comes from `_i`, `_n`, and `_d`. `_i` is the stable key for persisting per-device settings, layouts, and history across reconnects — `_n` and `_d` are for display and may change.
- **Limits** come from `_r`: the longest line the device can receive. A host batching entries onto one line MUST split to fit (§10), and `_r|0` means the device has no receive path at all, so a host renders every key read-only and never transmits.
- **Liveness** comes from the heartbeat (§9): connected, and stale after 3× the interval. It is a protocol-level signal, so it works identically over a cable, Bluetooth, or a socket.
- **Versioning** comes from `_v`, which a host should check before assuming any grammar beyond this document.

### 11.3 Logging and replay

An Upline stream is plain text with one line per record, so it can be appended to a file as-is and replayed verbatim through the same parser. No framing metadata is lost, and standard text tooling — `grep`, `tail`, `wc`, a spreadsheet — works on a capture without a decoder.

Capture one direction per file. The two directions carry different meanings for the same bytes — `^fan~1^` is a command from the host and a report from the device — and §3 assumes bytes arrive without interleaving from other senders, so a merged bidirectional log is not replayable.

### 11.4 What Upline deliberately does not provide

Know these before building on it: **no addressing or multiplexing** (one device per stream — see §2 on multi-drop), **no reliability or retransmission** (a lost line is lost; the heartbeat bounds detection), **no atomicity** (entries on one line are independent, §3), **no authentication or encryption** (add a layer below if the transport is untrusted), **no timestamps** (a host stamps on arrival; a device that needs its own clock declares a key for it), and **no request/response correlation** (there are no message IDs, so a host issuing overlapping commands must track them itself — which is why the echo carries the resulting value rather than a ticket, §6.1).

---

## 12. Footprint

avr-gcc 7.3.0, ATmega328P, `-Os -ffunction-sections -fdata-sections -Wl,--gc-sections`, measured as link deltas over an identical 184 B harness so every row is comparable:

| Component | Flash |
|---|---|
| **Line parser, including unescaping** | **626 B** |
| Unescaping alone | 76 B |
| `int` / `fixN` parse, validated (§8.4) | 428 B |
| `fixN` formatting | 340 B |
| base64url decode, computed alphabet | 198 B |
| `atoi` | 78 B |
| `strtol` | 846 B |
| `atof` | 1,664 B |
| `dtostrf` | 1,626 B |
| float multiply, no text conversion | 546 B |

Guidance:

- **Use `atoi`, not `strtol`** — 10.8× smaller for base-10, and a hand-rolled integer loop is no smaller than avr-libc's assembly. Better still, use the validated parser: it refuses what `atoi` would silently turn into 0 (§8.4), for 350 B more.
- **Never link `atof` or `dtostrf`.** Parsing and formatting `fixN` costs 768 B against 3,290 B for the float pair — and the float path cannot represent the protocol's decimals exactly anyway, which is why there is no floating-point type.
- **Compute the base64url alphabet, don't table it.** A 64-byte table costs the same flash *and* 66 more bytes of SRAM unless placed in `PROGMEM` — 12.9% of an ATtiny85's 512 B for a table never written to.
- **If a sensor library hands you a `float`,** you still pay ~546 B for float arithmetic but not for the text engine: convert to a scaled integer and format with the `fixN` helper for 886 B against 2,172 B via `dtostrf`. Where the sensor offers a raw integer read, skip float entirely.

Whole-device totals across boards are in §1. On the smallest targets the binding constraint is **SRAM, not flash**. The ATtiny85 also has **no hardware UART** — only a USI — so serial there is a software implementation that burns CPU and borrows a timer; Upline fits, but whether a given sketch services the line inside the drain deadline (§10.1) is a per-application question.

CPU cost is not a consideration at ordinary rates. The scan runs a handful of cycles per byte; at 115200 baud on a 16 MHz part there are roughly 1,389 cycles available per byte, so parsing consumes well under 1% of the budget.

---

## 13. Conformance vectors

The vectors are the deliverable: every implementation runs them.

[`vectors/vectors.tsv`](vectors/vectors.tsv) is the machine-readable form, generated from this
section by [`vectors/extract-vectors.py`](vectors/extract-vectors.py) so the two cannot drift. This markdown stays the
source of truth — it carries the reasoning alongside each case.

`expected` is one of four outcomes, deliberately **not** an error code — §7 leaves `_e`
device-defined, and asserting on codes would make the file unportable. The trailing comment
naming *why* is informative. These vectors confirm that an implementation drops, not that
it dropped for the right reason; implementations distinguishing causes internally should
assert those in their own unit tests.

Vectors live in fenced code blocks, never markdown tables. GFM rewrites `\|` to `|` even
inside code spans, which would make an escaped bar indistinguishable from a raw one.

**Notation.** Bytes are literal except `<LF>`, `<CR>` for single bytes and `\x20` for a
trailing space. `→` separates input from expectation.

| Outcome | Meaning |
|---|---|
| `IGNORE` | Not Upline — the line's first byte is not `^`. Never an error, never `_e`. |
| `DROP` | The receiver could not take the line at all — it exceeded the buffer (§10). Nothing is dispatched. Framing truncation does **not** produce this; see §3 rule 4. |
| `∅` | A valid line that dispatches nothing. |
| entry list | What a receiver dispatches, in order |

An entry is written `key` when it carries no values, and `key=[v]` per value — so `key=[a][b]`
is two values and `key=[a|b]` is one value with two sub-values. A literal bar *inside* a
sub-value is written `\|`, exactly as on the wire, so one sub-value containing a bar
(`[a\|b]`) is distinguishable from two sub-values (`[a|b]`) — which is the whole point of
the escaped-bar rule and cannot be asserted without it. `<empty>` is an empty sub-value.
Entries within a line are comma-separated in dispatch order.

### 13.1 Framing

```text
 1  ^^                              → ∅                   heartbeat; no entries
 2  ^a~1^                           → a=[1]
 3  ^a~1^b~2^                       → a=[1], b=[2]
 4  ^a~1^b~2^c~3^                   → a=[1], b=[2], c=[3]
 5  ^^^                             → ∅                   empty entries are ignored
 6  ^a~1^^b~2^                      → a=[1], b=[2]        empty entry between two real ones
 7  temp:23.5 rpm:1200              → IGNORE              foreign line, NOT an error
 8  hello world                     → IGNORE
 9  <empty line>                    → IGNORE
10  ^                               → ∅                   one caret; no complete entries
11  ^a~1                            → ∅                   nothing terminated it
12  ^a~1^b                          → a=[1]               trailing fragment b dropped
13  ^a~1^junk                       → a=[1]               same rule, longer fragment
14  ^a~1^\x20                       → a=[1]               trailing space is a fragment
15  ^a~1^b~2^c~3                    → a=[1], b=[2]        the canonical truncation
16  a~1^b~2^                        → IGNORE              no leading caret: the first entry's
                                                         left boundary is unproven, so the
                                                         whole line is foreign — NOT b=[2]
```

Vectors 11–15 are §3 rule 4 — an unterminated tail is dropped on its own, without taking
the fully bounded entries before it. A receiver MAY report the truncation but MUST NOT
discard what arrived whole.

Vector 16 is the same principle at the other end. It is the one that catches a lenient
implementation: the line *looks* like recoverable Upline and `b~2` even has carets on both
sides, but the receiver cannot know whether it joined the stream mid-line, so nothing is
salvaged.

### 13.2 Escaping

```text
17  ^v~a\^b^                        → v=[a^b]
18  ^v~a\~b^                        → v=[a~b]
19  ^v~a\|b^                        → v=[a\|b]            escaped bar: ONE sub-value
20  ^v~a\\b^                        → v=[a\b]
21  ^v~line1\nline2^                → v=[line1<LF>line2]  THE NEWLINE ESCAPE
22  ^v~x\r\ny^                      → v=[x<CR><LF>y]
23  ^k\~ey~1^                       → key "k~ey" = [1]
24  ^k\^ey~1^                       → key "k^ey" = [1]
25  ^k\|ey~1^                       → key "k|ey" = [1]
26  ^v~a\qb^                        → ∅                   undefined escape; entry dropped
27  ^v~a\^                          → ∅                   valid \^ eats the closing caret;
                                                         nothing was terminated
28  ^v~a\\^                         → v=[a\]              contrast with 27
29  ^a~1^v~b\qc^d~2^                → a=[1], d=[2]        bad entry dropped, rest apply
```

Contrast 19 with 33, and 27 with 28 — those pairs are why this is not a table.

### 13.3 Entries and values

```text
30  ^key^                           → key                 no values
31  ^key~^                          → key=[]              ONE empty value; distinct from 30
32  ^a~1~2^                         → a=[1][2]            two values
33  ^tags~a|b|c^                    → tags=[a|b|c]        one value, three sub-values
34  ^plot~1|2|3~red^                → plot=[1|2|3][red]   array value, then scalar value
35  ^a~|^                           → a=[<empty>|<empty>]  one value, two empty sub-values
36  ^~1^                            → ∅                   empty key; entry dropped
37  ^a~1^a~2^                       → a=[1], a=[2]        both dispatched; last-wins is the caller's
38  ^utf~温度^                       → utf=[温度]
39  ^t~12:34:56^                    → t=[12:34:56]        colon is not reserved
40  ^raw~-_8AAQ^                    → raw=[-_8AAQ]        base64url never needs escaping
41  ^desc~A "quoted", {list}: ok!^  → desc=[A "quoted", {list}: ok!]
```

30 and 31 are the load-bearing pair: they are a **read** and a **write of the empty string**
(§6), and an implementation that collapses them cannot distinguish the two operations.

### 13.4 Reserved keys

```text
42  ^_e~badescape^                  → _e=[badescape]
43  ^_e~unknown key: humidty^       → _e=[unknown key: humidty]
44  ^_z~2^                          → ∅                   unknown _ key ignored, NOT an error
45  ^a~1^_z~2^b~3^                  → a=[1], b=[3]
46  ^_i^                            → _i                  a defined reserved key, so it is
                                                          parsed rather than dropped (§7)
```

### 13.5 Schema

```text
47  ^?^                             → ?                   schema request
48  ^?~_i|x~_n|Bench~_d|Rig~_v|1~_r|128~t|r|fix2|20.00|-40.00|125.00^
      → _i=x, _n=Bench, _d=Rig, _v=1, _r=128
      → t: mode=r, type=fix2, default=20.00, min=-40.00, max=125.00
49  ^?~…~fan|rw|bool|0^             → fan: mode=rw, type=bool, default=0, no range
50  ^?~…~t||fix2|5.00^              → t: mode absent → r; default 5.00, no range
51  ^?~…~n|rw|int||10^              → n: min=10, max absent
52  ^?~…~beep|x|freqHz|volume^      → beep: mode=x, args = freqHz, volume
53  ^?~…~freqHz|d|fix2|440.00|20.00|20000.00^
      → freqHz: mode=d, declaration only; never rendered standalone
54  ^?~…~q|r|x-custom^              → q: unknown type → treat as str
55  ^?~…~q|zz|int^                  → q: unrecognized mode → treat as r
56  ^?~…~note|r|str|a\|b^           → note: default is the one sub-value a|b, not two
57  ^?~_n|x~_d|y~_v|1~_r|128^       → incomplete: _i missing
58  ^?~_i|x~_n|y~_d|z~_v|1~_r|0^    → transmit-only device; host never sends
```

Vector 56 is the schema's own escaping case: sub-fields split on **unescaped** bars only, so
a default containing a literal bar survives.

### 13.6 `fixN`

```text
59  23.45      at fix2              → 2345
60  23         at fix2              → 2300                missing fraction MUST be accepted
61  23.456     at fix2              → 2346                excess digits round half up
62  -23.456    at fix2              → -2346
63  .5         at fix2              → 50                  missing integer part
64  <empty>    at fix2              → REJECT              not zero — §8.4
65  abc        at fix2              → REJECT
66  2 3        at fix2              → REJECT              input not fully consumed
67  1.5e3      at fix2              → REJECT              no exponent form
```

64–67 are the contract of §8.4's `up_num_parse`. A parser that returns `0` for any of them turns a
corrupted value into a silent setpoint.

### 13.7 Line endings

Stream-level rather than line-level: these exercise the reader, and the expectation is how
many lines it dispatches.

```text
68  ^a~1^<LF>                       → 1 line              LF
69  ^a~1^<CR><LF>                   → 1 line              CRLF is ONE terminator
70  ^a~1^<CR>                       → 1 line              CR alone
71  ^a~1^<LF>^b~2^<LF>              → 2 lines
72  ^a~1^<CR><LF>^b~2^<CR><LF>      → 2 lines             no double-dispatch
73  <LF><LF>^a~1^<LF>               → 1 line              empty lines ignored
74  ^a~1^                           → 0 lines             no terminator: incomplete
```

Vector 74 is not an error — the line is unfinished, and a byte-oriented reader cannot know
whether more is coming. It is also the failure a user hits with a terminal set to send no
line ending.

### 13.8 Limits

```text
75  <line longer than _r>           → DROP                and the remainder up to the next
                                                          terminator is discarded too, not
                                                          reparsed — its tail can begin with ^
```

### 13.9 What sits above the parser

Only two things, both schema-dependent rather than positional:

- **30 and 46** parse identically — a key with no values. Whether that is a *read* or an
  *execute* depends on the key's declared mode (§8), which the parser does not know.
- **37** dispatches both entries; last-wins is the caller's state model, not the decoder's.

Everything else is decidable from the bytes alone. v1 needed four such carve-outs because
command-versus-flag depended on position; removing positional rules removed the carve-outs.

---

## 14. Status

**[`upline-arduino/`](upline-arduino/) implements this grammar** and passes every vector in
§11 that is mechanically checkable on a host — 55 of them, covering framing, escaping,
entries, the reserved namespace, and `up_num_parse`. The remaining 20 are the schema
interpretation, the line reader, and the `_r` limit, which a host-side harness asserts
rather than the device codec.

Writing the vectors before the implementation was the point. §11 caught four errors that
prose review had not: the mid-line resync case (vector 16), a reference parser that
rejected the largest legal `fix2` value, a `\n`-in-a-value rendering bug, and a library
that knew only `_e` when §7 defines six reserved keys.

**Not done:**

1. **The Rust host still speaks v1.** The entry model, the delimiters, and the schema shape
   all changed; the v1 code does not adapt.
2. **Version discovery stays unsolved.** `_v` lives inside a response a host must already
   parse, so a host cannot learn the grammar before assuming it. Accepted for now — `_v` is
   `1` and the tagged `v1.0.0` is treated as never having shipped — but the cost lands on
   whatever changes next.
3. **No CI.** `vectors/vectors.tsv` is generated and a host runner exists; neither is wired to run
   automatically.

---

## Appendix A — why not JSON or MQTT

**JSON.** On an ATmega328P, ArduinoJson v7.4.2 costs **9,680 B — 30% of flash** for work a hand-rolled splitter does in 2,432 B; v6.21.4 costs 6,486 B. The maintainer's own [v7 announcement](https://arduinojson.org/news/2024/01/03/arduinojson-7/) advises staying on v6 when memory is tight, and v7 requires dynamic allocation, which [the v6 docs](https://arduinojson.org/v6/how-to/reduce-memory-usage/) tell ATmega328 users to avoid. RAM scales with document *structure*, not payload — `[1,2,3,4]` is 9 bytes of JSON but 64 bytes of document — and AVR is capped at 255 slots and 255-char strings. On an ATtiny85 it does not compile at all. The tax is memory, not speed: at 115200 baud there are ~1,389 cycles per byte and ArduinoJson uses ~165, so performance cannot justify the 4×.

**MQTT.** For a point-to-point link it is inapplicable, not merely costly: [OASIS MQTT 3.1.1](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.pdf) §4.2 requires an ordered, lossless byte stream to a **Server**, and a cable has no broker and no socket. Where it does apply, Ethernet + PubSubClient measures 16,414 B — 50% of flash — with 256-byte messages; a software TCP stack reaches ~77%; lwIP wants ~40 KB of ROM, more than the whole chip. Real "MQTT over serial" projects have the microcontroller emit plain text while a host daemon owns the broker connection — which is what Upline formalizes.

**Both conclusions are 8-bit-specific.** On a 32-bit part with hundreds of kilobytes, JSON is a rounding error and MQTT is natural. The asymmetry is what settles it: one wire format that works on an ATmega168 also works on an ESP32; the reverse is false. Where a capable part genuinely needs structure, `b64json` (§8.3) is the escape hatch, and it costs the small parts nothing.

## Appendix B — prior art

A survey of 25 ASCII line formats — NMEA 0183, G-code/Marlin, SCPI/IEEE 488.2, Modbus ASCII, AT/V.250, logfmt, InfluxDB line protocol, RFC 5424, Graphite, StatsD, Teleplot, Arduino Serial Plotter, Betaflight CLI, FIX, RFC 7464, NDJSON and others — found none combining newline-delimited ASCII, key/value structure, complete escaping, and binary carriage.

| Format | Where it stops |
|---|---|
| **logfmt** | Only escape is `\"`; a literal backslash is unrepresentable and newline has no escape. No binary. Standardization is an explicit non-goal of its reference implementation. |
| **InfluxDB line protocol** | The most complete escape table surveyed, one character short — *"does not support the newline character in tag or field values."* No bytes type. |
| **RFC 5424 structured data** | Closest registered precedent; escapes `\"` `\\` `\]` and defines invalid-escape behavior. No newline escape, no binary. |
| **FIX tag=value** | Solves key/value, escaping, and binary — via non-printable SOH and length-prefixing, which breaks `readline()` by construction. |
| **NMEA 0183** | Line-delimited and self-framing, but positional: fields are identified by index within a fixed sentence, so nothing is self-describing and adding a field is a new sentence type. |
| **SCPI** | Rich command tree and a `*IDN?` identity query, but no schema — a host still needs a per-instrument driver to know what is queryable and in what range. |

Three findings shaped Upline.

**Only 4 of the 25 have a real escape mechanism, and none can escape a newline.** The rest use prohibition (Teleplot: *"avoid `:|` characters"*), destructive substitution (StatsD's `.replace(';','_')`; Datadog silently collapsing distinct labels), or nothing at all. The costs are documented: [statsd #585](https://github.com/statsd/statsd/issues/585) has been open ten years over a colon in a metric name, and StatsD fragmented into five incompatible tagging dialects because it could not be extended without claiming a new sigil. §4 exists so that no value is ever unrepresentable.

**Nothing surveyed is self-describing at baseline.** The closest precedent is Sparkplug B's mandatory birth-certificate metric declaration, which `?` adapts to a point-to-point link — with the addition that Upline declares *mode* as well as type, so a host knows not only what a key holds but whether it may write it (§8.1).

**Almost nothing validates numeric input.** The common shape returns 0 for unparseable text, which turns a corrupted setpoint into a command rather than an error. §8.4 refuses instead.

## References

- [RFC 4648](https://www.rfc-editor.org/rfc/rfc4648.html) §5, §3.2 · [RFC 8949](https://www.rfc-editor.org/rfc/rfc8949.html#section-6.1) §6.1 · [RFC 7515](https://www.rfc-editor.org/rfc/rfc7515.html) — unpadded base64url precedent
- [RFC 5424](https://www.rfc-editor.org/rfc/rfc5424.txt) §6.3.3 — invalid-escape behavior
- [MODBUS over Serial Line V1.02](https://www.modbus.org/file/secure/modbusoverserial.pdf) §2.5.2 — normative resynchronization
- [Sparkplug B 3.0.0](https://sparkplug.eclipse.org/specification/version/3.0/documents/sparkplug-specification-3.0.0.pdf) — birth-certificate self-description
- [kr/logfmt](https://pkg.go.dev/github.com/kr/logfmt) · [InfluxDB line protocol](https://docs.influxdata.com/influxdb/v2/reference/syntax/line-protocol/) · [Teleplot](https://github.com/nesnes/teleplot) · [Arduino Serial Plotter protocol](https://github.com/arduino/Arduino/blob/master/build/shared/ArduinoSerialPlotterProtocol.md)
- [NMEA 0183](https://www.nmea.org/nmea-0183.html) · [SCPI / IEEE 488.2](https://www.ivifoundation.org/docs/scpi-99.pdf)
- [ATmega328P datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) Table 19-12 · Arduino AVR core `HardwareSerial.h` / `HardwareSerial_private.h` — ring sizes, silent overflow

---

This work made possible by Small Craft, Inc. and is released under CC0. You are free to use,
extend, modify, redistribute, etc. as you see fit. Keep making awesome things!
