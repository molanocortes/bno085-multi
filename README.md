<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/header-dark.svg">
  <img alt="bno085-multi: several BNO085 IMUs, one microcontroller. C, single header, MIT. Two sensors at addresses 0x4A and 0x4B, each with its own TinyBNO085 instance, feeding one Teensy 4.1." src="docs/header-light.svg" width="100%">
</picture>

# bno085-multi

**Run two or more BNO085 IMUs on one microcontroller, simultaneously, at full rate. The stock Adafruit library cannot.**

`tiny_bno085.h` is a single-header, dependency-free, fully **reentrant** driver for the
CEVA/Hillcrest **BNO085** (also BNO086 / BNO080) that reads the **rotation-vector
quaternion** over I2C. Because it keeps *all* protocol state **inside each object** (no
globals), you can instantiate as many sensors as you have I2C buses or addresses and
stream them all at once.

Bench-run on a **Teensy 4.1**: two BNO085s at the library's default **100 Hz** report
rate, polled in the same loop as twelve AS5600 encoder channels and logged at 50 Hz.

## Why this exists

The popular `Adafruit_BNO08x` library wraps CEVA's `sh2` driver, which stores its state in
**file-scope globals**: one `sh2` context, one sensor-value pointer, one callback. It is
therefore **single-instance by construction**. The moment you poll a second sensor, the
shared state is clobbered and the blocking read **hangs the whole sketch**. This is a
commonly reported problem with no clean fix in the stock library.

`bno085-multi` sidesteps it entirely: it speaks the BNO085's **SHTP protocol** directly in
~150 lines, with every byte of state living inside the `TinyBNO085` instance. Nothing is
shared, so nothing collides.

## Features

- Multiple BNO085s at once, each at full report rate (tested: two at 100 Hz reports).
- Single header; only dependency is Arduino `Wire.h`.
- **Non-blocking, bounded `poll()`** you can call every loop pass next to other sensors.
- Rotation vector (orientation quaternion), the field most projects actually need.
- Tested on Teensy 4.1; portable to any Arduino-compatible I2C.

## Wiring

Each sensor needs `VCC (3V3)`, `GND`, `SDA`, `SCL`, and I2C-mode straps (`PS0 = PS1 = GND`).
Two ways to run several:

- **Different addresses on one bus:** strap `ADO` low (`0x4A`) or high (`0x4B`).
- **One bus per sensor (recommended):** the BNO085 clock-stretches aggressively, so giving
  each its own I2C peripheral removes bus-level contention as well. This is how the
  reference rig runs, on the Teensy's `Wire1` and `Wire2`.

> This library fixes the *software* single-instance limit. The clock-stretch contention is
> a separate, hardware-level issue; separate buses solve both at once.

## Usage

```cpp
#include <Wire.h>
#include "tiny_bno085.h"

TinyBNO085 imuA(&Wire1, 0x4A);   // e.g. hand
TinyBNO085 imuB(&Wire2, 0x4B);   // e.g. forearm

void setup() {
  Wire1.begin(); Wire1.setClock(400000);
  Wire2.begin(); Wire2.setClock(400000);
  imuA.begin();
  imuB.begin();
}

void loop() {
  imuA.poll();   // non-blocking; refreshes imuA.qw/qx/qy/qz when a report arrives
  imuB.poll();
  // use imuA.qw..qz and imuB.qw..qz  (unit quaternion, order w, x, y, z)
}
```

See [`examples/dual_bno085`](examples/dual_bno085) for a complete sketch.

## API

| member | description |
|---|---|
| `TinyBNO085(TwoWire* bus, uint8_t addr)` | construct on a bus and address |
| `bool present()` | does the chip ACK on the bus? |
| `bool begin(...)` | drain boot chatter, then enable the rotation vector |
| `bool poll()` | read one pending packet, non-blocking; `true` if one was consumed |
| `float qw, qx, qy, qz` | latest unit quaternion (w, x, y, z) |
| `bool fresh` | a quaternion has arrived since `begin()` |

## Scope and limits

Rotation vector only: no raw accel/gyro/mag, no calibration or activity reports. This is
deliberate; the file is small on purpose. If you need the full sensor suite on a *single*
BNO085, use the Adafruit library. Use this one when you need **several** BNO085s at once.

## Origin

Written for **TAKTO**, an open-source hand exoskeleton with four instrumented fingers (M.Sc. thesis, Sebastian Molano,
Hochschule Anhalt), where a hand IMU and a forearm IMU must stream together with twelve
joint encoders. If it helps you, a star is appreciated.

## License

MIT, see [LICENSE](LICENSE).
