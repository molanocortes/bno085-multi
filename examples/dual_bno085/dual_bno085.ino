// dual_bno085 - two BNO085 IMUs streaming at once with bno085-multi.
//
// The stock Adafruit_BNO08x library is single-instance and hangs with two
// sensors. This reads both at full rate because all state is per-object.
//
// Wiring (Teensy 4.1): each BNO085 on its own I2C bus, in I2C mode (PS0=PS1=GND).
//   IMU A -> Wire1  (SDA 17, SCL 16),  ADO -> GND  (address 0x4A)
//   IMU B -> Wire2  (SDA 25, SCL 24),  ADO -> 3V3  (address 0x4B)
// Two on ONE bus also works: same Wire, different addresses (0x4A / 0x4B).

#include <Wire.h>
#include "tiny_bno085.h"

TinyBNO085 imuA(&Wire1, 0x4A);
TinyBNO085 imuB(&Wire2, 0x4B);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire1.begin(); Wire1.setClock(400000);
  Wire2.begin(); Wire2.setClock(400000);

  Serial.print("IMU A (0x4A): "); Serial.println(imuA.begin() ? "ok" : "not found");
  Serial.print("IMU B (0x4B): "); Serial.println(imuB.begin() ? "ok" : "not found");
}

void loop() {
  imuA.poll();          // non-blocking: refreshes the quaternion when a report is ready
  imuB.poll();

  Serial.print("A ");
  Serial.print(imuA.qw, 3); Serial.print(' '); Serial.print(imuA.qx, 3); Serial.print(' ');
  Serial.print(imuA.qy, 3); Serial.print(' '); Serial.print(imuA.qz, 3);
  Serial.print("   B ");
  Serial.print(imuB.qw, 3); Serial.print(' '); Serial.print(imuB.qx, 3); Serial.print(' ');
  Serial.print(imuB.qy, 3); Serial.print(' '); Serial.println(imuB.qz, 3);

  delay(20);            // 50 Hz print; poll() itself is happy much faster
}
