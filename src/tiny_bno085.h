// tiny_bno085.h - minimal, REENTRANT BNO085 rotation-vector driver over I2C.
//
// Why this exists: the Adafruit_BNO08x/CEVA sh2 driver keeps its context in
// file-scope globals (one sh2 instance, one sensor-value pointer, one callback),
// so it is single-instance by construction: two sensors clobber each other and
// the blocking read hangs. This driver keeps ALL state per-object, so any number
// of BNO085s can stream simultaneously (one per I2C bus here).
//
// Scope: rotation vector (quaternion) only - exactly what the rig needs.
// Protocol: SHTP over I2C (BNO08x datasheet 1.3.1, SH-2 ref manual 6.5.18).
//   - every read transaction starts with a 4-byte header: len LSB, len MSB
//     (bit15 = continuation), channel, sequence
//   - enable a sensor: Set Feature Command (0xFD) on channel 2
//   - data arrives as input reports on channel 3: 5-byte timebase (0xFB)
//     then 0x05 rotation vector, int16 Q14 for i,j,k,real
// No warranty beyond this rig; bench-verified against the wearable's sensors.

#pragma once
#include <Wire.h>

struct TinyBNO085 {
  TwoWire *bus;
  uint8_t  addr;
  uint8_t  seqTx = 0;      // our tx sequence on channel 2
  bool     ok    = false;  // begin() succeeded (feature command acked by wire)
  float    qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  bool     fresh = false;  // a quaternion has arrived since begin()

  TinyBNO085(TwoWire *w, uint8_t a) : bus(w), addr(a) {}

  bool present() {
    bus->beginTransmission(addr);
    return bus->endTransmission() == 0;
  }

  // ---- SHTP write: 4-byte header + payload in one transaction ------------
  bool sendPacket(uint8_t channel, const uint8_t *data, uint8_t len) {
    uint16_t total = (uint16_t)len + 4;
    bus->beginTransmission(addr);
    bus->write((uint8_t)(total & 0xFF));
    bus->write((uint8_t)(total >> 8));
    bus->write(channel);
    bus->write(seqTx++);
    bus->write(data, len);
    return bus->endTransmission() == 0;
  }

  // Set Feature Command: rotation vector (0x05) at `interval_us` per report.
  bool enableRotationVector(uint32_t interval_us = 10000) {  // 100 Hz
    uint8_t p[17] = {0};
    p[0] = 0xFD;                       // SET_FEATURE_COMMAND
    p[1] = 0x05;                       // feature: rotation vector
    p[5] = (uint8_t)(interval_us & 0xFF);
    p[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    p[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    p[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    return sendPacket(2, p, sizeof(p)); // channel 2 = SH-2 control
  }

  // ---- SHTP read: one packet if available; parse rotation vectors --------
  // Returns true if it consumed a packet (of any kind).
  bool poll() {
    // read the 4-byte header alone first to learn the packet length
    if (bus->requestFrom((int)addr, 4) != 4) return false;
    uint8_t h0 = bus->read(), h1 = bus->read();
    uint8_t chan = bus->read(); (void)bus->read();      // seq, unused
    uint16_t len = ((uint16_t)h0 | ((uint16_t)h1 << 8)) & 0x7FFF;
    if (len == 0 || len == 0x7FFF) return false;        // nothing pending

    if (len <= 128) {
      // whole packet (header again + payload) fits one Teensy Wire read
      if (bus->requestFrom((int)addr, (int)len) != (int)len) return false;
      uint8_t buf[128];
      for (uint16_t i = 0; i < len; i++) buf[i] = bus->read();
      if (chan == 3) parseInput(buf + 4, len - 4);      // skip the 4-byte header
      return true;
    }
    // oversized packet (boot advertisement ~272 B): discard in chunks.
    // Each chunked transaction re-sends a 4-byte continuation header.
    uint16_t consumed = 0;
    while (consumed < len) {
      int n = bus->requestFrom((int)addr, 32);
      if (n <= 4) break;
      for (int i = 0; i < n; i++) (void)bus->read();
      consumed += (uint16_t)(n - 4);
    }
    return true;
  }

  void parseInput(const uint8_t *p, uint16_t n) {
    uint16_t i = 0;
    while (i < n) {
      uint8_t id = p[i];
      if (id == 0xFB) { i += 5; continue; }             // timebase reference
      if (id == 0x05 && i + 14 <= n) {                  // rotation vector, Q14
        int16_t qi = (int16_t)(p[i+4]  | (p[i+5]  << 8));
        int16_t qj = (int16_t)(p[i+6]  | (p[i+7]  << 8));
        int16_t qk = (int16_t)(p[i+8]  | (p[i+9]  << 8));
        int16_t qr = (int16_t)(p[i+10] | (p[i+11] << 8));
        const float s = 1.0f / 16384.0f;                // 2^-14
        qx = qi * s; qy = qj * s; qz = qk * s; qw = qr * s;
        fresh = true;
        i += 14; continue;
      }
      break;                                            // unknown report: stop
    }
  }

  // Drain boot chatter (bounded), then enable the rotation vector.
  bool begin() {
    ok = false; fresh = false;
    if (!present()) return false;
    uint32_t t0 = millis();
    while (millis() - t0 < 150) { if (!poll()) break; } // discard advertisements
    if (!enableRotationVector()) return false;
    ok = true;
    return true;
  }
};
