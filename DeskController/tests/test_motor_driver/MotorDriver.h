// =====================================================================
//  MotorDriver.h  —  motor output abstraction
//
//  The controller only ever calls  set(-1.0 .. +1.0) :
//    +1 = full power UP, -1 = full power DOWN, 0 = brake (low sides on)
//
//  BtsMotor     : one port of the "i2c BTS7960" board
//                 (PCA9685 -> 74AHC244 -> 2x BTS7960B per motor, I2C via
//                 an ISO1540 isolator).
// =====================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

class MotorDriver {
public:
  virtual ~MotorDriver() {}
  virtual bool begin() = 0;

  void set(float cmd) {
    if (cmd >  1.0f) cmd =  1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    if (_invert) cmd = -cmd;
    _last = cmd;
    writeHardware(cmd);
  }
  // Outputs high-impedance (coast). Used at boot and on driver faults.
  virtual void disable() { set(0.0f); }
  float last() const { return _invert ? -_last : _last; }
  void  setInvert(bool inv) { _invert = inv; }

protected:
  virtual void writeHardware(float cmd) = 0;
  bool  _invert = false;
  float _last   = 0.0f;
};

// =====================================================================
//  PCA9685 16-channel 12-bit PWM controller
//  Keeps a shadow copy of every channel and only writes what changed.
//  Each channel is one short I2C transaction (~0.15 ms at 400 kHz).
// =====================================================================
class Pca9685 {
public:
  static constexpr uint8_t MODE1 = 0x00, MODE2 = 0x01, LED0 = 0x06,
                           ALL_LED = 0xFA, PRESCALE = 0xFE;

  // Returns false if the chip does not ACK.
  bool begin(TwoWire& bus, uint8_t addr, float pwmHz) {
    _bus = &bus; _addr = addr; _hz = pwmHz;
    return reinit();
  }

  // Full (re)initialisation: all outputs off FIRST, then configure.
  // Safe to call any time, e.g. after a fault or a power cycle of the board.
  bool reinit() {
    _err = false;
    for (int i = 0; i < 16; i++) _valid[i] = false;
    if (!probe()) { _err = true; return false; }

    // 1. All channels full-OFF immediately. If the ESP32 rebooted while a
    //    motor was running, the PCA9685 is still driving it until now.
    uint8_t allOff[4] = {0, 0, 0, 0x10};
    writeRegs(ALL_LED, allOff, 4);

    // 2. Totem-pole outputs (they drive 74AHC244 inputs), not inverted,
    //    outputs update on I2C STOP so each 4-byte channel write is atomic.
    write8(MODE2, 0x04);

    // 3. PWM frequency. The prescaler can only be written while asleep.
    //    The PCA9685 tops out at ~1526 Hz (prescale 3).
    int pre = (int)lroundf(25000000.0f / (4096.0f * _hz)) - 1;
    pre = constrain(pre, 3, 255);
    write8(MODE1, 0x30);            // SLEEP + auto-increment
    write8(PRESCALE, (uint8_t)pre);
    write8(MODE1, 0x20);            // wake, auto-increment, ALLCALL off
    delayMicroseconds(600);         // oscillator start-up (500 us max)
    _actualHz = 25000000.0f / (4096.0f * (pre + 1));
    return !_err;
  }

  // Reads MODE1. If the chip lost power and reset, it comes back asleep.
  bool looksReset() {
    uint8_t m;
    if (!read8(MODE1, m)) return true;
    return (m & 0x10) != 0;
  }

  // duty 0..4095 (0 = full off, >= 4095 = full on). 'phase' staggers the
  // ON edge so the four motors don't all switch at once (less supply ripple).
  void setDuty(uint8_t ch, uint16_t duty, uint16_t phase = 0) {
    uint8_t r[4];
    if (duty == 0)         { r[0] = 0; r[1] = 0;    r[2] = 0; r[3] = 0x10; }   // full off
    else if (duty >= 4095) { r[0] = 0; r[1] = 0x10; r[2] = 0; r[3] = 0;    }   // full on
    else {
      uint16_t on = phase & 0x0FFF, off = (on + duty) & 0x0FFF;               // wraps OK
      r[0] = on & 0xFF; r[1] = on >> 8; r[2] = off & 0xFF; r[3] = off >> 8;
    }
    if (_valid[ch] && memcmp(r, _shadow[ch], 4) == 0) return;               // unchanged
    if (writeRegs(LED0 + 4 * ch, r, 4)) { memcpy(_shadow[ch], r, 4); _valid[ch] = true; }
    else _valid[ch] = false;
  }

  bool  ok() const      { return !_err; }      // false after any failed transfer
  uint32_t errors() const { return _errCount; }
  float actualHz() const { return _actualHz; }
  uint8_t address() const { return _addr; }

private:
  TwoWire* _bus = nullptr;
  uint8_t  _addr = 0x40;
  float    _hz = 1526, _actualHz = 0;
  bool     _err = false;
  uint32_t _errCount = 0;
  uint8_t  _shadow[16][4];
  bool     _valid[16] = {false};

  bool fail() { _err = true; _errCount++; return false; }
  bool probe() { _bus->beginTransmission(_addr); return _bus->endTransmission() == 0; }
  bool write8(uint8_t reg, uint8_t v) { return writeRegs(reg, &v, 1); }
  bool writeRegs(uint8_t reg, const uint8_t* d, uint8_t n) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(d, n);
    return _bus->endTransmission() == 0 ? true : fail();
  }
  bool read8(uint8_t reg, uint8_t& v) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return fail();
    if (_bus->requestFrom(_addr, (uint8_t)1) != 1) return fail();
    v = _bus->read();
    return true;
  }
};

// =====================================================================
//  One motor port on the i2c BTS7960 board.
//
//  Each port is two BTS7960B half-bridges (a full H-bridge). Channel map
//  traced from the KiCad schematic. It matches BTS7960_MotorShield.cpp:
//
//   port | IN half A | IN half B | INH (both) | spare->hdr | motor out
//   -----+-----------+-----------+------------+------------+-----------
//    1   |  10 (U8)  |  13 (U9)  |    11      |  12 (J10)  | J5 1-2 / J2 1-2
//    2   |  15 (U10) |   8 (U11) |    14      |   9 (J10)  | J5 3-4 / J2 3-4
//    3   |   6 (U2)  |   1 (U3)  |     7      |   0 (J9)   | J3 1-2 / J4 1-2
//    4   |   3 (U4)  |   4 (U6)  |     2      |   5 (J9)   | J3 3-4 / J4 3-4
//
//  Drive scheme (sign-magnitude, synchronous):
//    UP   : INH on, IN_B low, IN_A = PWM
//    DOWN : INH on, IN_A low, IN_B = PWM
//    0    : INH on, both IN low  -> both low sides on = dynamic brake
//    disable(): INH off -> outputs high-impedance (coast)
// =====================================================================
struct BtsPort { uint8_t inA, inB, inh, spare; };
static const BtsPort BTS_PORTS[4] = {
  { 10, 13, 11, 12 },
  { 15,  8, 14,  9 },
  {  6,  1,  7,  0 },
  {  3,  4,  2,  5 },
};

class BtsMotor : public MotorDriver {
public:
  BtsMotor(Pca9685& pca, uint8_t port /* 1..4 */)
    : _pca(pca), _p(BTS_PORTS[(port - 1) & 3]), _phase(((port - 1) & 3) * 1024) {}

  bool begin() override { disable(); return _pca.ok(); }

  void disable() override {
    _pca.setDuty(_p.inh, 0);
    _pca.setDuty(_p.inA, 0);
    _pca.setDuty(_p.inB, 0);
    _pca.setDuty(_p.spare, 0);
    _last = 0;
  }

protected:
  void writeHardware(float cmd) override {
    uint16_t duty = (uint16_t)lroundf(fabsf(cmd) * 4095.0f);
    if (duty == 0) {                               // brake
      _pca.setDuty(_p.inA, 0);
      _pca.setDuty(_p.inB, 0);
    } else if (cmd > 0) {                          // drop the other side first
      _pca.setDuty(_p.inB, 0);
      _pca.setDuty(_p.inA, duty, _phase);
    } else {
      _pca.setDuty(_p.inA, 0);
      _pca.setDuty(_p.inB, duty, _phase);
    }
    _pca.setDuty(_p.inh, 4095);                    // enabled (no-op if already on)
  }

private:
  Pca9685&      _pca;
  const BtsPort _p;
  uint16_t      _phase;
};
