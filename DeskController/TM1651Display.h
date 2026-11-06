// =====================================================================
//  TM1651Display.h  —  driver for the Gotek LEDC68 3-digit LED board
//
//  The TM1651 uses a 2-wire CLK/DIO protocol. It looks like I2C but is
//  not: there is no device address, data goes LSB first, and every byte
//  is ACKed by the chip pulling DIO low on the 9th clock.
//
//  Written from the TM1651 datasheet (V1.1):
//    * Clock max 500 kHz. We run about 50 kHz so long wires are fine.
//    * Data may change only while CLK is low.
//    * START = DIO falls while CLK is high, STOP = DIO rises while CLK is high.
//
//  DIO is driven open-drain (the chip pulls it low for ACK), with the
//  ESP32's internal pull-up. Fit a 10k external pull-up to 3.3V on DIO
//  if your wires are longer than ~20 cm.
//
//  Power the board from 3.3V, NOT 5V. At 5V the TM1651 needs a 3.5V
//  logic high, more than the ESP32 can drive.
// =====================================================================
#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

class TM1651Display {
public:
  TM1651Display(uint8_t clkPin, uint8_t dioPin) : _clk(clkPin), _dio(dioPin) {}

  void begin(uint8_t brightness = 4) {
    pinMode(_clk, OUTPUT);
    pinMode(_dio, OUTPUT_OPEN_DRAIN);
    gpio_pullup_en((gpio_num_t)_dio);
    digitalWrite(_clk, HIGH);
    digitalWrite(_dio, HIGH);
    _bright = brightness > 7 ? 7 : brightness;
    _on = false;
    for (int i = 0; i < 3; i++) _shown[i] = 0xFF;   // force the first write
    const uint8_t blank[3] = {0, 0, 0};
    writeSegments(blank);
    off();
  }

  // True if the chip ACKed the last byte sent (i.e. the board is connected)
  bool present() const { return _lastAck; }

  // Raw segment bytes, digit 0 = left. bit0..6 = segments a..g.
  // Only talks to the chip if something changed.
  void writeSegments(const uint8_t seg[3]) {
    if (seg[0] == _shown[0] && seg[1] == _shown[1] && seg[2] == _shown[2]) return;
    start(); writeByte(0x40); stop();             // data cmd: write, auto-increment
    start(); writeByte(0xC0);                     // address 0
    for (int i = 0; i < 3; i++) { writeByte(seg[i] & 0x7F); _shown[i] = seg[i]; }
    writeByte(0x00);                              // address 3: decimal points off
    stop();
  }

  // Up to 3 characters, right-aligned if shorter.
  void print(const char* s) {
    uint8_t seg[3] = {0, 0, 0};
    int n = strlen(s); if (n > 3) n = 3;
    for (int i = 0; i < n; i++) seg[3 - n + i] = font(s[i]);
    writeSegments(seg);
  }

  void on() {
    if (_on) return;
    start(); writeByte(0x88 | _bright); stop();   // display ON + brightness
    _on = true;
  }
  void off() {
    start(); writeByte(0x80); stop();             // display OFF (contents kept)
    _on = false;
  }
  bool isOn() const { return _on; }
  void setBrightness(uint8_t b) { _bright = b > 7 ? 7 : b; if (_on) { _on = false; on(); } }

  // 7-segment font. Some letters are approximations, as on any 7-segment display.
  static uint8_t font(char c) {
    static const uint8_t DIGITS[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
    if (c >= '0' && c <= '9') return DIGITS[c - '0'];
    switch (c) {
      case 'A': case 'a': return 0x77;
      case 'b': case 'B': return 0x7C;
      case 'C':           return 0x39;
      case 'c':           return 0x58;
      case 'd': case 'D': return 0x5E;
      case 'E': case 'e': return 0x79;
      case 'F': case 'f': return 0x71;
      case 'H':           return 0x76;
      case 'h':           return 0x74;
      case 'L': case 'l': return 0x38;
      case 'n': case 'N': return 0x54;
      case 'o':           return 0x5C;
      case 'O':           return 0x3F;
      case 'P': case 'p': return 0x73;
      case 'r': case 'R': return 0x50;
      case 'S': case 's': return 0x6D;
      case 't': case 'T': return 0x78;
      case 'U':           return 0x3E;
      case 'u':           return 0x1C;
      case 'y': case 'Y': return 0x6E;
      case '-':           return 0x40;
      case '_':           return 0x08;
      default:            return 0x00;   // space and anything unknown
    }
  }

private:
  uint8_t _clk, _dio, _bright = 4;
  bool    _on = false;
  bool    _lastAck = false;
  uint8_t _shown[3];

  static inline void bitDelay() { delayMicroseconds(10); }

  void start() {
    digitalWrite(_dio, HIGH); digitalWrite(_clk, HIGH); bitDelay();
    digitalWrite(_dio, LOW);  bitDelay();
    digitalWrite(_clk, LOW);  bitDelay();
  }
  void stop() {
    digitalWrite(_clk, LOW);  digitalWrite(_dio, LOW); bitDelay();
    digitalWrite(_clk, HIGH); bitDelay();
    digitalWrite(_dio, HIGH); bitDelay();
  }
  bool writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {                 // LSB first
      digitalWrite(_clk, LOW);
      digitalWrite(_dio, (b & 1) ? HIGH : LOW);   // HIGH = released (open-drain)
      b >>= 1;
      bitDelay();
      digitalWrite(_clk, HIGH);
      bitDelay();
    }
    // ACK: the chip pulls DIO low after the 8th falling edge and releases
    // it on the 9th rising edge, so sample while CLK is still low.
    digitalWrite(_clk, LOW);
    digitalWrite(_dio, HIGH);                     // release the line
    bitDelay();
    bool ack = digitalRead(_dio) == LOW;
    digitalWrite(_clk, HIGH);                     // 9th clock
    bitDelay();
    digitalWrite(_clk, LOW);
    bitDelay();
    _lastAck = ack;
    return ack;
  }
};
