// =====================================================================
//  test_motor_driver  —  the i2c BTS7960 board on its own
//
//  Checks: I2C bus and isolator, PCA9685 address, PWM frequency, each
//  motor port in both directions, brake vs coast, raw channels, and the
//  optional motor-power kill pin.
//
//  Every run is time-limited (default 500 ms, max 3 s) and any key stops
//  it at once. Ports are the board's ports 1-4, NOT legs. 'map' shows
//  which leg is on which port per Config.h.
//
//  !! A port driving one leg on its own tilts the desk. Keep duties low
//     and runs short, or test with the motors unplugged/on the bench.
//
//  Serial 115200, commands end with Enter:
//    scan                    I2C bus scan
//    run <port> <duty> [ms]  duty -1.0..1.0 (+ = UP if wiring is standard)
//    all <duty> [ms]         all four ports at once (open loop)
//    brake <port|0>          INH on, both low sides on   (0 = all ports)
//    coast <port|0>          INH off, outputs high-impedance
//    regs                    dump PCA9685 mode/prescale/channel registers
//    ch <0-15> <0-4095>      raw PCA9685 channel duty (see table below)
//    power on|off            toggle PIN_MOTOR_POWER (if configured)
//    map                     port / channel / leg table
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "MotorDriver.h"

Pca9685   pca;
BtsMotor* port[4];
bool      boardOk = false;

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}
static bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

void scan() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " nothing found. Check the qwiic cable, 3.3V on pin 2, and the board's 24V supply.");
  if (found) Serial.println("  (PCA9685 = 0x40 + jumpers JP1..JP6; 0x70 is its all-call address)");
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(BTS_BOARD_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint8_t)BTS_BOARD_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

void dumpRegs() {
  uint8_t m1 = readReg(0x00), m2 = readReg(0x01), pre = readReg(0xFE);
  Serial.printf("MODE1 0x%02X (%s%s)  MODE2 0x%02X (%s)  PRESCALE %u -> %.0f Hz\n",
                m1, (m1 & 0x10) ? "SLEEP " : "awake ", (m1 & 0x20) ? "auto-inc" : "",
                m2, (m2 & 0x04) ? "totem-pole" : "open-drain", pre, 25000000.0f / (4096.0f * (pre + 1)));
  for (int ch = 0; ch < 16; ch++) {
    uint8_t onL = readReg(0x06 + 4 * ch), onH = readReg(0x07 + 4 * ch);
    uint8_t offL = readReg(0x08 + 4 * ch), offH = readReg(0x09 + 4 * ch);
    const char* st;
    char duty[24];
    if (offH & 0x10)      st = "full OFF";
    else if (onH & 0x10)  st = "full ON";
    else { int on = onL | ((onH & 15) << 8), off = offL | ((offH & 15) << 8);
           snprintf(duty, sizeof duty, "%4.1f %%", ((off - on) & 0xFFF) / 40.96f); st = duty; }
    Serial.printf("  ch%-2d %-9s%s", ch, st, (ch % 4 == 3) ? "\n" : " | ");
  }
}

void printMap() {
  Serial.println("port | IN A | IN B | INH | spare | output       | leg (Config.h LEG_PORT)");
  for (int p = 1; p <= 4; p++) {
    const BtsPort& b = BTS_PORTS[p - 1];
    int legOn = -1;
    for (int i = 0; i < NUM_LEGS; i++) if (LEG_PORT[i] == p) legOn = i;
    const char* out[] = {"", "J5 1-2/J2 1-2", "J5 3-4/J2 3-4", "J3 1-2/J4 1-2", "J3 3-4/J4 3-4"};
    Serial.printf("  %d  |  %2d  |  %2d  | %2d  |  %2d   | %-12s | ", p, b.inA, b.inB, b.inh, b.spare, out[p]);
    if (legOn >= 0) Serial.printf("leg %d\n", legOn); else Serial.println("-");
  }
}

void setPower(bool on) {
#if PIN_MOTOR_POWER != PIN_NONE
  digitalWrite(PIN_MOTOR_POWER, on ? HIGH : LOW);
  Serial.printf("Motor power %s\n", on ? "ON" : "OFF");
  if (on) { delay(150); boardOk = pca.reinit(); for (auto m : port) m->disable(); }
#else
  (void)on;
  Serial.println("PIN_MOTOR_POWER is not configured in Config.h");
#endif
}

// Drive the given ports for ms, abort on any key, then brake.
void timedRun(int first, int last, float duty, uint32_t ms) {
  if (!boardOk) { Serial.println("Board not initialised. Check 'scan'."); return; }
  ms = constrain(ms, 50u, 3000u);
  duty = constrain(duty, -1.0f, 1.0f);
  if (first == last) Serial.printf("Driving port %d at %+.2f for %lu ms (any key stops)...\n", first, duty, (unsigned long)ms);
  else               Serial.printf("Driving ports %d-%d at %+.2f for %lu ms (any key stops)...\n", first, last, duty, (unsigned long)ms);
  uint32_t errs0 = pca.errors();
  for (int p = first; p <= last; p++) port[p - 1]->set(duty);
  uint32_t t0 = millis();
  bool aborted = false;
  while (millis() - t0 < ms) { if (keyPressed()) { aborted = true; break; } delay(5); }
  for (int p = first; p <= last; p++) port[p - 1]->set(0);    // brake
  uint32_t errs = pca.errors() - errs0;
  Serial.printf("%s after %lu ms. I2C errors during run: %lu\n", aborted ? "STOPPED" : "Done",
                (unsigned long)(millis() - t0), (unsigned long)errs);
  if (errs) Serial.println("!! The board missed commands, so it may NOT have stopped. Cut the 24 V if a motor is still running.\n"
                           "   Check the qwiic cable, the 3.3 V on its pin 2, and the I2C pull-ups (see Config.h I2C_FREQ_HZ).");
}

void setup() {
#if PIN_MOTOR_POWER != PIN_NONE
  pinMode(PIN_MOTOR_POWER, OUTPUT);
  digitalWrite(PIN_MOTOR_POWER, HIGH);
#endif
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(20);
  for (int p = 0; p < 4; p++) port[p] = new BtsMotor(pca, p + 1);
  boardOk = pca.begin(Wire, BTS_BOARD_ADDR, BTS_PWM_HZ);      // all outputs off first
  for (int p = 0; p < 4; p++) port[p]->disable();

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_motor_driver ===");
  Serial.printf("I2C SDA %d SCL %d @ %d Hz, board address 0x%02X\n", PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ, BTS_BOARD_ADDR);
  scan();
  if (boardOk) Serial.printf("PCA9685 OK at 0x%02X, PWM %.0f Hz. All outputs off.\n", BTS_BOARD_ADDR, pca.actualHz());
  else         Serial.printf("PCA9685 NOT answering at 0x%02X. Check BTS_BOARD_ADDR against the scan above.\n", BTS_BOARD_ADDR);
  printMap();
  Serial.println("Commands: scan | run <port> <duty> [ms] | all <duty> [ms] | brake <p> | coast <p> | regs | ch <n> <val> | power on|off | map");
}

void loop() {
  String line = readLine();
  if (!line.length()) return;
  char cmd[16] = ""; char word[8] = "";
  float a = 0, b = 0, c = 0;
  int n = sscanf(line.c_str(), "%15s %f %f %f", cmd, &a, &b, &c);
  String s(cmd); s.toLowerCase();

  if (s == "scan") scan();
  else if (s == "map") printMap();
  else if (s == "regs") dumpRegs();
  else if (s == "run" && n >= 3 && a >= 1 && a <= 4) timedRun((int)a, (int)a, b, n >= 4 ? (uint32_t)c : 500);
  else if (s == "all" && n >= 2) timedRun(1, 4, a, n >= 3 ? (uint32_t)b : 500);
  else if ((s == "brake" || s == "coast") && n >= 2) {
    int lo = a == 0 ? 1 : (int)a, hi = a == 0 ? 4 : (int)a;
    for (int p = lo; p <= hi && p >= 1 && p <= 4; p++) {
      if (s == "brake") port[p - 1]->set(0); else port[p - 1]->disable();
    }
    Serial.printf("%s port(s) %d-%d\n", s == "brake" ? "Braked" : "Coasting", lo, hi);
  }
  else if (s == "ch" && n >= 3 && a >= 0 && a <= 15) {
    pca.setDuty((uint8_t)a, (uint16_t)constrain((int)b, 0, 4095));
    Serial.printf("ch%d = %d/4095%s\n", (int)a, (int)b,
                  ((int)a == 0 || (int)a == 5 || (int)a == 9 || (int)a == 12) ? " (spare channel on J9/J10 header)" : "");
  }
  else if (s == "power" && sscanf(line.c_str(), "%*s %7s", word) == 1) setPower(!strcmp(word, "on"));
  else Serial.println("? scan | run <port 1-4> <duty -1..1> [ms] | all <duty> [ms] | brake <p|0> | coast <p|0> | regs | ch <0-15> <0-4095> | power on|off | map");
}
