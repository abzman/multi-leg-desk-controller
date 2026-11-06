// =====================================================================
//  test_leg  —  motors + encoders together, open loop
//
//  For bring-up and tuning before running the full firmware:
//    * does each leg move, and does its encoder count while it does?
//    * which way does each leg go? (the firmware's 'calib' fixes directions)
//    * COUNTS_PER_MM: move the whole desk a known distance
//    * MIN_DUTY / KFF: how fast the legs go at each duty
//
//  Uses the leg->port map, inversions and COUNTS_PER_MM from Config.h,
//  plus any direction calibration saved by the firmware's 'calib'.
//  Single-leg moves are capped at duty 0.6 and 1.5 s; multi-leg moves stop
//  if the legs drift more than SYNC_FAULT_MM apart. Any key aborts.
//
//  Serial 115200:
//    pulse <leg> <duty> [ms]   one leg; prints counts, mm, mm/s
//    all <duty> [ms]           all legs open loop; prints each leg's travel
//    scale <mm>                after an 'all' move: enter the distance you
//                              measured -> suggested COUNTS_PER_MM
//    sweep                     all legs, duty 0.2..1.0 up then back down;
//                              prints speed per duty, suggests MIN_DUTY & KFF
//    show                      current counts
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "Config.h"
#include "MotorDriver.h"
#include "QuadEncoder.h"

Pca9685      pca;
MotorDriver* motor[NUM_LEGS];
QuadEncoder  enc[NUM_LEGS];
int32_t      lastAllCounts[NUM_LEGS];
bool         haveAll = false;

bool runLegs(int first, int last, float duty, uint32_t ms, int32_t* moved, float* rate = nullptr);

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
static void allStop() { for (int i = 0; i < NUM_LEGS; i++) motor[i]->set(0); }

// Run legs [first..last] at duty for ms. Fills moved[] with counts (incl.
// coasting) and, if given, rate[] with the steady speed in mm/s measured
// over the second half of the run. Returns false if aborted (key or legs
// drifting apart).
bool runLegs(int first, int last, float duty, uint32_t ms, int32_t* moved, float* rate) {
  int32_t start[NUM_LEGS], mid[NUM_LEGS];
  uint32_t tMid = 0;
  for (int i = 0; i < NUM_LEGS; i++) start[i] = enc[i].read();
  for (int i = first; i <= last; i++) motor[i]->set(duty);
  uint32_t t0 = millis();
  bool ok = true;
  while (millis() - t0 < ms) {
    if (!tMid && millis() - t0 >= ms / 2) { tMid = millis(); for (int i = 0; i < NUM_LEGS; i++) mid[i] = enc[i].read(); }
    if (keyPressed()) { Serial.println("ABORTED (key)"); ok = false; break; }
    if (last > first) {
      float lo = 1e9f, hi = -1e9f;
      for (int i = first; i <= last; i++) {
        float mm = (enc[i].read() - start[i]) / COUNTS_PER_MM;
        lo = fminf(lo, mm); hi = fmaxf(hi, mm);
      }
      if (hi - lo > SYNC_FAULT_MM) {
        int lag = first; float worst = 1e9f;                       // the leg furthest behind
        for (int i = first; i <= last; i++) {
          float mm = (enc[i].read() - start[i]) / COUNTS_PER_MM * (duty > 0 ? 1 : -1);
          if (mm < worst) { worst = mm; lag = i; }
        }
        Serial.printf("ABORTED: legs drifted %.1f mm apart (open loop). Leg %d is lagging (%.1f mm). "
                      "Overloaded, binding, or its motor/encoder isn't working?\n", hi - lo, lag, worst);
        ok = false; break;
      }
    }
    delay(2);
  }
  if (rate) {
    float dt = tMid ? (millis() - tMid) / 1000.0f : 0;
    for (int i = 0; i < NUM_LEGS; i++) rate[i] = dt > 0 ? (enc[i].read() - mid[i]) / COUNTS_PER_MM / dt : 0;
  }
  allStop();
  delay(250);                                         // let them coast to a stop before reading
  for (int i = 0; i < NUM_LEGS; i++) moved[i] = enc[i].read() - start[i];
  return ok;
}

void setup() {
#if PIN_MOTOR_POWER != PIN_NONE
  pinMode(PIN_MOTOR_POWER, OUTPUT);
  digitalWrite(PIN_MOTOR_POWER, HIGH);
#endif
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(20);
  bool ok = pca.begin(Wire, BTS_BOARD_ADDR, BTS_PWM_HZ);
  for (int i = 0; i < NUM_LEGS; i++) { motor[i] = new BtsMotor(pca, LEG_PORT[i]); motor[i]->disable(); }

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_leg ===");
  if (!ok) Serial.printf("!! Motor board not answering at 0x%02X. Run test_motor_driver first.\n", BTS_BOARD_ADDR);

  // directions: firmware calibration if saved, else Config.h defaults
  Preferences prefs;
  prefs.begin("desk", true);
  uint8_t defM = 0, defE = 0;
  for (int i = 0; i < NUM_LEGS; i++) { defM |= MOTOR_INVERT[i] << i; defE |= ENCODER_INVERT[i] << i; }
  bool cal = prefs.isKey("mInv");
  uint8_t mInv = prefs.getUChar("mInv", defM), eInv = prefs.getUChar("eInv", defE);
  prefs.end();
  for (int i = 0; i < NUM_LEGS; i++) {
    motor[i]->setInvert((mInv >> i) & 1);
    enc[i].begin(LEG_PINS[i].encA, LEG_PINS[i].encB, (eInv >> i) & 1);
    Serial.printf("leg %d: port %d, motor %s, encoder %s\n", i, LEG_PORT[i],
                  (mInv >> i) & 1 ? "INV" : "norm", (eInv >> i) & 1 ? "INV" : "norm");
  }
  Serial.printf("Directions from %s. %.1f counts/mm.\n", cal ? "saved calibration" : "Config.h (not calibrated yet)", COUNTS_PER_MM);
  Serial.println("Commands: pulse <leg> <duty> [ms] | all <duty> [ms] | scale <mm> | sweep | show");
}

void loop() {
  String line = readLine();
  if (!line.length()) return;
  char cmd[16] = "";
  float a = 0, b = 0, c = 0;
  int n = sscanf(line.c_str(), "%15s %f %f %f", cmd, &a, &b, &c);
  String s(cmd); s.toLowerCase();
  int32_t moved[NUM_LEGS];

  if (s == "pulse" && n >= 3 && a >= 0 && a < NUM_LEGS) {
    int i = (int)a;
    float duty = constrain(b, -0.6f, 0.6f);
    uint32_t ms = constrain(n >= 4 ? (uint32_t)c : 400u, 50u, 1500u);
    Serial.printf("leg %d at %+.2f for %lu ms (tilts the desk; keep it short)...\n", i, duty, (unsigned long)ms);
    float rate[NUM_LEGS];
    runLegs(i, i, duty, ms, moved, rate);
    float mm = moved[i] / COUNTS_PER_MM;
    Serial.printf("  leg %d: %+ld counts = %+.2f mm, speed %+.1f mm/s", i, (long)moved[i], mm, rate[i]);
    if (labs(moved[i]) < 3) Serial.println("  <- NO MOVEMENT: motor not turning, or encoder not counting?");
    else Serial.printf("  (count went %s for a %s command)\n", (moved[i] > 0) ? "UP" : "DOWN", duty > 0 ? "+" : "-");
  }
  else if (s == "all" && n >= 2) {
    float duty = constrain(a, -1.0f, 1.0f);
    uint32_t ms = constrain(n >= 3 ? (uint32_t)b : 1000u, 50u, 5000u);
    Serial.printf("all legs at %+.2f for %lu ms (open loop)...\n", duty, (unsigned long)ms);
    bool ok = runLegs(0, NUM_LEGS - 1, duty, ms, moved);
    double sum = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
      Serial.printf("  leg %d: %+7ld counts  %+7.2f mm\n", i, (long)moved[i], moved[i] / COUNTS_PER_MM);
      sum += moved[i]; lastAllCounts[i] = moved[i];
    }
    haveAll = ok;
    if (ok) Serial.printf("  average %+.0f counts. Measure how far the desktop moved, then: scale <mm>\n", sum / NUM_LEGS);
  }
  else if (s == "scale" && n >= 2 && a != 0) {
    if (!haveAll) { Serial.println("Do a complete 'all <duty> <ms>' move first"); return; }
    double sum = 0;
    for (int i = 0; i < NUM_LEGS; i++) sum += labs(lastAllCounts[i]);
    double cpm = sum / NUM_LEGS / fabs(a);
    Serial.printf("Suggested: #define COUNTS_PER_MM %.2ff   (now %.2f)\n", cpm, COUNTS_PER_MM);
    for (int i = 0; i < NUM_LEGS; i++)
      Serial.printf("  leg %d alone would be %.2f counts/mm\n", i, labs(lastAllCounts[i]) / fabs(a));
  }
  else if (s == "sweep") {
    Serial.println("Sweeping duty 0.2..1.0, all legs, 400 ms up then 400 ms down at each step (any key aborts)");
    Serial.println(" duty |  up mm/s | down mm/s");
    float sx = 0, sy = 0, sxx = 0, sxy = 0; int np = 0;          // least-squares fit: speed = m*duty + c
    for (int k = 1; k <= 5; k++) {
      float duty = 0.2f * k;
      int32_t mv[NUM_LEGS];
      float ru[NUM_LEGS], rd[NUM_LEGS];
      if (!runLegs(0, NUM_LEGS - 1, duty, 400, mv, ru)) return;
      if (!runLegs(0, NUM_LEGS - 1, -duty, 400, mv, rd)) return;
      float vu = 0, vd = 0;
      for (int i = 0; i < NUM_LEGS; i++) { vu += ru[i] / NUM_LEGS; vd += rd[i] / NUM_LEGS; }
      Serial.printf(" %.1f  | %+8.1f | %+8.1f\n", duty, vu, vd);
      float v = fminf(fabsf(vu), fabsf(vd));                     // the slower direction rules
      if (v > 1.0f) { sx += duty; sy += v; sxx += duty * duty; sxy += duty * v; np++; }
    }
    if (np < 2) { Serial.println("Legs barely moved during the sweep. Check the motor supply / wiring."); return; }
    float m = (np * sxy - sx * sy) / (np * sxx - sx * sx), c0 = (sy - m * sx) / np;
    float minDuty = -c0 / m, kff = (MAX_SPEED_MM_S - c0) / m;
    Serial.printf("Fit: speed = %.1f mm/s per unit duty, starts moving at duty %.2f\n", m, minDuty);
    Serial.println("Suggested starting values (the firmware's closed loop corrects the rest):");
    Serial.printf("  #define MIN_DUTY %.2ff   (now %.2f)\n", fmaxf(0.0f, minDuty), MIN_DUTY);
    if (kff <= 1.0f) Serial.printf("  #define KFF      %.2ff   (now %.2f): duty for MAX_SPEED_MM_S = %.0f mm/s\n", kff, KFF, MAX_SPEED_MM_S);
    else Serial.printf("  MAX_SPEED_MM_S (%.0f) is out of reach (needs duty %.2f): lower it to ~%.0f, and KFF to ~0.9\n",
                       MAX_SPEED_MM_S, kff, (0.9f * m + c0));
  }
  else if (s == "show") {
    for (int i = 0; i < NUM_LEGS; i++) Serial.printf("  leg %d: %+ld counts (%+.2f mm)\n", i, (long)enc[i].read(), enc[i].read() / COUNTS_PER_MM);
  }
  else Serial.println("? pulse <leg 0-3> <duty> [ms] | all <duty> [ms] | scale <mm> | sweep | show");
}
