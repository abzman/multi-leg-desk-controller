// =====================================================================
//  test_preset_ladder  —  the 4 preset buttons on the R-2R ladder
//
//  Shows the live ADC voltage and decodes it exactly like the firmware:
//  the same tolerance rule, and the levels learned by the firmware's
//  'btncal' if saved (else the ideal R-2R levels). Each press reports
//  its average voltage, its noise and how much margin is left before it
//  would misread.
//
//  Schematic: docs/preset_ladder.png in the main sketch folder.
//
//  Serial 115200:
//    live     toggle a live voltage bar (5 Hz)
//    levels   show the levels in use
//    noise    1 s noise measurement of whatever is (not) pressed now
//    table    summary of every button seen so far vs expected
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"

uint16_t level[NUM_PRESETS];      // expected mV per button P1..P4
bool     learned = false, live = false;

// per-button results
uint32_t seenSum[NUM_PRESETS], seenN[NUM_PRESETS];
uint16_t seenMin[NUM_PRESETS], seenMax[NUM_PRESETS];

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}

uint16_t readMv() { return (analogReadMilliVolts(PIN_PRESET_ADC) + analogReadMilliVolts(PIN_PRESET_ADC)) / 2; }
uint16_t tolerance() { return level[NUM_PRESETS - 1] * 40 / 100; }

// same rule as the firmware: -1 none, 0..3 button, -2 not a clean level
int decode(uint16_t mv) {
  uint16_t lsb = level[NUM_PRESETS - 1], tol = tolerance();
  if (mv < lsb / 2) return -1;
  for (int k = 0; k < NUM_PRESETS; k++) if (abs((int)mv - (int)level[k]) <= tol) return k;
  return -2;
}

void printLevels() {
  Serial.printf("Levels (%s): ", learned ? "learned by btncal" : "ideal R-2R, not calibrated");
  for (int k = 0; k < NUM_PRESETS; k++) Serial.printf("P%d %u mV  ", k + 1, level[k]);
  Serial.printf("| tolerance +/-%u mV, 'none' below %u mV\n", tolerance(), level[NUM_PRESETS - 1] / 2);
}

void noise() {
  uint32_t sum = 0, sum2 = 0, n = 0; uint16_t lo = 65535, hi = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    uint16_t v = readMv();
    sum += v; sum2 += (uint32_t)v * v; n++; lo = min(lo, v); hi = max(hi, v);
    delay(2);
  }
  float mean = (float)sum / n, sd = sqrtf(fmaxf(0, (float)sum2 / n - mean * mean));
  Serial.printf("1 s, %lu samples: mean %.0f mV, min %u, max %u, std dev %.1f mV -> reads as %d\n",
                (unsigned long)n, mean, lo, hi, sd, decode((uint16_t)mean) + 1);
  if (hi - lo > tolerance()) Serial.println("  !! noise spread exceeds the tolerance: add/check the 100 nF cap on GPIO 33, shorten the ladder wires");
}

void table() {
  Serial.println("button | expected | seen avg | seen min..max | margin left");
  for (int k = 0; k < NUM_PRESETS; k++) {
    if (!seenN[k]) { Serial.printf("  P%d   | %5u mV | (not pressed yet)\n", k + 1, level[k]); continue; }
    uint16_t avg = seenSum[k] / seenN[k];
    int worst = max(abs((int)seenMin[k] - (int)level[k]), abs((int)seenMax[k] - (int)level[k]));
    Serial.printf("  P%d   | %5u mV | %5u mV | %4u..%-4u mV | %d mV%s\n", k + 1, level[k], avg, seenMin[k], seenMax[k],
                  (int)tolerance() - worst, (int)tolerance() - worst < 30 ? "  <- tight: run btncal in the firmware" : "");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_preset_ladder ===");
  Serial.printf("Ladder on GPIO %d (ADC1), reference %d mV\n", PIN_PRESET_ADC, PRESET_SUPPLY_MV);
  for (int k = 0; k < NUM_PRESETS; k++) level[k] = PRESET_SUPPLY_MV >> (k + 1);
  Preferences prefs;
  prefs.begin("desk", true);
  for (int k = 0; k < NUM_PRESETS; k++) {
    char key[4] = "b0"; key[1] = '0' + k;
    if (prefs.isKey(key)) { level[k] = prefs.getUShort(key, level[k]); learned = true; }
  }
  prefs.end();
  for (int k = 0; k < NUM_PRESETS; k++) { seenMin[k] = 65535; seenMax[k] = 0; }
  printLevels();
  uint16_t idle = readMv();
  Serial.printf("Idle reading: %u mV %s\n", idle, idle < level[NUM_PRESETS - 1] / 2 ? "(ok)" :
                "<- should be ~0 with nothing pressed: check the 2R terminator / pull-downs");
  Serial.println("Press each button (and try two at once). Commands: live | levels | noise | table");
}

void loop() {
  String line = readLine();
  if (line == "live") live = !live;
  else if (line == "levels") printLevels();
  else if (line == "noise") noise();
  else if (line == "table") table();
  else if (line.length()) Serial.println("? live | levels | noise | table");

  static uint32_t lastS = 0;
  if (millis() - lastS < 10) return;                   // 100 Hz like the firmware
  lastS = millis();
  uint16_t mv = readMv();
  int d = decode(mv);

  // debounce exactly like the firmware: same decode 3 samples in a row
  static int cand = -1, count = 0, stable = -1;
  static uint32_t sum = 0, n = 0; static uint16_t lo = 65535, hi = 0;
  if (d == cand) { if (count < 3) count++; } else { cand = d; count = 1; }
  if (stable >= 0 && d == stable) { sum += mv; n++; lo = min(lo, mv); hi = max(hi, mv); }
  if (count == 3 && cand != stable && cand != -2) {
    if (stable >= 0 && n) {                              // a press ended: report it
      uint16_t avg = sum / n;
      seenSum[stable] += avg; seenN[stable]++;
      seenMin[stable] = min(seenMin[stable], lo); seenMax[stable] = max(seenMax[stable], hi);
      int off = (int)avg - (int)level[stable];
      Serial.printf("  P%d released: avg %u mV (%+d from expected), noise %u mV p-p, margin %d mV\n",
                    stable + 1, avg, off, hi - lo, (int)tolerance() - max(abs((int)lo - (int)level[stable]), abs((int)hi - (int)level[stable])));
    }
    stable = cand;
    sum = 0; n = 0; lo = 65535; hi = 0;
    if (stable >= 0) Serial.printf("P%d pressed (%u mV)\n", stable + 1, mv);
  }
  static uint32_t lastReject = 0;
  if (d == -2 && count == 3 && millis() - lastReject > 500) {
    lastReject = millis();
    Serial.printf("  %u mV: not a single-button level (two buttons at once?) - ignored, like the firmware\n", mv);
  }

  static uint32_t lastLive = 0;
  if (live && millis() - lastLive >= 200) {
    lastLive = millis();
    char bar[41]; int len = constrain(mv * 40 / 3300, 0, 40);
    for (int i = 0; i < 40; i++) bar[i] = i < len ? '#' : '.';
    bar[40] = 0;
    Serial.printf("%4u mV [%s] %s\n", mv, bar, d == -1 ? "none" : d == -2 ? "??" : (String("P") + (d + 1)).c_str());
  }
}
