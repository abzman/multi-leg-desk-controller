// =====================================================================
//  test_settings  —  view or reset what the firmware keeps in flash
//
//  The firmware stores its settings in the ESP32's NVS, namespace "desk".
//  This sketch reads them (or selectively clears them) without
//  touching any hardware. Flashing it doesn't erase them, so you can go
//  back to the firmware afterwards and everything is still there.
//
//  Serial 115200:
//    dump                 show everything (also runs at boot)
//    clear position       forget the position -> firmware asks to home
//    clear presets        delete presets 1-4
//    clear directions     forget 'calib' results -> back to Config.h
//    clear buttons        forget 'btncal' levels -> back to ideal R-2R
//    clear all yes        erase the whole "desk" namespace
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"

Preferences prefs;

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}

static void key(char* k, char prefix, int i) { k[0] = prefix; k[1] = '0' + i; k[2] = 0; }

void dump() {
  char k[4];
  Serial.println("--- desk settings in flash ---");
  bool homed = prefs.getBool("homed", false), moving = prefs.getBool("moving", false);
  Serial.printf("homed: %s   moving flag: %s%s\n", prefs.isKey("homed") ? (homed ? "yes" : "no") : "(never saved)",
                moving ? "SET" : "clear", moving ? "  <- power was lost mid-move; firmware will ask to home" : "");
  Serial.print("leg positions:");
  for (int i = 0; i < NUM_LEGS; i++) {
    key(k, 'c', i);
    if (prefs.isKey(k)) { long c = prefs.getInt(k, 0); Serial.printf("  leg %d %ld counts (%.1f mm)", i, c, c / COUNTS_PER_MM); }
    else Serial.printf("  leg %d -", i);
  }
  Serial.println();
  Serial.print("presets:");
  for (int n = 0; n < NUM_PRESETS; n++) {
    key(k, 'p', n);
    float v = prefs.getFloat(k, NAN);
    if (isnan(v)) Serial.printf("  P%d empty", n + 1);
    else Serial.printf("  P%d %.1f mm (desk %.1f cm)", n + 1, v, (v + HEIGHT_OFFSET_MM) / 10.0f);
  }
  Serial.println();
  if (prefs.isKey("mInv")) {
    uint8_t m = prefs.getUChar("mInv", 0), e = prefs.getUChar("eInv", 0);
    Serial.print("directions (from calib):");
    for (int i = 0; i < NUM_LEGS; i++) Serial.printf("  leg %d motor %s enc %s", i, (m >> i) & 1 ? "INV" : "norm", (e >> i) & 1 ? "INV" : "norm");
    Serial.println();
  } else Serial.println("directions: not calibrated (firmware uses Config.h MOTOR_INVERT / ENCODER_INVERT)");
  if (prefs.isKey("b0")) {
    Serial.print("preset-button levels (from btncal):");
    for (int n = 0; n < NUM_PRESETS; n++) { key(k, 'b', n); Serial.printf("  P%d %u mV", n + 1, prefs.getUShort(k, 0)); }
    Serial.println();
  } else Serial.println("preset-button levels: not learned (firmware uses ideal R-2R levels)");
  Serial.printf("free NVS entries: %u\n", (unsigned)prefs.freeEntries());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_settings ===");
  prefs.begin("desk", false);
  dump();
  Serial.println("Commands: dump | clear position | clear presets | clear directions | clear buttons | clear all yes");
}

void loop() {
  String line = readLine();
  if (!line.length()) return;
  char k[4];
  if (line == "dump") dump();
  else if (line == "clear position") {
    for (int i = 0; i < NUM_LEGS; i++) { key(k, 'c', i); prefs.remove(k); }
    prefs.remove("homed"); prefs.remove("moving");
    Serial.println("Position forgotten. The firmware will ask you to home.");
  }
  else if (line == "clear presets") {
    for (int n = 0; n < NUM_PRESETS; n++) { key(k, 'p', n); prefs.remove(k); }
    Serial.println("Presets deleted.");
  }
  else if (line == "clear directions") {
    prefs.remove("mInv"); prefs.remove("eInv");
    for (int i = 0; i < NUM_LEGS; i++) { key(k, 'c', i); prefs.remove(k); }
    prefs.remove("homed");
    Serial.println("Calibration forgotten (position too, since encoder sign may change). Run 'calib' in the firmware.");
  }
  else if (line == "clear buttons") {
    for (int n = 0; n < NUM_PRESETS; n++) { key(k, 'b', n); prefs.remove(k); }
    Serial.println("Button levels forgotten. Run 'btncal' in the firmware.");
  }
  else if (line == "clear all yes") { prefs.clear(); Serial.println("All desk settings erased."); }
  else if (line == "clear all") Serial.println("This erases everything. Type 'clear all yes' to confirm.");
  else Serial.println("? dump | clear position | clear presets | clear directions | clear buttons | clear all yes");
}
