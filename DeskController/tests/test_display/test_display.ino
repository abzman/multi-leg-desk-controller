// =====================================================================
//  test_display  —  the Gotek LEDC68 (TM1651) 3-digit display
//
//  At boot runs a self-test:
//    1. ACK check (is the chip answering on CLK/DIO?)
//    2. all segments on (888) for 2 s: every segment of every digit lit?
//    3. segment walk: lights a..g one at a time on each digit, naming
//       each on serial. A wrong name means the board's segment wiring
//       differs from the font table.
//    4. brightness ramp 0..7
//    5. counter 0..999
//    6. every message the firmware shows (Er1, H--, P 1, S 2, ...)
//
//  Then, serial 115200:
//    show <text>    up to 3 chars, e.g.  show 102   show Er2   show P 1
//    b <0-7>        brightness
//    on | off
//    seg <d> <hex>  raw segments for digit d (0=left), bit0=a .. bit6=g
//    walk | count | msgs | all   rerun parts of the self-test
//
//  Power the board from 3.3 V. With wires over ~20 cm, add 10k from DIO to 3.3 V.
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include "Config.h"
#include "TM1651Display.h"

TM1651Display disp(PIN_DISP_CLK, PIN_DISP_DIO);
uint8_t raw[3] = {0, 0, 0};
uint8_t bright = DISPLAY_BRIGHTNESS;

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}

void allOn()  { uint8_t s[3] = {0x7F, 0x7F, 0x7F}; disp.writeSegments(s); disp.on(); }

void walk() {
  const char names[] = "abcdefg";
  Serial.println("Segment walk (standard layout: a top, b top-right, c bottom-right, d bottom, e bottom-left, f top-left, g middle):");
  for (int d = 0; d < 3; d++) {
    for (int b = 0; b < 7; b++) {
      uint8_t s[3] = {0, 0, 0}; s[d] = 1 << b;
      disp.writeSegments(s);
      Serial.printf("  digit %d segment %c\n", d, names[b]);
      delay(400);
    }
  }
}

void count() {
  Serial.println("Counting 0..999");
  char buf[4];
  for (int i = 0; i <= 999; i += (i < 20 ? 1 : 7)) { snprintf(buf, sizeof buf, "%d", i); disp.print(buf); delay(15); }
  disp.print("999"); delay(300);
}

void msgs() {
  const char* m[] = {"102", " 75", "---", "H--", "Er1", "Er2", "Er3", "Er4", "P 1", "S 2", "P -", "C 0", "b 1", "End"};
  const char* what[] = {"height", "height (2 digits)", "not homed", "homing", "legs out of sync", "stall",
                        "encoder fault", "motor driver", "preset 1", "preset 2 saved", "empty preset",
                        "calibrating leg 0", "learning button 1", "calibration done"};
  Serial.println("Firmware messages:");
  for (int i = 0; i < 14; i++) { disp.print(m[i]); Serial.printf("  '%s'  %s\n", m[i], what[i]); delay(900); }
}

void selfTest() {
  Serial.println("1. all segments on (888) for 2 s");
  allOn(); delay(2000);
  Serial.println("2. segment walk");
  walk();
  Serial.println("3. brightness 0..7");
  allOn();
  for (int b = 0; b <= 7; b++) { disp.setBrightness(b); Serial.printf("  brightness %d\n", b); delay(500); }
  disp.setBrightness(bright);
  Serial.println("4. counter");
  count();
  Serial.println("5. messages");
  msgs();
  disp.print("rdy");                                 // "ready" (no k in a 7-segment font)
  Serial.println("Self-test done. Commands: show <txt> | b <0-7> | on | off | seg <d> <hex> | walk | count | msgs | all");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_display ===");
  Serial.printf("CLK GPIO %d, DIO GPIO %d\n", PIN_DISP_CLK, PIN_DISP_DIO);
  disp.begin(bright);
  if (disp.present()) Serial.println("TM1651 ACKs: wiring and power OK");
  else Serial.println("!! No ACK from the TM1651. Check 3.3V/GND, CLK<->DIO swap, and the DIO pull-up. Continuing anyway.");
  disp.on();
  selfTest();
}

void loop() {
  String line = readLine();
  if (!line.length()) return;
  String lower = line; lower.toLowerCase();
  if (lower.startsWith("show ")) {
    String t = line.substring(5);
    disp.print(t.c_str()); disp.on();
    Serial.printf("showing '%s'%s\n", t.c_str(), t.length() > 3 ? " (only 3 digits fit)" : "");
  }
  else if (lower.startsWith("b ")) { bright = constrain(line.substring(2).toInt(), 0, 7); disp.setBrightness(bright); Serial.printf("brightness %d\n", bright); }
  else if (lower == "on")  { disp.on();  Serial.println("on"); }
  else if (lower == "off") { disp.off(); Serial.println("off (contents kept)"); }
  else if (lower.startsWith("seg ")) {
    int d = -1; unsigned v = 0;
    if (sscanf(line.c_str() + 4, "%d %x", &d, &v) == 2 && d >= 0 && d < 3) {
      raw[d] = v & 0x7F; disp.writeSegments(raw); disp.on();
      Serial.printf("digit %d = 0x%02X\n", d, raw[d]);
    } else Serial.println("usage: seg <digit 0-2> <hex 00-7F>");
  }
  else if (lower == "walk")  walk();
  else if (lower == "count") count();
  else if (lower == "msgs")  msgs();
  else if (lower == "all")   selfTest();
  else Serial.println("? show <txt> | b <0-7> | on | off | seg <d> <hex> | walk | count | msgs | all");
}
