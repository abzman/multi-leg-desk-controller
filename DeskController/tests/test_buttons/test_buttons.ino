// =====================================================================
//  test_buttons  —  the UP / DOWN buttons on their own
//
//  Prints every press and release with hold time, counts contact bounce
//  (raw edges inside the firmware's 30 ms debounce window), and spots the
//  "both held" combo the firmware uses for clear-fault + re-home.
//
//  Wiring: each button from its GPIO to GND (internal pull-ups are used).
//  Idle should read 1 and pressed 0. A pin that reads 0 with nothing
//  pressed is shorted to GND or wired to the wrong pin.
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include "Config.h"

struct Btn {
  const char* name; uint8_t pin;
  bool stable = false;            // debounced: true = pressed
  bool raw = false;
  uint32_t lastEdge = 0, pressedAt = 0;
  uint16_t bounces = 0;           // raw edges counted for the current press
};
Btn btn[2] = { {"UP", PIN_BTN_UP}, {"DOWN", PIN_BTN_DOWN} };
uint32_t bothSince = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_buttons ===");
  for (auto& b : btn) {
    pinMode(b.pin, INPUT_PULLUP);
    delay(2);
    b.raw = b.stable = digitalRead(b.pin) == LOW;
    Serial.printf("%-4s on GPIO %d reads %d %s\n", b.name, b.pin, digitalRead(b.pin),
                  b.raw ? " <- reads PRESSED at rest: shorted, or wired to the wrong pin?" : "(idle, ok)");
  }
  Serial.println("Press, hold and release the buttons. Hold both for 3 s to see the combo.");
}

void loop() {
  uint32_t now = millis();
  for (auto& b : btn) {
    bool r = digitalRead(b.pin) == LOW;
    if (r != b.raw) {                                  // raw edge
      b.raw = r;
      if (now - b.lastEdge < 30) b.bounces++;
      b.lastEdge = now;
    }
    if (b.raw != b.stable && now - b.lastEdge >= 30) { // stable for 30 ms -> debounced change
      b.stable = b.raw;
      if (b.stable) {
        b.pressedAt = now;
        Serial.printf("[%7.2f s] %-4s pressed\n", now / 1000.0f, b.name);
      } else {
        Serial.printf("[%7.2f s] %-4s released after %lu ms, %u bounce edge%s%s\n", now / 1000.0f, b.name,
                      (unsigned long)(now - b.pressedAt), b.bounces, b.bounces == 1 ? "" : "s",
                      b.bounces > 6 ? "  <- very bouncy; consider a 100nF cap across the switch" : "");
        b.bounces = 0;
      }
    }
  }

  static bool comboReported = false;
  bool both = btn[0].stable && btn[1].stable;
  if (both && !bothSince) { bothSince = now ? now : 1; Serial.println("            both held..."); }
  if (both && !comboReported && now - bothSince >= 3000) {
    Serial.println("            both held 3 s -> firmware would clear a fault and re-home");
    comboReported = true;                              // once per hold
  }
  if (!both) { bothSince = 0; comboReported = false; }
}
