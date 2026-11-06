// =====================================================================
//  test_encoders  —  the four quadrature encoders on their own
//
//  Shows each leg's count, speed and the raw A/B pin levels, so you can
//  spot dead lines, missing pull-ups and swapped wires. Nothing is
//  driven. Move a leg with test_leg, by hand with the motor unplugged,
//  or by spinning a bench motor.
//
//  What to look for:
//   * Raw levels stuck at 0 -> missing pull-up, or the sensor isn't powered
//   * One of A/B never changes while the other does -> that line's broken;
//     the count will jitter +1/-1 instead of climbing
//   * Count goes the wrong way -> fine; the firmware's 'calib' fixes it
//   * Count jumps by hundreds with the motor still -> electrical noise:
//     shorten/shield the encoder wires and keep them away from motor leads
//
//  Serial 115200:
//    z        zero all counts
//    raw      toggle the raw A/B level display
//    fast     toggle 5 Hz / 20 Hz updates
//    p        pause / resume printing
// =====================================================================
#define ENABLE_HOME_ASSISTANT 0
#include <Arduino.h>
#include "Config.h"
#include "QuadEncoder.h"

QuadEncoder enc[NUM_LEGS];
int32_t  last[NUM_LEGS];
int32_t  minSeen[NUM_LEGS], maxSeen[NUM_LEGS];
bool     showRaw = true, fast = false, paused = false;
uint8_t  rawChanges[NUM_LEGS][2];          // how often each line was seen changing (sampled)
uint8_t  prevRaw[NUM_LEGS][2];

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length()) { String l = buf; buf = ""; return l; } }
    else if (buf.length() < 80) buf += c;
  }
  return "";
}

void zeroAll() {
  for (int i = 0; i < NUM_LEGS; i++) {
    enc[i].set(0); last[i] = 0; minSeen[i] = maxSeen[i] = 0;
    rawChanges[i][0] = rawChanges[i][1] = 0;
    prevRaw[i][0] = digitalRead(LEG_PINS[i].encA); prevRaw[i][1] = digitalRead(LEG_PINS[i].encB);
  }
  Serial.println("Counts zeroed");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== test_encoders ===");
  for (int i = 0; i < NUM_LEGS; i++) {
    bool ok = enc[i].begin(LEG_PINS[i].encA, LEG_PINS[i].encB, ENCODER_INVERT[i]);
    Serial.printf("leg %d: A=GPIO%-2d B=GPIO%-2d %s%s\n", i, LEG_PINS[i].encA, LEG_PINS[i].encB,
                  ok ? "ok" : "INIT FAILED",
                  (LEG_PINS[i].encA >= 34 || LEG_PINS[i].encB >= 34) ? "  (34-39 need EXTERNAL pull-ups)" : "");
  }
  Serial.printf("Backend: %s, glitch filter %d ns, %.1f counts/mm (Config.h)\n",
                enc[0].backend(), ENCODER_GLITCH_NS, COUNTS_PER_MM);
  zeroAll();
  Serial.println("Commands: z | raw | fast | p");
}

void loop() {
  // sample raw pin levels as fast as loop() runs to catch lines that toggle
  for (int i = 0; i < NUM_LEGS; i++) {
    uint8_t a = digitalRead(LEG_PINS[i].encA), b = digitalRead(LEG_PINS[i].encB);
    if (a != prevRaw[i][0] && rawChanges[i][0] < 255) rawChanges[i][0]++;
    if (b != prevRaw[i][1] && rawChanges[i][1] < 255) rawChanges[i][1]++;
    prevRaw[i][0] = a; prevRaw[i][1] = b;
  }

  String line = readLine();
  if (line == "z") zeroAll();
  else if (line == "raw") showRaw = !showRaw;
  else if (line == "fast") fast = !fast;
  else if (line == "p") { paused = !paused; Serial.println(paused ? "paused" : "resumed"); }
  else if (line.length()) Serial.println("? z | raw | fast | p");

  static uint32_t lastPrint = 0;
  uint32_t period = fast ? 50 : 200;
  if (paused || millis() - lastPrint < period) return;
  float dt = (millis() - lastPrint) / 1000.0f;
  lastPrint = millis();

  for (int i = 0; i < NUM_LEGS; i++) {
    int32_t c = enc[i].read();
    float rate = (c - last[i]) / dt;
    last[i] = c;
    if (c < minSeen[i]) minSeen[i] = c;
    if (c > maxSeen[i]) maxSeen[i] = c;
    Serial.printf("L%d %+8ld (%+7.1f mm) %+7.0f c/s", i, (long)c, c / COUNTS_PER_MM, rate);
    if (showRaw) {
      Serial.printf(" AB=%d%d", prevRaw[i][0], prevRaw[i][1]);
      // a line that never changed while the count moved a lot is suspicious
      bool moved = (maxSeen[i] - minSeen[i]) > 40;
      if (moved && rawChanges[i][0] == 0) Serial.print(" A never changed?");
      else if (moved && rawChanges[i][1] == 0) Serial.print(" B never changed?");
    }
    Serial.print(i < NUM_LEGS - 1 ? " | " : "\n");
  }
}
