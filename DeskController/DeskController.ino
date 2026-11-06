// =====================================================================
//  4-Leg Synchronized Standing Desk Controller — ESP32 / Arduino
// ---------------------------------------------------------------------
//  How the sync works
//  ------------------
//  1. One shared SETPOINT moves along a trapezoidal profile (soft
//     accel, cruise, soft decel) toward the goal height.
//  2. Each leg runs its own position loop (feed-forward + PI) and
//     tracks that one setpoint, so all four legs chase the same target
//     instead of each running at "about the same speed".
//  3. LEAD CLAMP: the setpoint may never get more than MAX_LEAD_MM ahead
//     of the slowest leg. If one leg is heavily loaded (monitor
//     corner), the whole desk slows down to match it.
//  4. SAFETY: any two legs more than SYNC_FAULT_MM apart, any leg
//     stalled (obstruction), or an implausible encoder speed cuts power
//     to all legs at once.
//
//  Homing: all legs drive down slowly in sync. Each leg zeroes itself
//  when it stalls on its bottom stop, then the desk backs off a few mm.
//
//  Serial (115200, newline-terminated):
//    home | up | down | stop | goto <mm> | level | p <1-4> | save <1-4>
//    status | log | clear | zero | test <leg> <duty> | kp/ki/kff <val>
//    calib | calib reset     (direction calibration, see below)
//    btncal                  (learn the preset-button ladder voltages)
//    setp <1-4> <mm>         (set a preset height without moving)
//    reboot                  (only when stopped)
//  Buttons: hold UP/DOWN to move. Any press during an automatic move
//  stops it. Hold both for 3 s to clear a fault and re-home.
//  Preset buttons P1-P4 (R-2R ladder on one ADC pin): tap = go to preset,
//  hold 2 s = save current height to it.
//
//  Display (Gotek LEDC68 / TM1651, 3 digits): desk height in cm, or
//  Er1 (legs out of sync), Er2 (stall/obstruction), Er3 (encoder fault),
//  Er4 (motor driver board not answering on I2C),
//  "---" not homed, "H--" homing, "P n"/"S n" preset recalled/saved.
//  Goes dark 5 s after the last button or serial input (stays lit while moving).
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "Config.h"
#include "QuadEncoder.h"
#include "MotorDriver.h"
#include "TM1651Display.h"
#include "DeskLog.h"
#if CONTROL_TASK
#include "esp_task_wdt.h"
#endif

DeskLog Out;                       // serial output + "last message" for Home Assistant

// ---------------------------------------------------------------------
//  Locking. The control loop runs in its own task (CONTROL_TASK = 1).
//  Serial, buttons, display and Home Assistant run in loop(). Every entry
//  point that touches controller state takes this lock, so a command
//  can never land half-way through a control tick. It's recursive, so
//  nested calls are fine.
// ---------------------------------------------------------------------
#if CONTROL_TASK
SemaphoreHandle_t deskMutex;
struct DeskLock {
  DeskLock()  { xSemaphoreTakeRecursive(deskMutex, portMAX_DELAY); }
  ~DeskLock() { xSemaphoreGiveRecursive(deskMutex); }
};
#else
struct DeskLock { DeskLock() {} };
#endif

// Explicit prototypes for functions with default arguments (keeps the
// Arduino IDE's auto-generated prototypes from clashing with them).
void startMove(float target, float speed, bool homing = false);
void jog(int dir, bool fromButton = false);
#if CONTROL_TASK
void controlTask(void*);
#endif

#if ENABLE_HOME_ASSISTANT
void haBegin();                    // HomeAssistant.h, included at the end of this file
void haLoop();
#endif

// ---------------------------------------------------------------------
enum Mode : uint8_t { M_IDLE, M_MOVING, M_HOMING, M_FAULT, M_TEST, M_CALIB };
static const char* MODE_NAMES[] = { "IDLE", "MOVING", "HOMING", "FAULT", "TEST", "CALIB" };

// Fault codes, shown on the display as "Er1", "Er2", ...
enum FaultCode : uint8_t {
  FAULT_NONE    = 0,
  FAULT_SYNC    = 1,   // Er1: legs drifted apart by more than SYNC_FAULT_MM
  FAULT_STALL   = 2,   // Er2: a leg stalled, obstruction or end stop
  FAULT_ENCODER = 3,   // Er3: implausible encoder speed, noise or wiring
  FAULT_DRIVER  = 4,   // Er4: motor driver board not answering on I2C
};

struct Leg {
  float    pos = 0, prevPos = 0, vel = 0;   // mm, mm, mm/s
  float    integ = 0, out = 0;              // integral state, motor command
  float    stallRef = 0;
  uint32_t stallSince = 0;
  bool     active = false;                  // taking part in the current move
};

QuadEncoder  enc[NUM_LEGS];
MotorDriver* motor[NUM_LEGS];
Leg          leg[NUM_LEGS];
Preferences  prefs;

Pca9685      pca;                  // the i2c BTS7960 board

Mode     mode = M_IDLE;
bool     homed = false;
float    sp = 0, spVel = 0, goal = 0, vmax = MAX_SPEED_MM_S;
uint32_t arrivedSince = 0;
float    kp = KP, ki = KI, kff = KFF;
bool     logging = false;
String   faultMsg;
FaultCode faultCode = FAULT_NONE;
float    presets[NUM_PRESETS];

// Per-leg direction settings. Defaults come from Config.h; `calib`
// overrides them and saves them to flash.
bool     motorInv[NUM_LEGS], encInv[NUM_LEGS];

// display
TM1651Display display(PIN_DISP_CLK, PIN_DISP_DIO);
uint32_t lastActivity = 0;         // last button/serial input (or motion)
uint32_t msgUntil = 0;             // a short message is showing until then
char     msgText[4] = "";
float    shownHeight = -1e9f;      // for display hysteresis

void wakeDisplay()                    { lastActivity = millis(); }
void flashMessage(const char* text) {
  strncpy(msgText, text, 3); msgText[3] = 0;
  msgUntil = millis() + DISPLAY_MSG_MS;
  wakeDisplay();
}

// test mode
int      testLeg = -1;
uint32_t testUntil = 0;
int32_t  testStartCount = 0;
Mode     testPrevMode = M_IDLE;

// buttons
bool     jogging = false, swallowPress = false;
uint32_t bothHeldSince = 0;

// ---------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------
static inline float sgn(float x) { return (x > 0) - (x < 0); }

void allMotorsOff() {
  for (int i = 0; i < NUM_LEGS; i++) { leg[i].out = 0; motor[i]->set(0); }
}

float meanPos() {
  float s = 0;
  for (int i = 0; i < NUM_LEGS; i++) s += leg[i].pos;
  return s / NUM_LEGS;
}

void persistPositions(bool moving) {
#if PERSIST_POSITION
  char key[4] = "c0";
  for (int i = 0; i < NUM_LEGS; i++) { key[1] = '0' + i; prefs.putInt(key, enc[i].read()); }
  prefs.putBool("homed", homed);
  prefs.putBool("moving", moving);
#endif
}

void markMoving() {
#if PERSIST_POSITION
  prefs.putBool("moving", true);   // if power dies mid-move, re-home on boot
#endif
}

void setMotorPower(bool on) {
#if PIN_MOTOR_POWER != PIN_NONE
  digitalWrite(PIN_MOTOR_POWER, on ? HIGH : LOW);
#endif
  (void)on;
}

// (Re)initialise the motor driver board. Returns false if it doesn't answer.
bool initMotorDriver() {
  if (!pca.begin(Wire, BTS_BOARD_ADDR, BTS_PWM_HZ)) return false;
  for (int i = 0; i < NUM_LEGS; i++) motor[i]->begin();   // INH off, all off
  return pca.ok();
}

void fault(FaultCode code, const String& msg);

// Clear a fault: restore motor power, re-check the driver board.
bool clearFault() {
  if (mode != M_FAULT) return true;
  setMotorPower(true);
#if PIN_MOTOR_POWER != PIN_NONE
  delay(150);                          // let the board's buck + LDO + PCA9685 start
#endif
  if (!initMotorDriver()) {
    Out.printf("Motor driver at 0x%02X still not answering\n", BTS_BOARD_ADDR);
    fault(FAULT_DRIVER, "motor driver not answering on I2C");
    return false;
  }
  mode = M_IDLE; faultMsg = ""; faultCode = FAULT_NONE;
  Out.println("Fault cleared");
  return true;
}

void fault(FaultCode code, const String& msg) {
  allMotorsOff();
  if (code == FAULT_DRIVER) for (int i = 0; i < NUM_LEGS; i++) motor[i]->disable();
  setMotorPower(false);            // no-op unless PIN_MOTOR_POWER is wired
  mode = M_FAULT;
  jogging = false;
  faultMsg = msg;
  faultCode = code;
  spVel = 0;
  wakeDisplay();                   // always show a fault, even if the display was dark
  Out.printf("!! FAULT E%d: %s  (send 'clear', or hold both buttons 3 s to re-home)\n", code, msg.c_str());
  persistPositions(false);         // encoders still counted, positions still valid
}

// ---------------------------------------------------------------------
//  Motion commands
// ---------------------------------------------------------------------
void startMove(float target, float speed, bool homing) {
  if (mode == M_FAULT) { Out.println("In FAULT: 'clear' first"); return; }
  if (mode == M_TEST) return;

  if (homed && !homing) target = constrain(target, MIN_HEIGHT_MM, MAX_HEIGHT_MM);

  if (mode == M_IDLE) {
    // Fresh move: start the setpoint at the average leg height. If the
    // legs are slightly out of level, this levels them as the move starts.
    sp = meanPos();
    spVel = 0;
    uint32_t now = millis();
    for (int i = 0; i < NUM_LEGS; i++) {
      leg[i].integ = 0;
      leg[i].out = 0;
      leg[i].active = true;
      leg[i].stallRef = leg[i].pos;
      leg[i].stallSince = now;
    }
    if (pca.looksReset() && !initMotorDriver()) {
      fault(FAULT_DRIVER, "motor driver not answering on I2C");
      return;
    }
    markMoving();
  }
  goal = target;
  vmax = speed;
  mode = homing ? M_HOMING : M_MOVING;
  arrivedSince = 0;
}

// Decelerate smoothly to a stop from the current speed
void softStop() {
  jogging = false;
  if (mode == M_HOMING) mode = M_MOVING;              // abort homing
  if (mode != M_MOVING) return;
  goal = sp + sgn(spVel) * (spVel * spVel) / (2.0f * ACCEL_MM_S2);
}

void startHoming() {
  if (mode == M_FAULT && !clearFault()) return;
  if (mode != M_IDLE) { Out.println("Stop first"); return; }
  homed = false;
  for (int i = 0; i < NUM_LEGS; i++) {               // relative frame for homing
    enc[i].set(0);
    leg[i].pos = leg[i].prevPos = leg[i].vel = 0;
  }
  Out.println("Homing: driving down to the end stops...");
  startMove(-100000.0f, HOMING_SPEED_MM_S, true);
}

// fromButton: held-button jog, which stops when the button is released.
// Serial / Home Assistant "up"/"down" run until "stop", a limit, or any
// button press, like any other automatic move.
void jog(int dir, bool fromButton) {
  float target;
  if (homed) target = dir > 0 ? MAX_HEIGHT_MM : MIN_HEIGHT_MM;
  else       target = sp + dir * 2000.0f;             // no limits until homed
  startMove(target, MAX_SPEED_MM_S);
  if (mode == M_MOVING) jogging = fromButton;
}

void finishMove() {
  allMotorsOff();
  mode = M_IDLE;
  jogging = false;
  spVel = 0;
  persistPositions(false);
  Out.printf("Arrived: %.1f mm (desk %.1f mm)\n", meanPos(), meanPos() + HEIGHT_OFFSET_MM);
}

// ---------------------------------------------------------------------
//  Direction calibration  ("calib")
//
//  For each leg in turn:
//    1. drive it alone at +CALIB_DUTY for CALIB_PULSE_MS, and read its encoder
//    2. drive it back the same amount, so the desk ends up level again
//    3. ask: "did it go UP first?" Answer y/n on serial, or press UP/DOWN.
//  If a leg doesn't move (e.g. the desk is at the bottom and the nudge
//  went into the end stop), it retries with the nudge the other way.
//  From the answer plus the encoder reading:
//    went down       -> flip that motor
//    encoder counted the wrong way for the real direction -> flip that encoder
//  Results go to flash and are loaded at every boot. 'calib reset'
//  returns to the Config.h defaults.
// ---------------------------------------------------------------------
#define CALIB_DUTY       0.35f
#define CALIB_PULSE_MS    400
#define CALIB_GAP_MS      250
#define CALIB_MIN_MM      0.5f     // less movement than this = "didn't move"

enum CalState : uint8_t { CAL_START, CAL_OUT, CAL_GAP, CAL_BACK, CAL_SETTLE, CAL_ASK };
CalState calState;
int      calLeg;
float    calSign = 1;          // direction of the first nudge (+1, or -1 on retry)
uint32_t calUntil;
int32_t  calStart, calDelta, calBackDelta;
bool     calNewMotorInv[NUM_LEGS], calNewEncInv[NUM_LEGS], calSkipped[NUM_LEGS];

uint8_t packBits(const bool* b) { uint8_t m = 0; for (int i = 0; i < NUM_LEGS; i++) if (b[i]) m |= 1 << i; return m; }
void    unpackBits(uint8_t m, bool* b) { for (int i = 0; i < NUM_LEGS; i++) b[i] = (m >> i) & 1; }

void applyDirections() {
  for (int i = 0; i < NUM_LEGS; i++) { motor[i]->setInvert(motorInv[i]); enc[i].setInvert(encInv[i]); }
}

void printDirections() {
  Out.print("  directions:");
  for (int i = 0; i < NUM_LEGS; i++)
    Out.printf("  leg %d motor %s enc %s", i, motorInv[i] ? "INV" : "norm", encInv[i] ? "INV" : "norm");
  Out.println(prefs.isKey("mInv") ? "   (calibrated, from flash)" : "   (Config.h defaults)");
}

void startCalib() {
  if (mode != M_IDLE) { Out.println("calib: stop / clear first"); return; }
  for (int i = 0; i < NUM_LEGS; i++) { calNewMotorInv[i] = motorInv[i]; calNewEncInv[i] = encInv[i]; calSkipped[i] = false; }
  calLeg = 0; calSign = 1; calState = CAL_START; mode = M_CALIB;
  Out.println("\n== Direction calibration ==");
  Out.println("Watch the desk. Each leg moves a few mm on its own, then returns.");
  Out.println("After each one, answer: did it go UP first?  y / n  (or press UP / DOWN). 'stop' aborts.");
}

void abortCalib(const char* why) {
  allMotorsOff();
  applyDirections();                       // back to the settings we started with
  mode = M_IDLE;
  Out.printf("calib aborted (%s); nothing changed\n", why);
}

void finishCalib() {
  allMotorsOff();
  bool encChanged = false, anyChange = false;
  for (int i = 0; i < NUM_LEGS; i++) {
    if (calNewEncInv[i] != encInv[i]) encChanged = true;
    if (calNewEncInv[i] != encInv[i] || calNewMotorInv[i] != motorInv[i]) anyChange = true;
    motorInv[i] = calNewMotorInv[i]; encInv[i] = calNewEncInv[i];
  }
  applyDirections();
  prefs.putUChar("mInv", packBits(motorInv));
  prefs.putUChar("eInv", packBits(encInv));
  if (encChanged) {
    // positions were counted with the old sign, so they're meaningless now
    for (int i = 0; i < NUM_LEGS; i++) { enc[i].set(0); leg[i].pos = leg[i].prevPos = leg[i].vel = 0; }
    homed = false;
    persistPositions(false);
  }
  mode = M_IDLE;
  Out.printf("== Calibration %s and saved ==\n", anyChange ? "changed settings" : "confirmed settings");
  printDirections();
  for (int i = 0; i < NUM_LEGS; i++)
    if (calSkipped[i]) Out.printf("  !! leg %d was skipped (no encoder movement), so check its wiring and run calib again\n", i);
  if (encChanged) Out.println("Encoder direction changed: position reset, run 'home'.");
  flashMessage("End");
}

// Called with the user's answer: true = it went UP first
void calibAnswer(bool wentUp) {
  if (mode != M_CALIB || calState != CAL_ASK) return;
  int i = calLeg;
  // Motor: a positive command must mean UP. The first nudge was calSign.
  if (wentUp != (calSign > 0)) calNewMotorInv[i] = !motorInv[i];
  // Encoder: counts must increase while the leg physically goes up.
  // The first pulse went up if wentUp, so the reading should match.
  bool countedUp = calDelta > 0;
  if (countedUp != wentUp) calNewEncInv[i] = !encInv[i];
  Out.printf("  leg %d: motor %s, encoder %s\n", i,
                calNewMotorInv[i] != motorInv[i] ? "FLIPPED" : "ok",
                calNewEncInv[i]   != encInv[i]   ? "FLIPPED" : "ok");
  calLeg++; calSign = 1;
  calState = CAL_START;
}

void calibStep(uint32_t now) {
  switch (calState) {
    case CAL_START:
      if (calLeg >= NUM_LEGS) { finishCalib(); return; }
      Out.printf("leg %d: moving...\n", calLeg);
      calStart = enc[calLeg].read();
      motor[calLeg]->set(calSign * CALIB_DUTY);
      calUntil = now + CALIB_PULSE_MS; calState = CAL_OUT;
      break;
    case CAL_OUT:
      if ((int32_t)(now - calUntil) < 0) break;
      motor[calLeg]->set(0);
      calUntil = now + CALIB_GAP_MS; calState = CAL_GAP;
      break;
    case CAL_GAP:                                    // let it coast to a stop, then read
      if ((int32_t)(now - calUntil) < 0) break;
      calDelta = enc[calLeg].read() - calStart;
      calStart = enc[calLeg].read();
      motor[calLeg]->set(-calSign * CALIB_DUTY);     // bring it back
      calUntil = now + CALIB_PULSE_MS; calState = CAL_BACK;
      break;
    case CAL_BACK:
      if ((int32_t)(now - calUntil) < 0) break;
      motor[calLeg]->set(0);
      calUntil = now + CALIB_GAP_MS; calState = CAL_SETTLE;
      break;
    case CAL_SETTLE:
      if ((int32_t)(now - calUntil) < 0) break;
      calBackDelta = enc[calLeg].read() - calStart;
      Out.printf("  encoder: %+ld counts out, %+ld back\n", (long)calDelta, (long)calBackDelta);
      if (labs(calDelta) < CALIB_MIN_MM * COUNTS_PER_MM) {
        if (calSign > 0) {                            // maybe it hit an end stop: try the other way
          Out.printf("  leg %d didn't move; trying the other direction (it may be at an end stop)\n", calLeg);
          calSign = -1; calState = CAL_START;
          break;
        }
        Out.printf("  !! leg %d: encoder barely moved either way. Motor not running, or encoder not wired? Skipping it.\n", calLeg);
        calSkipped[calLeg] = true; calLeg++; calSign = 1; calState = CAL_START;
        break;
      }
      if ((calDelta > 0) == (calBackDelta > 0))
        Out.println("  !! out and back counted the same direction. The encoder may be noisy or have A/B miswired.");
      Out.printf("  Did leg %d go UP first?  y / n  (or press UP / DOWN)\n", calLeg);
      calState = CAL_ASK;
      break;
    case CAL_ASK:
      break;                                          // waiting for calibAnswer()
  }
}

// ---------------------------------------------------------------------
//  Presets (shared by serial commands and the preset buttons). n is 0-based
//  here, shown as 1-4 to humans.
// ---------------------------------------------------------------------
void recallPreset(int n) {
  if (n < 0 || n >= NUM_PRESETS) { Out.println("Presets are 1-4"); return; }
  if (!homed) { Out.println("Not homed: run 'home' first"); flashMessage("---"); return; }
  if (isnan(presets[n])) {
    Out.printf("Preset %d is empty (hold its button 2 s, or 'save %d')\n", n + 1, n + 1);
    char m[4] = "P -"; flashMessage(m); return;
  }
  char m[4] = "P 0"; m[2] = '1' + n; flashMessage(m);
  Out.printf("Preset %d -> %.1f mm\n", n + 1, presets[n] + HEIGHT_OFFSET_MM);
  startMove(presets[n], MAX_SPEED_MM_S);
}

void savePreset(int n) {
  if (n < 0 || n >= NUM_PRESETS) { Out.println("Presets are 1-4"); return; }
  if (!homed) { Out.println("Not homed: can't save a preset yet"); flashMessage("---"); return; }
  if (mode != M_IDLE) { Out.println("Stop first"); return; }
  presets[n] = meanPos();
  char key[4] = "p0"; key[1] = '0' + n;
  prefs.putFloat(key, presets[n]);
  char m[4] = "S 0"; m[2] = '1' + n; flashMessage(m);
  Out.printf("Preset %d = %.1f mm\n", n + 1, presets[n] + HEIGHT_OFFSET_MM);
}

// ---------------------------------------------------------------------
//  Preset buttons on an R-2R ladder (one ADC pin)
//
//  Each button's level is either the ideal R-2R value or what 'btncal'
//  learned. A reading counts as button k when it's within 40% of the
//  smallest step of level k. Anything else (two buttons at once, or an
//  edge mid-press) is ignored. A press must read the same for 3 samples
//  (30 ms) to count.
// ---------------------------------------------------------------------
uint16_t ladderMv[NUM_PRESETS];        // expected mV per button (P1..P4)
uint16_t ladderRawMv = 0;              // last reading (for status / btncal)

int16_t  btnCalStep = -1;              // -1 = not calibrating, else button being learned
bool     btnCalWaitRelease = false;
uint32_t btnCalStableSince = 0;
uint32_t btnCalSum = 0, btnCalN = 0;
uint16_t btnCalRef = 0, btnCalNew[NUM_PRESETS];

void ladderDefaults() {
  for (int k = 0; k < NUM_PRESETS; k++) ladderMv[k] = PRESET_SUPPLY_MV >> (k + 1);  // 1/2, 1/4, 1/8, 1/16
}

// -1 = none, 0..3 = button, -2 = not a clean single-button level.
// Any two buttons together land at least one LSB step (the P4 level) away
// from every single-button level, so a tolerance under half of that
// rejects all chords.
int decodeLadder(uint16_t mv) {
  uint16_t lsb = ladderMv[NUM_PRESETS - 1];
  uint16_t tol = lsb * 40 / 100;
  if (mv < lsb / 2) return -1;
  for (int k = 0; k < NUM_PRESETS; k++)
    if (abs((int)mv - (int)ladderMv[k]) <= tol) return k;
  return -2;
}

void startButtonCal() {
  if (mode != M_IDLE) { Out.println("btncal: stop first"); return; }
  btnCalStep = 0; btnCalWaitRelease = false; btnCalStableSince = 0; btnCalN = 0;
  Out.println("\n== Preset button calibration ==  ('stop' aborts)");
  Out.println("Press and HOLD preset button 1 until told to release...");
  wakeDisplay();
}

void buttonCalSample(uint16_t mv, uint32_t now) {
  if (btnCalWaitRelease) {
    if (mv < 60) {                                   // released
      btnCalWaitRelease = false; btnCalStableSince = 0; btnCalN = 0;
      if (++btnCalStep >= NUM_PRESETS) {
        // sanity: levels must be clearly descending P1 > P2 > P3 > P4 > 0
        bool ok = true;
        for (int k = 0; k < NUM_PRESETS; k++)
          if (btnCalNew[k] < 80 || (k && btnCalNew[k] + 60 > btnCalNew[k - 1])) ok = false;
        if (!ok) {
          Out.println("!! Levels aren't clearly separated (check the ladder wiring and the button order). Not saved.");
        } else {
          for (int k = 0; k < NUM_PRESETS; k++) {
            ladderMv[k] = btnCalNew[k];
            char key[4] = "b0"; key[1] = '0' + k; prefs.putUShort(key, ladderMv[k]);
          }
          Out.printf("== Saved: P1 %u  P2 %u  P3 %u  P4 %u mV ==\n", ladderMv[0], ladderMv[1], ladderMv[2], ladderMv[3]);
          flashMessage("End");
        }
        btnCalStep = -1;
      } else {
        Out.printf("Press and HOLD preset button %d...\n", btnCalStep + 1);
      }
    }
    return;
  }
  if (mv < 60) { btnCalStableSince = 0; btnCalN = 0; return; }      // nothing pressed yet
  if (!btnCalStableSince || abs((int)mv - (int)btnCalRef) > 40) {   // (re)start a steady window
    btnCalRef = mv; btnCalStableSince = now; btnCalSum = 0; btnCalN = 0;
  }
  btnCalSum += mv; btnCalN++;
  if (now - btnCalStableSince >= 500 && btnCalN >= 20) {
    btnCalNew[btnCalStep] = btnCalSum / btnCalN;
    Out.printf("  button %d = %u mV. Release it.\n", btnCalStep + 1, btnCalNew[btnCalStep]);
    btnCalWaitRelease = true;
  }
}

void pollPresetButtons() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 10) return;                       // 100 Hz
  last = now;

  uint16_t mv = (analogReadMilliVolts(PIN_PRESET_ADC) + analogReadMilliVolts(PIN_PRESET_ADC)) / 2;
  ladderRawMv = mv;
  DeskLock lock;
  if (btnCalStep >= 0) { if (mv > 60) wakeDisplay(); buttonCalSample(mv, now); return; }

  // debounce: same decode 3 samples in a row
  static int cand = -1, candCount = 0, stable = -1;
  static uint32_t pressStart = 0;
  static bool savedThisPress = false, swallow = false;
  int d = decodeLadder(mv);
  if (d == -2) return;                               // between levels: ignore, keep state
  if (d == cand) { if (candCount < 3) candCount++; } else { cand = d; candCount = 1; }
  if (candCount < 3 || cand == stable) {
    // held: long-press save
    if (stable >= 0 && !swallow && !savedThisPress && now - pressStart >= PRESET_LONG_PRESS_MS) {
      savedThisPress = true;
      savePreset(stable);
    }
    return;
  }

  int prev = stable;
  stable = cand;
  if (stable >= 0) {                                 // ---- pressed
    wakeDisplay();
    pressStart = now; savedThisPress = false; swallow = false;
    if (mode == M_FAULT || mode == M_CALIB || mode == M_TEST) { swallow = true; return; }
    if (mode == M_MOVING || mode == M_HOMING) {      // any press stops a move
      softStop(); swallow = true;
    }
  } else if (prev >= 0) {                            // ---- released
    if (!swallow && !savedThisPress && mode == M_IDLE) recallPreset(prev);
  }
}

// ---------------------------------------------------------------------
//  Control loop — called at CONTROL_HZ
// ---------------------------------------------------------------------
void readEncoders(float dt) {
  for (int i = 0; i < NUM_LEGS; i++) {
    Leg& L = leg[i];
    L.pos = enc[i].read() / COUNTS_PER_MM;
    float v = (L.pos - L.prevPos) / dt;
    L.vel += 0.15f * (v - L.vel);                     // light low-pass
    L.prevPos = L.pos;
  }
}

void controlStep(float dt) {
  uint32_t now = millis();
  readEncoders(dt);

  // ---- open-loop single-leg test -----------------------------------
  if (mode == M_CALIB) {
    calibStep(now);
    if (!pca.ok()) { fault(FAULT_DRIVER, "I2C write to motor driver failed during calib"); }
    return;
  }

  if (mode == M_TEST) {
    if ((int32_t)(now - testUntil) >= 0) {
      motor[testLeg]->set(0);
      int32_t d = enc[testLeg].read() - testStartCount;
      Out.printf("test leg %d: moved %ld counts (%.2f mm). Expect +counts for a +duty (UP).\n",
                    testLeg, (long)d, d / COUNTS_PER_MM);
      mode = testPrevMode;
    }
    return;
  }

  if (mode != M_MOVING && mode != M_HOMING) return;

  // ---- 1. trapezoidal setpoint profile ------------------------------
  float d     = goal - sp;
  float vStop = sqrtf(2.0f * ACCEL_MM_S2 * fabsf(d));  // fastest speed that can still stop at the goal
  float vT    = sgn(d) * fminf(vmax, vStop);
  float dv    = ACCEL_MM_S2 * dt;
  spVel += constrain(vT - spVel, -dv, dv);
  float spNew = sp + spVel * dt;
  if ((goal - sp) * (goal - spNew) <= 0.0f && fabsf(d) < 1.0f) { spNew = goal; spVel = 0; }

  // ---- 2. lead clamp: setpoint waits for the slowest leg ------------
  float minPos = 1e9f, maxPos = -1e9f, minVel = 0, maxVel = 0;
  int   nActive = 0;
  for (int i = 0; i < NUM_LEGS; i++) {
    if (!leg[i].active) continue;
    nActive++;
    if (leg[i].pos < minPos) { minPos = leg[i].pos; minVel = leg[i].vel; }
    if (leg[i].pos > maxPos) { maxPos = leg[i].pos; maxVel = leg[i].vel; }
  }
  if (nActive > 0) {
    if (spVel > 0) {
      float lim = fmaxf(sp, minPos + MAX_LEAD_MM);
      if (spNew > lim) { spNew = lim; spVel = fminf(spVel, fmaxf(minVel, 0.0f)); }
    } else if (spVel < 0) {
      float lim = fminf(sp, maxPos - MAX_LEAD_MM);
      if (spNew < lim) { spNew = lim; spVel = fmaxf(spVel, fminf(maxVel, 0.0f)); }
    }
  }
  sp = spNew;

  // ---- 3. per-leg position loops ------------------------------------
  float ff   = kff * spVel / MAX_SPEED_MM_S;
  float slew = OUTPUT_SLEW_PER_S * dt;
  for (int i = 0; i < NUM_LEGS; i++) {
    Leg& L = leg[i];
    if (!L.active) { L.out = 0; motor[i]->set(0); continue; }

    float err = sp - L.pos;
    if (ki > 0) {
      L.integ += err * dt;
      float lim = I_LIMIT / ki;
      L.integ = constrain(L.integ, -lim, lim);
    }
    float u = ff + kp * err + ki * L.integ;

    // dead-band compensation: tiny commands get bumped past motor stiction
    if (fabsf(u) > 0.02f) u = sgn(u) * (MIN_DUTY + (1.0f - MIN_DUTY) * fminf(fabsf(u), 1.0f));
    else                  u = 0;

    L.out += constrain(u - L.out, -slew, slew);       // soft output slew
    motor[i]->set(L.out);
  }

  // ---- 4. safety checks ---------------------------------------------
  if (!pca.ok()) {
    fault(FAULT_DRIVER, String("I2C write to motor driver failed (") + (int)pca.errors() + " errors)");
    return;
  }
  if (nActive > 1 && (maxPos - minPos) > SYNC_FAULT_MM) {
    fault(FAULT_SYNC, String("legs out of sync by ") + String(maxPos - minPos, 1) + " mm");
    return;
  }
  for (int i = 0; i < NUM_LEGS; i++) {
    Leg& L = leg[i];
    if (!L.active) continue;

    if (fabsf(L.vel) > MAX_PLAUSIBLE_MM_S) {
      fault(FAULT_ENCODER, String("leg ") + i + " encoder implausible (" + String(L.vel, 0) + " mm/s), noise or wiring?");
      return;
    }

    // stall detection
    if (fabsf(L.out) >= STALL_DUTY) {
      if (fabsf(L.pos - L.stallRef) > STALL_MIN_MM) { L.stallRef = L.pos; L.stallSince = now; }
      else if (now - L.stallSince > STALL_TIME_MS) {
        if (mode == M_HOMING && L.out < 0) {
          // this leg has reached its bottom stop
          motor[i]->set(0);
          L.out = 0; L.active = false;
          Out.printf("  leg %d at bottom\n", i);
        } else {
          fault(FAULT_STALL, String("leg ") + i + " stalled (obstruction?)");
          return;
        }
      }
    } else {
      L.stallRef = L.pos; L.stallSince = now;
    }
  }

  // ---- 5. homing finished? ------------------------------------------
  if (mode == M_HOMING) {
    bool any = false;
    for (int i = 0; i < NUM_LEGS; i++) any |= leg[i].active;
    if (!any) {
      for (int i = 0; i < NUM_LEGS; i++) {
        enc[i].set(0);
        leg[i].pos = leg[i].prevPos = leg[i].vel = 0;
      }
      homed = true;
      mode = M_IDLE;
      Out.println("Homed. Backing off the end stops.");
      startMove(HOMING_BACKOFF_MM, HOMING_SPEED_MM_S);
    }
    return;
  }

  // ---- 6. arrival ---------------------------------------------------
  if (sp == goal && spVel == 0) {
    if (!arrivedSince) arrivedSince = now ? now : 1;
    bool settled = true;
    for (int i = 0; i < NUM_LEGS; i++) settled &= fabsf(goal - leg[i].pos) < POSITION_TOL_MM;
    if (settled || now - arrivedSince > SETTLE_TIMEOUT_MS) finishMove();
  }
}

// ---------------------------------------------------------------------
//  Status / logging
// ---------------------------------------------------------------------
void printStatus() {
  Out.printf("mode=%s homed=%s encoder=%s sp=%.2f goal=%.2f v=%.1f\n",
                MODE_NAMES[mode], homed ? "yes" : "NO", enc[0].backend(), sp, goal, spVel);
  for (int i = 0; i < NUM_LEGS; i++)
    Out.printf("  leg %d: %8.2f mm  %6.1f mm/s  out %+.2f  (%ld counts)%s\n",
                  i, leg[i].pos, leg[i].vel, leg[i].out, (long)enc[i].read(),
                  leg[i].active ? "" : "  [inactive]");
  Out.printf("  desk height %.1f mm   kp=%.3f ki=%.3f kff=%.3f\n",
                meanPos() + HEIGHT_OFFSET_MM, kp, ki, kff);
  Out.printf("  driver: PCA9685 @0x%02X %s, PWM %.0f Hz, I2C errors %lu\n", pca.address(),
                pca.ok() ? "ok" : "ERROR", pca.actualHz(), (unsigned long)pca.errors());
  printDirections();
  Out.printf("  preset ladder: now %u mV, levels P1 %u P2 %u P3 %u P4 %u mV%s\n", ladderRawMv,
                ladderMv[0], ladderMv[1], ladderMv[2], ladderMv[3], prefs.isKey("b0") ? " (learned)" : " (ideal; run btncal)");
  Out.print("  presets:");
  for (int n = 0; n < NUM_PRESETS; n++) {
    if (isnan(presets[n])) Out.printf("  P%d empty", n + 1);
    else                   Out.printf("  P%d %.0f mm", n + 1, presets[n] + HEIGHT_OFFSET_MM);
  }
  Out.println();
  if (mode == M_FAULT) Out.printf("  fault: E%d %s\n", faultCode, faultMsg.c_str());
}

void logLine() {
  // CSV for the Arduino Serial Plotter or a spreadsheet
  Serial.printf("%lu,%.2f", millis(), sp);
  for (int i = 0; i < NUM_LEGS; i++) Serial.printf(",%.2f", leg[i].pos);
  for (int i = 0; i < NUM_LEGS; i++) Serial.printf(",%.2f", leg[i].out);
  Serial.println();
}

// ---------------------------------------------------------------------
//  Serial commands
// ---------------------------------------------------------------------
void handleCommand(String line) {
  DeskLock lock;
  line.trim();
  if (!line.length()) return;
  int sp1 = line.indexOf(' ');
  String cmd = sp1 < 0 ? line : line.substring(0, sp1);
  String arg = sp1 < 0 ? "" : line.substring(sp1 + 1);
  cmd.toLowerCase();

  if (btnCalStep >= 0) {
    if (cmd == "stop" || cmd == "s" || cmd == "abort") { btnCalStep = -1; Out.println("btncal aborted; nothing changed"); }
    else Out.println("btncal: press the buttons as prompted, or 'stop'");
    return;
  }
  if (mode == M_CALIB) {                     // during calibration only these apply
    if      (cmd == "y" || cmd == "yes" || cmd == "up")   calibAnswer(true);
    else if (cmd == "n" || cmd == "no"  || cmd == "down") calibAnswer(false);
    else if (cmd == "stop" || cmd == "s" || cmd == "abort") abortCalib("stopped");
    else Out.println("calibrating: answer y / n, or 'stop' to abort");
    return;
  }

  if      (cmd == "home")   startHoming();
  else if (cmd == "up")     jog(+1);
  else if (cmd == "down")   jog(-1);
  else if (cmd == "stop" || cmd == "s") softStop();
  else if (cmd == "goto")   {
    if (!homed) Out.println("Not homed: run 'home' first");
    else startMove(arg.toFloat() - HEIGHT_OFFSET_MM, MAX_SPEED_MM_S);
  }
  else if (cmd == "level")  startMove(meanPos(), MAX_SPEED_MM_S / 2);
  else if (cmd == "p")      recallPreset(arg.toInt() - 1);
  else if (cmd == "save")   savePreset(arg.toInt() - 1);
  else if (cmd == "btncal") startButtonCal();
  else if (cmd == "setp")   {
    // setp <1-4> <desk height mm>: set a preset without moving there
    int sp2 = arg.indexOf(' ');
    int n = arg.substring(0, sp2).toInt() - 1;
    float mm = sp2 < 0 ? NAN : arg.substring(sp2 + 1).toFloat();
    if (n < 0 || n >= NUM_PRESETS || isnan(mm)) { Out.println("usage: setp <1-4> <desk height mm>"); return; }
    float rel = constrain(mm - HEIGHT_OFFSET_MM, MIN_HEIGHT_MM, MAX_HEIGHT_MM);
    presets[n] = rel;
    char key[4] = "p0"; key[1] = '0' + n;
    prefs.putFloat(key, rel);
    Out.printf("Preset %d = %.1f mm\n", n + 1, rel + HEIGHT_OFFSET_MM);
  }
  else if (cmd == "reboot") {
    if (mode != M_IDLE && mode != M_FAULT) { Out.println("Stop first"); return; }
    Out.println("Rebooting");
    allMotorsOff();
    delay(100);
    ESP.restart();
  }
  else if (cmd == "status") printStatus();
  else if (cmd == "log")    { logging = !logging; Out.println("t,sp,p0,p1,p2,p3,o0,o1,o2,o3"); }
  else if (cmd == "clear")  { if (mode == M_FAULT) clearFault(); }
  else if (cmd == "zero")   {
    if (mode != M_IDLE) { Out.println("Stop first"); return; }
    for (int i = 0; i < NUM_LEGS; i++) enc[i].set(0);
    homed = true; persistPositions(false);
    Out.println("Current position set as bottom (0 mm)");
  }
  else if (cmd == "calib")  {
    if (arg == "reset") {
      prefs.remove("mInv"); prefs.remove("eInv");
      for (int i = 0; i < NUM_LEGS; i++) { motorInv[i] = MOTOR_INVERT[i]; encInv[i] = ENCODER_INVERT[i]; }
      applyDirections();
      for (int i = 0; i < NUM_LEGS; i++) enc[i].set(0);
      homed = false; persistPositions(false);
      Out.println("Directions reset to Config.h defaults; position reset, run 'home'");
      printDirections();
    } else startCalib();
  }
  else if (cmd == "test")   {
    // test <leg> <duty>: one leg alone, 400 ms. Keep the duty small (0.3);
    // this tilts the desk slightly.
    int s2 = arg.indexOf(' ');
    int   n = arg.substring(0, s2).toInt();
    float duty = s2 < 0 ? 0.3f : arg.substring(s2 + 1).toFloat();
    if (mode != M_IDLE && mode != M_FAULT) { Out.println("Stop first"); return; }
    if (n < 0 || n >= NUM_LEGS) return;
    duty = constrain(duty, -0.6f, 0.6f);
    testLeg = n; testStartCount = enc[n].read(); testUntil = millis() + 400;
    testPrevMode = mode;
    mode = M_TEST;
    motor[n]->set(duty);
  }
  else if (cmd == "kp")  { kp  = arg.toFloat(); Out.printf("kp=%.3f\n", kp); }
  else if (cmd == "ki")  { ki  = arg.toFloat(); Out.printf("ki=%.3f\n", ki); }
  else if (cmd == "kff") { kff = arg.toFloat(); Out.printf("kff=%.3f\n", kff); }
  else Out.println("cmds: home up down stop goto <mm> level p <n> save <n> status log clear zero calib [reset] btncal setp <n> <mm> reboot test <leg> <duty> kp/ki/kff <v>");
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    wakeDisplay();
    if (c == '\n' || c == '\r') { handleCommand(buf); buf = ""; }
    else if (buf.length() < 64) buf += c;
  }
}

// ---------------------------------------------------------------------
//  Buttons
// ---------------------------------------------------------------------
void pollButtons() {
  DeskLock lock;
  static uint32_t lastChange = 0;
  static bool upS = false, dnS = false;
  bool up = digitalRead(PIN_BTN_UP)   == LOW;
  bool dn = digitalRead(PIN_BTN_DOWN) == LOW;
  uint32_t now = millis();
  if ((up != upS || dn != dnS) && now - lastChange < 30) return;   // debounce
  if (up != upS || dn != dnS) lastChange = now;
  bool upEdge = up && !upS, dnEdge = dn && !dnS;
  upS = up; dnS = dn;
  if (up || dn) wakeDisplay();              // pressing or holding counts as activity

  // both held 3 s -> clear fault and re-home
  if (up && dn) {
    if (!bothHeldSince) { bothHeldSince = now; if (mode == M_CALIB) abortCalib("both buttons"); else softStop(); }
    else if (now - bothHeldSince > 3000) {
      bothHeldSince = 0; swallowPress = true;
      if (mode == M_FAULT) clearFault();
      if (mode == M_IDLE) startHoming();
    }
    return;
  }
  bothHeldSince = 0;

  if (!up && !dn) {                         // released
    swallowPress = false;
    if (jogging) softStop();
    return;
  }
  if (swallowPress || mode == M_FAULT) return;

  if (mode == M_CALIB) {                    // UP = "it went up", DOWN = "it went down"
    if (upEdge) calibAnswer(true);
    if (dnEdge) calibAnswer(false);
    swallowPress = true;
    return;
  }

  // a press during an automatic move (preset/goto/homing) is a stop
  if ((upEdge || dnEdge) && !jogging && (mode == M_MOVING || mode == M_HOMING)) {
    softStop(); swallowPress = true; return;
  }
  if (upEdge) jog(+1, true);
  if (dnEdge) jog(-1, true);
}

// ---------------------------------------------------------------------
//  Display — called at ~20 Hz from loop()
//
//  What it shows, in priority order:
//    Er1..Er4      fault (blinking)
//    P 1 / S 1     preset recalled / saved (brief)
//    H--           homing in progress
//    ---           position unknown, needs homing
//    75            desk height (cm, or inches if DISPLAY_INCHES)
//  Goes dark DISPLAY_TIMEOUT_MS after the last button or serial input.
//  Motion counts as activity, so it stays lit while the desk moves.
// ---------------------------------------------------------------------
void updateDisplay() {
  DeskLock lock;
  uint32_t now = millis();
  if (mode == M_MOVING || mode == M_HOMING || mode == M_TEST || mode == M_CALIB || btnCalStep >= 0) lastActivity = now;
#if DISPLAY_FAULT_STAYS_ON
  if (mode == M_FAULT) lastActivity = now;
#endif

  if (now - lastActivity > DISPLAY_TIMEOUT_MS) {
    if (display.isOn()) display.off();
    return;
  }

  char buf[4];
  if (mode == M_FAULT) {
    // blink at 2 Hz so a fault is obvious from across the room
    if ((now / 250) & 1) snprintf(buf, sizeof buf, "Er%c", '0' + (faultCode % 10));
    else                 strcpy(buf, "   ");
  }
  else if ((int32_t)(msgUntil - now) > 0) {
    strcpy(buf, msgText);
  }
  else if (btnCalStep >= 0) {
    strcpy(buf, "b 1"); buf[2] = '1' + btnCalStep;   // which button to press
  }
  else if (mode == M_CALIB) {
    // "C 2" while leg 2 moves; blinks while waiting for the up/down answer
    if (calState == CAL_ASK && !((now / 300) & 1)) strcpy(buf, "   ");
    else { strcpy(buf, "C 0"); buf[2] = '0' + (calLeg < NUM_LEGS ? calLeg : NUM_LEGS - 1); }
  }
  else if (mode == M_HOMING) {
    strcpy(buf, "H--");
  }
  else if (!homed) {
    strcpy(buf, "---");
  }
  else {
#if DISPLAY_INCHES
    float h = (meanPos() + HEIGHT_OFFSET_MM) / 25.4f;
#else
    float h = (meanPos() + HEIGHT_OFFSET_MM) / 10.0f;
#endif
    // hysteresis: only change the shown number once the height is 0.1 past
    // the rounding boundary, so it doesn't flicker when sitting on one
    if (fabsf(h - shownHeight) > 0.6f) shownHeight = roundf(h);
    int v = constrain((int)shownHeight, 0, 999);
    snprintf(buf, sizeof buf, "%d", v);
  }
  display.print(buf);
  display.on();
}

// ---------------------------------------------------------------------
void setup() {
#if CONTROL_TASK
  deskMutex = xSemaphoreCreateRecursiveMutex();
#endif
  // ---- motors off FIRST ---------------------------------------------
  // If we got here via a crash, watchdog or firmware upload while the desk
  // was moving, the PCA9685 is still driving the motors. Stop them before
  // anything else (before even the serial port).
#if PIN_MOTOR_POWER != PIN_NONE
  pinMode(PIN_MOTOR_POWER, OUTPUT);
  setMotorPower(true);
#endif
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(20);                       // ms; don't hang on a stuck bus
  for (int i = 0; i < NUM_LEGS; i++) motor[i] = new BtsMotor(pca, LEG_PORT[i]);
  for (int i = 0; i < NUM_LEGS; i++) motor[i]->setInvert(MOTOR_INVERT[i]);   // until flash is read
  bool driverOk = initMotorDriver();

  Serial.begin(115200);
  delay(200);
  Out.println("\n4-leg desk controller");

  if (driverOk) {
    Out.printf("Motor driver: PCA9685 at 0x%02X, PWM %.0f Hz, legs on ports %d %d %d %d\n",
                  BTS_BOARD_ADDR, pca.actualHz(), LEG_PORT[0], LEG_PORT[1], LEG_PORT[2], LEG_PORT[3]);
  } else {
    Out.printf("Motor driver NOT found at 0x%02X. I2C scan:", BTS_BOARD_ADDR);
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) Out.printf(" 0x%02X", a);
    }
    Out.println("\n  (PCA9685 answers at 0x40-0x7F per its JP1-JP6 jumpers; 0x70 is its all-call address)");
  }

  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

  display.begin(DISPLAY_BRIGHTNESS);
  if (!display.present()) Out.println("Display not responding (check CLK/DIO wiring and 3.3V power)");
  wakeDisplay();                            // show the height for a few seconds after boot

  for (int i = 0; i < NUM_LEGS; i++) {
    if (!enc[i].begin(LEG_PINS[i].encA, LEG_PINS[i].encB, ENCODER_INVERT[i]))
      Out.printf("encoder %d init FAILED\n", i);
  }
  Out.printf("Encoder backend: %s\n", enc[0].backend());

  prefs.begin("desk", false);
  // Direction settings: saved calibration if there is one, else Config.h
  uint8_t defM = 0, defE = 0;
  for (int i = 0; i < NUM_LEGS; i++) { defM |= MOTOR_INVERT[i] << i; defE |= ENCODER_INVERT[i] << i; }
  unpackBits(prefs.getUChar("mInv", defM), motorInv);
  unpackBits(prefs.getUChar("eInv", defE), encInv);
  applyDirections();
  for (int n = 0; n < NUM_PRESETS; n++) {
    char key[4] = "p0"; key[1] = '0' + n;
    presets[n] = prefs.getFloat(key, NAN);
  }
  ladderDefaults();
  for (int k = 0; k < NUM_PRESETS; k++) {
    char key[4] = "b0"; key[1] = '0' + k;
    ladderMv[k] = prefs.getUShort(key, ladderMv[k]);
  }
#if PERSIST_POSITION
  if (prefs.getBool("homed", false) && !prefs.getBool("moving", true)) {
    char key[4] = "c0";
    for (int i = 0; i < NUM_LEGS; i++) { key[1] = '0' + i; enc[i].set(prefs.getInt(key, 0)); }
    homed = true;
    Out.println("Restored position from flash");
  }
#endif
  readEncoders(1.0f);
  for (int i = 0; i < NUM_LEGS; i++) leg[i].vel = 0;
  if (!homed) Out.println("Position unknown: send 'home' (or hold both buttons 3 s)");
  if (!driverOk) fault(FAULT_DRIVER, "motor driver not found on I2C");
  printStatus();

  // Watchdog on loop(): if the firmware ever hangs, the ESP32 resets and
  // setup() turns the motors off (default timeout 5 s).
  enableLoopWDT();

#if CONTROL_TASK
  xTaskCreatePinnedToCore(controlTask, "desk-ctl", 8192, nullptr, 5, nullptr, 1);
#endif
#if ENABLE_HOME_ASSISTANT
  haBegin();
#endif
}

// ---------------------------------------------------------------------
//  Control task: CONTROL_HZ, pinned to core 1 at a higher priority than
//  loop(). Wi-Fi and MQTT live on core 0 and in loop(), so nothing
//  network-side can ever stall a control tick. It has its own watchdog
//  entry: if a tick ever hangs, the ESP32 resets and setup() turns the
//  motors off.
// ---------------------------------------------------------------------
void controlTick(float dt) {
  static uint32_t logTick = 0;
  DeskLock lock;
  controlStep(dt);
  if (logging && (mode == M_MOVING || mode == M_HOMING) && ++logTick % (CONTROL_HZ / 20) == 0) logLine();
}

#if CONTROL_TASK
void controlTask(void*) {
  esp_task_wdt_add(NULL);
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastUs = micros();
  for (;;) {
    vTaskDelayUntil(&wake, period);
    uint32_t now = micros();
    controlTick((now - lastUs) * 1e-6f);
    lastUs = now;
    esp_task_wdt_reset();
  }
}
#endif

void loop() {
#if !CONTROL_TASK
  static uint32_t last = micros();
  const  uint32_t period = 1000000UL / CONTROL_HZ;
  uint32_t now = micros();
  if (now - last >= period) {
    float dt = (now - last) * 1e-6f;
    last = now;
    controlTick(dt);
  }
#endif
  pollSerial();
  pollButtons();
  pollPresetButtons();

  // display refresh at 20 Hz. It only talks to the chip when something changes.
  static uint32_t lastDisp = 0;
  uint32_t ms = millis();
  if (ms - lastDisp >= 50) { lastDisp = ms; updateDisplay(); }

#if ENABLE_HOME_ASSISTANT
  haLoop();
#endif
}

#if ENABLE_HOME_ASSISTANT
#include "HomeAssistant.h"
#endif
