/*
 * XY cell culture shaker — X + Y, sinusoidal, ALTERNATING
 * ESP32 + two TMC2209 (standalone STEP/DIR) + two NEMA 17
 *
 * The two axes never move at the same time. One axis runs its sinusoid,
 * ramps back to center, fully settles, THEN the other axis takes over:
 *   soft-start -> N full cycles -> soft-stop -> settle -> hand off.
 *
 * Core is the same drift-free two-layer design as the single-axis sketch:
 *   ISR (40 kHz)     — services both axes, each chasing its own step target.
 *                      Only the active axis has a moving target, so only one
 *                      motor ever steps.
 *   Profile (1 kHz)  — supervisor state machine + phase accumulator.
 *
 * Pins:  X_STEP=32  X_DIR=33   Y_STEP=18  Y_DIR=19
 *        X_EN=25    Y_EN=4     (TMC2209 EN active LOW — see note below)
 *
 * Serial (115200):
 *   A 40   amplitude, mm (both axes)   F 0.5  frequency, Hz
 *   N 3    cycles per burst            X pause   R resume   ? status
 *
 * Requires ESP32 Arduino core 3.x.
 */

#include <math.h>

// ---------------- mechanics ----------------
constexpr float STEPS_PER_MM = 1600.0f / 40.0f;   // 40 steps/mm
constexpr float MAX_AMP_MM   = 100.0f;
constexpr float AMP_SLEW     = 12.0f;             // mm/s ramp

constexpr uint32_t ISR_HZ     = 40000;
constexpr uint32_t PROFILE_HZ = 1000;
constexpr float TWO_PI_F      = 2.0f * (float)M_PI;

// EN pins (active LOW). If your drivers' EN are already tied to GND in
// hardware, these are harmless. If not, wire each driver's EN here.
constexpr int X_EN = 25;
constexpr int Y_EN = 4;

// ---------------- axes ----------------
struct Axis {
  const int stepPin;
  const int dirPin;
  volatile int32_t target;    // written by profile loop
  volatile int32_t current;   // owned by ISR
  float ampNow;               // slew-limited amplitude, mm
  float phase;                // accumulated, rad
};

Axis axisX = { 32, 33, 0, 0, 0.0f, 0.0f };
Axis axisY = { 18, 19, 0, 0, 0.0f, 0.0f };

// ---------------- commanded motion ----------------
float ampCmd  = 50.8f;   // mm  (2 in)
float freqCmd = 1.5;   // Hz
int   cyclesPerBurst = 3;
bool  running = true;

hw_timer_t *stepTimer = nullptr;

// ---------------- supervisor ----------------
enum Sub { RAMP_UP, RUN, RAMP_DOWN, SETTLE };
Axis* act  = &axisX;     // the axis currently allowed to move
Axis* idle = &axisY;
Sub   sub  = RAMP_UP;
int   cyclesLeft = 0;

// ---------------- step generator (both axes) ----------------
void IRAM_ATTR onStepISR() {
  int32_t ex = axisX.target - axisX.current;
  if (ex != 0) {
    digitalWrite(axisX.dirPin, ex > 0 ? HIGH : LOW);
    digitalWrite(axisX.stepPin, HIGH);
    axisX.current += (ex > 0) ? 1 : -1;
    digitalWrite(axisX.stepPin, LOW);
  }
  int32_t ey = axisY.target - axisY.current;
  if (ey != 0) {
    digitalWrite(axisY.dirPin, ey > 0 ? HIGH : LOW);
    digitalWrite(axisY.stepPin, HIGH);
    axisY.current += (ey > 0) ? 1 : -1;
    digitalWrite(axisY.stepPin, LOW);
  }
}

// ---------------- helpers ----------------
static const char* axisName(Axis* a) { return (a == &axisX) ? "X" : "Y"; }

static void status() {
  Serial.printf("active=%s  sub=%d  A=%.1f mm  F=%.3f Hz  N=%d  %s\n",
                axisName(act), (int)sub, ampCmd, freqCmd, cyclesPerBurst,
                running ? "running" : "PAUSED");
}

static void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '?')             { status(); return; }
  if (c == 'X' || c == 'x') { running = false; Serial.println("pausing");  return; }
  if (c == 'R' || c == 'r') { running = true;  Serial.println("resuming"); return; }

  float v = Serial.parseFloat();
  switch (c) {
    case 'A': case 'a':
      if (v >= 0 && v <= MAX_AMP_MM) ampCmd = v; else Serial.println("amp out of range");
      break;
    case 'F': case 'f':
      if (v > 0) freqCmd = v; else Serial.println("freq must be > 0");
      break;
    case 'N': case 'n':
      if (v >= 1) cyclesPerBurst = (int)v;
      break;
    default: return;
  }
  status();
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);

  pinMode(axisX.stepPin, OUTPUT); pinMode(axisX.dirPin, OUTPUT);
  pinMode(axisY.stepPin, OUTPUT); pinMode(axisY.dirPin, OUTPUT);
  pinMode(X_EN, OUTPUT); digitalWrite(X_EN, LOW);   // enable X driver
  pinMode(Y_EN, OUTPUT); digitalWrite(Y_EN, LOW);   // enable Y driver

  // both carriages assumed centered at power-on
  axisX.current = axisY.current = 0;
  axisX.target  = axisY.target  = 0;

  stepTimer = timerBegin(1000000);                  // 1 MHz tick
  timerAttachInterrupt(stepTimer, &onStepISR);
  timerAlarm(stepTimer, 1000000 / ISR_HZ, true, 0);

  // --- core 2.x equivalent ---
  // stepTimer = timerBegin(0, 80, true);
  // timerAttachInterrupt(stepTimer, &onStepISR, true);
  // timerAlarmWrite(stepTimer, 1000000 / ISR_HZ, true);
  // timerAlarmEnable(stepTimer);

  Serial.println("XY alternating shaker ready");
  status();
}

void loop() {
  handleSerial();

  static uint32_t last = 0;
  uint32_t now = micros();
  if ((uint32_t)(now - last) < 1000000UL / PROFILE_HZ) return;
  float dt = (now - last) * 1e-6f;
  last = now;
  if (dt > 0.05f) dt = 0.05f;

  // idle axis is always parked at center — it never moves
  idle->target = 0;

  // if paused, bring the active axis down and hold
  if (!running && (sub == RAMP_UP || sub == RUN)) sub = RAMP_DOWN;

  // advance active-axis phase, detect a full-cycle wrap
  act->phase += TWO_PI_F * freqCmd * dt;
  bool wrapped = false;
  if (act->phase >= TWO_PI_F) { act->phase -= TWO_PI_F; wrapped = true; }

  switch (sub) {
    case RAMP_UP:
      act->ampNow = fminf(ampCmd, act->ampNow + AMP_SLEW * dt);
      if (act->ampNow >= ampCmd) { sub = RUN; cyclesLeft = cyclesPerBurst; }
      break;
    case RUN:                                  // hold at amplitude (slew if changed live)
      if      (act->ampNow < ampCmd) act->ampNow = fminf(ampCmd, act->ampNow + AMP_SLEW * dt);
      else if (act->ampNow > ampCmd) act->ampNow = fmaxf(ampCmd, act->ampNow - AMP_SLEW * dt);
      if (wrapped && --cyclesLeft <= 0) sub = RAMP_DOWN;
      break;
    case RAMP_DOWN:
      act->ampNow = fmaxf(0.0f, act->ampNow - AMP_SLEW * dt);
      if (act->ampNow <= 0.0f) { act->ampNow = 0.0f; sub = SETTLE; }
      break;
    case SETTLE:
      act->ampNow = 0.0f;
      break;
  }

  act->target = (int32_t)lroundf(act->ampNow * sinf(act->phase) * STEPS_PER_MM);

  // hand off only once the active axis has fully stopped at center
  if (running && sub == SETTLE && act->current == 0) {
    Serial.printf("hand off %s -> %s\n", axisName(act), axisName(idle));
    Axis* t = act; act = idle; idle = t;      // swap active/idle
    act->phase  = 0.0f;
    act->ampNow = 0.0f;
    sub = RAMP_UP;
  }
}
