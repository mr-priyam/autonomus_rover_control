#include <Servo.h>

/* ================== STEPPER PINS ================== */
#define FL_DIR 29
#define FL_PUL 28
#define FR_DIR 27
#define FR_PUL 26
#define RL_DIR 25
#define RL_PUL 24
#define RR_DIR 23
#define RR_PUL 22

/* ================== SERVO PINS ================== */
#define FL_SERVO 5
#define FR_SERVO 7
#define RL_SERVO 9
#define RR_SERVO 11

Servo FL_S, FR_S, RL_S, RR_S;

/* ================== TIMING ================== */
const unsigned long STEP_PULSE_US = 10;     // 10 µs
const unsigned long RUN_TIME_MS   = 3000;
const unsigned long PAUSE_MS      = 1000;
const unsigned long SERVO_STEP_MS = 20;     // servo speed

/* ================== SERVO STATE ================== */
int curFL = 90, curFR = 90, curRL = 90, curRR = 90;
int tgtFL = 90, tgtFR = 90, tgtRL = 90, tgtRR = 90;
unsigned long lastServoMove = 0;

/* ================== STEPPER STATE ================== */
unsigned long lastPulse = 0;
bool pulseState = LOW;

/* ================================================= */
void setup() {
  pinMode(FL_DIR, OUTPUT); pinMode(FL_PUL, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(FR_PUL, OUTPUT);
  pinMode(RL_DIR, OUTPUT); pinMode(RL_PUL, OUTPUT);
  pinMode(RR_DIR, OUTPUT); pinMode(RR_PUL, OUTPUT);

  FL_S.attach(FL_SERVO);
  FR_S.attach(FR_SERVO);
  RL_S.attach(RL_SERVO);
  RR_S.attach(RR_SERVO);

  writeAllServos(90);
}

/* ================================================= */
void loop() {

  /* ===== FORWARD + 135° ===== */
  setDirection(true);
  setServoTargets(135);
  runSteppersWithServoUpdate(RUN_TIME_MS);
  stopSteppers();
  delay(PAUSE_MS);

  /* ===== BACKWARD + 45° ===== */
  setDirection(false);
  setServoTargets(45);
  runSteppersWithServoUpdate(RUN_TIME_MS);
  stopSteppers();
  delay(PAUSE_MS);
}

/* ================== CORE FUNCTIONS ================== */

void runSteppersWithServoUpdate(unsigned long durationMs) {
  unsigned long startMs = millis();
  lastPulse = micros();

  while (millis() - startMs < durationMs) {
    updateSteppers();
    updateServosStepwise();
  }
}

void updateSteppers() {
  unsigned long now = micros();
  if (now - lastPulse >= STEP_PULSE_US) {
    lastPulse = now;
    pulseState = !pulseState;

    digitalWrite(FL_PUL, pulseState);
    digitalWrite(FR_PUL, pulseState);
    digitalWrite(RL_PUL, pulseState);
    digitalWrite(RR_PUL, pulseState);
  }
}

void updateServosStepwise() {
  unsigned long now = millis();
  if (now - lastServoMove < SERVO_STEP_MS) return;
  lastServoMove = now;

  stepServo(curFL, tgtFL, FL_S);
  stepServo(curFR, tgtFR, FR_S);
  stepServo(curRL, tgtRL, RL_S);
  stepServo(curRR, tgtRR, RR_S);
}

/* ================== HELPERS ================== */

void stepServo(int &current, int target, Servo &s) {
  if (current < target) current++;
  else if (current > target) current--;
  s.write(current);
}

void setServoTargets(int angle) {
  tgtFL = tgtFR = tgtRL = tgtRR = angle;
}

void writeAllServos(int angle) {
  curFL = curFR = curRL = curRR = angle;
  FL_S.write(angle);
  FR_S.write(angle);
  RL_S.write(angle);
  RR_S.write(angle);
}

void setDirection(bool forward) {
  digitalWrite(FL_DIR, forward ? HIGH : LOW);
  digitalWrite(RL_DIR, forward ? HIGH : LOW);

  // FR & RR reversed
  digitalWrite(FR_DIR, forward ? LOW : HIGH);
  digitalWrite(RR_DIR, forward ? LOW : HIGH);
}

void stopSteppers() {
  digitalWrite(FL_PUL, LOW);
  digitalWrite(FR_PUL, LOW);
  digitalWrite(RL_PUL, LOW);
  digitalWrite(RR_PUL, LOW);
}
