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
const unsigned long STEP_PULSE_US = 10;
const unsigned long MOTOR_RUN_MS  = 2000;
const unsigned long SERVO_STEP_MS = 20;

/* ================================================= */
void setup() {
  Serial.begin(115200);

  pinMode(FL_DIR, OUTPUT); pinMode(FL_PUL, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(FR_PUL, OUTPUT);
  pinMode(RL_DIR, OUTPUT); pinMode(RL_PUL, OUTPUT);
  pinMode(RR_DIR, OUTPUT); pinMode(RR_PUL, OUTPUT);

  FL_S.attach(FL_SERVO);
  FR_S.attach(FR_SERVO);
  RL_S.attach(RL_SERVO);
  RR_S.attach(RR_SERVO);

  writeAllServos(90);
  Serial.println("Motor + Servo Test Ready");
}

/* ================================================= */
void loop() {
  if (!Serial.available()) return;

  int cmd = Serial.parseInt();   // <-- IMPORTANT

  switch (cmd) {
    case 1:  runSingleMotor(FL_DIR, FL_PUL, true);  break;
    case 2:  runSingleMotor(FR_DIR, FR_PUL, false); break;
    case 3:  runSingleMotor(RL_DIR, RL_PUL, true);  break;
    case 4:  runSingleMotor(RR_DIR, RR_PUL, false); break;

    case 5:  servoSequenceAll(); break;
    case 6:  servoSequenceSingle(FL_S); break;
    case 7:  servoSequenceSingle(FR_S); break;
    case 8:  servoSequenceSingle(RL_S); break;
    case 9:  servoSequenceSingle(RR_S); break;

    case 10: runAllMotors(true);  break;   // all forward
    case 11: runAllMotors(false); break;   // all backward
  }
}

/* ================== MOTOR FUNCTIONS ================== */

void runSingleMotor(int dirPin, int pulPin, bool normalDir) {
  digitalWrite(dirPin, normalDir ? HIGH : LOW);

  unsigned long start = millis();
  unsigned long lastPulse = micros();
  bool pulse = LOW;

  while (millis() - start < MOTOR_RUN_MS) {
    if (micros() - lastPulse >= STEP_PULSE_US) {
      lastPulse = micros();
      pulse = !pulse;
      digitalWrite(pulPin, pulse);
    }
  }
  digitalWrite(pulPin, LOW);
}

void runAllMotors(bool forward) {
  // Direction
  digitalWrite(FL_DIR, forward ? HIGH : LOW);
  digitalWrite(RL_DIR, forward ? HIGH : LOW);

  digitalWrite(FR_DIR, forward ? LOW : HIGH);
  digitalWrite(RR_DIR, forward ? LOW : HIGH);

  unsigned long start = millis();
  unsigned long lastPulse = micros();
  bool pulse = LOW;

  while (millis() - start < MOTOR_RUN_MS) {
    if (micros() - lastPulse >= STEP_PULSE_US) {
      lastPulse = micros();
      pulse = !pulse;

      digitalWrite(FL_PUL, pulse);
      digitalWrite(FR_PUL, pulse);
      digitalWrite(RL_PUL, pulse);
      digitalWrite(RR_PUL, pulse);
    }
  }

  stopAllMotors();
}

void stopAllMotors() {
  digitalWrite(FL_PUL, LOW);
  digitalWrite(FR_PUL, LOW);
  digitalWrite(RL_PUL, LOW);
  digitalWrite(RR_PUL, LOW);
}

/* ================== SERVO FUNCTIONS ================== */

void servoSequenceAll() {
  moveAllServosStepwise(135);
  moveAllServosStepwise(45);
  moveAllServosStepwise(90);
}

void servoSequenceSingle(Servo &s) {
  moveServoStepwise(s, 135);
  moveServoStepwise(s, 45);
  moveServoStepwise(s, 90);
}

void moveAllServosStepwise(int target) {
  int cur = FL_S.read();
  while (cur != target) {
    cur += (cur < target) ? 1 : -1;
    FL_S.write(cur);
    FR_S.write(cur);
    RL_S.write(cur);
    RR_S.write(cur);
    delay(SERVO_STEP_MS);
  }
}

void moveServoStepwise(Servo &s, int target) {
  int cur = s.read();
  while (cur != target) {
    cur += (cur < target) ? 1 : -1;
    s.write(cur);
    delay(SERVO_STEP_MS);
  }
}

void writeAllServos(int angle) {
  FL_S.write(angle);
  FR_S.write(angle);
  RL_S.write(angle);
  RR_S.write(angle);
}
