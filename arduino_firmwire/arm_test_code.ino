/* ========= HARDWARE TEST CODE =========
   Serial-based testing for:
   - Shoulder relay
   - Elbow relay
   - DC motors (base movement)
   - Stepper motor
*/

#define IN1 42   // Shoulder
#define IN2 40
#define IN3 44   // Elbow
#define IN4 46

#define DIR1_PIN 7
#define DIR2_PIN 11
#define PWM1_PIN 5
#define PWM2_PIN 9

#define DIR3_PIN 25
#define PUL3_PIN 23

char cmd;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);

  stopAll();

  Serial.println("=== HARDWARE TEST MODE ===");
  Serial.println("Press keys 1–9, 0 or s");
}

void loop() {
  if (!Serial.available()) return;
  cmd = Serial.read();
  stopAll();   // Safety: stop before next test

  switch (cmd) {

    case '1':  // Shoulder forward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      Serial.println("Shoulder Forward");
      break;

    case '2':  // Shoulder reverse
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      Serial.println("Shoulder Reverse");
      break;

    case '3':  // Elbow forward
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      Serial.println("Elbow Forward");
      break;

    case '4':  // Elbow reverse
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Elbow Reverse");
      break;

    case '5':  // DC forward
      digitalWrite(DIR1_PIN, HIGH);
      digitalWrite(DIR2_PIN, LOW);
      analogWrite(PWM1_PIN, 80);
      analogWrite(PWM2_PIN, 80);
      Serial.println("DC Forward");
      break;

    case '6':  // DC backward
      digitalWrite(DIR1_PIN, LOW);
      digitalWrite(DIR2_PIN, HIGH);
      analogWrite(PWM1_PIN, 80);
      analogWrite(PWM2_PIN, 80);
      Serial.println("DC Backward");
      break;

    case '7':  // DC left
      digitalWrite(DIR1_PIN, LOW);
      digitalWrite(DIR2_PIN, LOW);
      analogWrite(PWM1_PIN, 80);
      analogWrite(PWM2_PIN, 80);
      Serial.println("DC Left");
      break;

    case '8':  // DC right
      digitalWrite(DIR1_PIN, HIGH);
      digitalWrite(DIR2_PIN, HIGH);
      analogWrite(PWM1_PIN, 80);
      analogWrite(PWM2_PIN, 80);
      Serial.println("DC Right");
      break;

    case '9':  // Stepper CW
      Serial.println("Stepper CW");
      digitalWrite(DIR3_PIN, HIGH);
      stepperPulse(200);
      break;

    case '0':  // Stepper CCW
      Serial.println("Stepper CCW");
      digitalWrite(DIR3_PIN, LOW);
      stepperPulse(200);
      break;

    case 's':
      stopAll();
      Serial.println("STOP ALL");
      break;
  }
}

/* ===== STEPPER PULSE ===== */
void stepperPulse(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(20);
  }
}

/* ===== STOP EVERYTHING ===== */
void stopAll() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
}
