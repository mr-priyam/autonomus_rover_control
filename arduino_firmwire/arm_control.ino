/* ROBOTIC ARM & BASE CONTROL
   - Shoulder/Elbow: Controlled via Relays (Data: A3, A7)
   - Movement/Base: Controlled via PWM/DIR Driver (Data: B0–B3)
   - Auxiliary (Motor 3): Controlled via Stepper Driver (Data: B4, B5)
*/

// ===== EXISTING VARIABLES =====
int shoulder_joint = 0;
int elbow_joint    = 0;
int buttons[6]     = {0, 0, 0, 0, 0, 0};

// ===== RELAY PINS (Shoulder & Elbow) =====
#define IN1 42   // Shoulder
#define IN2 40   // Shoulder
#define IN3 44   // Elbow
#define IN4 46   // Elbow

// ===== DC MOTOR DRIVER PINS (Movement) =====
#define DIR1_PIN 7
#define DIR2_PIN 11
#define PWM1_PIN 5
#define PWM2_PIN 9

// ===== STEPPER DRIVER PINS (Motor 3) =====
#define DIR3_PIN 25
#define PUL3_PIN 23

// ===== FUNCTION PROTOTYPES =====
void readSerialData();
void handleMovement();
void handleStepper();
void stopAllRelays();
void stopMovement();
void printParsedValues();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Relay pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // DC motor pins
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  // Stepper pins
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);

  stopAllRelays();
  stopMovement();

  Serial.println("Arduino Ready: Arm, Base & Stepper Logic Active");
}

void loop() {
  readSerialData();

  /* ===== SHOULDER ACTUATOR ===== */
  if (shoulder_joint == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (shoulder_joint == -1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  /* ===== ELBOW ACTUATOR ===== */
  if (elbow_joint == 1) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (elbow_joint == -1) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  handleMovement();
  handleStepper();
}

/* ================= STEPPER LOGIC ================= */
void handleStepper() {
  if (buttons[4] == 1) {
    digitalWrite(DIR3_PIN, HIGH);
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(20);
  } else if (buttons[5] == 1) {
    digitalWrite(DIR3_PIN, LOW);
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(20);
  }
}

/* ================= MOVEMENT LOGIC ================= */
void handleMovement() {
  if (buttons[1] == 1) {          // Forward
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, LOW);
    analogWrite(PWM1_PIN, 50);
    analogWrite(PWM2_PIN, 50);
  } else if (buttons[0] == 1) {   // Backward
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, HIGH);
    analogWrite(PWM1_PIN, 50);
    analogWrite(PWM2_PIN, 50);
  } else if (buttons[2] == 1) {   // Left
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, LOW);
    analogWrite(PWM1_PIN, 50);
    analogWrite(PWM2_PIN, 50);
  } else if (buttons[3] == 1) {   // Right
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, HIGH);
    analogWrite(PWM1_PIN, 50);
    analogWrite(PWM2_PIN, 50);
  } else {
    stopMovement();
  }
}

void stopMovement() {
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
}

/* ================= SERIAL PARSING ================= */
void readSerialData() {
  if (!Serial.available()) return;

  Serial.setTimeout(10);
  String data = Serial.readStringUntil('\n');
  data.trim();

  int a3_index = data.indexOf("A3");
  int a7_index = data.indexOf("A7");
  int b_index  = data.indexOf("B");

  if (a3_index == -1 || a7_index == -1 || b_index == -1) return;

  // Parse A3 and A7
  shoulder_joint = data.substring(a3_index + 3, a7_index).toInt();
  elbow_joint    = data.substring(a7_index + 3, b_index).toInt();

  // Parse B values
  String b_data = data.substring(b_index + 2);
  b_data.trim();

  for (int i = 0; i < 6; i++) {
    int spaceIndex = b_data.indexOf(' ');
    if (spaceIndex != -1) {
      buttons[i] = b_data.substring(0, spaceIndex).toInt();
      b_data = b_data.substring(spaceIndex + 1);
    } else {
      buttons[i] = b_data.toInt();
    }
  }

  // printParsedValues();
}

/* ================= DEBUG PRINT ================= */
void printParsedValues() {
  Serial.print("A3 (Shoulder): ");
  Serial.print(shoulder_joint);
  Serial.print(" | A7 (Elbow): ");
  Serial.print(elbow_joint);
  Serial.print(" | B: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(buttons[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/* ================= SAFETY ================= */
void stopAllRelays() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}