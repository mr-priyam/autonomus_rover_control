/* ROBOTIC ARM & BASE CONTROL
   - Shoulder/Elbow: Controlled via Relays (Data: A3, A1)
   - Movement/Base: Controlled via PWM/DIR Driver (Data: B0-B3)
*/

// ===== EXISTING VARIABLES =====
int shoulder_joint = 0;
int elbow_joint    = 0;
int buttons[6]     = {0, 0, 0, 0, 0, 0};

// ===== RELAY PINS (Shoulder & Elbow) =====
#define IN1 5   // Shoulder
#define IN2 6   // Shoulder
#define IN3 9   // Elbow
#define IN4 10  // Elbow

// ===== NEW MOTOR DRIVER PINS (Movement) =====
// TODO: Update these pin numbers to match your wiring!
// PWM pins on Uno: 3, 5, 6, 9, 10, 11
#define DIR1_PIN 4
#define DIR2_PIN 7
#define PWM1_PIN 3
#define PWM2_PIN 11

void readSerialData();
void handleMovement();
void stopAllRelays();
void stopMovement();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // --- Setup Relay Pins ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // --- Setup Motor Driver Pins ---
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  stopAllRelays();
  stopMovement();
  
  Serial.println("Arduino Ready: Arm & Movement Logic Active");
}

void loop() {
  readSerialData();

  /* ===== SHOULDER ACTUATOR (Relays) ===== */
  if (shoulder_joint == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (shoulder_joint == -1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  /* ===== ELBOW ACTUATOR (Relays) ===== */
  if (elbow_joint == 1) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (elbow_joint == -1) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  /* ===== MOVEMENT CONTROL (PWM/DIR) ===== */
  handleMovement();
}

/* ================= MOVEMENT LOGIC ================= */
void handleMovement() {
  // Logic based on buttons[0] to buttons[3]
  
  // UPWARD: b[0] == 1 -> DIR1 High, DIR2 Low
  if (buttons[0] == 1) {
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, LOW);
    analogWrite(PWM1_PIN, 200);
    analogWrite(PWM2_PIN, 200);
    Serial.println("upward");
  }
  // DOWNWARD: b[1] == 1 -> DIR1 Low, DIR2 High
  else if (buttons[1] == 1) {
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, HIGH);
    analogWrite(PWM1_PIN, 200);
    analogWrite(PWM2_PIN, 200);
    Serial.println("downward");
  }
  // LEFTWARD: b[2] == 1 -> DIR1 Low, DIR2 Low
  else if (buttons[2] == 1) {
    digitalWrite(DIR1_PIN, LOW);
    digitalWrite(DIR2_PIN, LOW);
    analogWrite(PWM1_PIN, 200);
    analogWrite(PWM2_PIN, 200);
    Serial.println("leftward");
  }
  // RIGHTWARD: b[3] == 1 -> DIR1 High, DIR2 High
  else if (buttons[3] == 1) {
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, HIGH);
    analogWrite(PWM1_PIN, 200);
    analogWrite(PWM2_PIN, 200);
    Serial.println("rightward");
  }
  // STOP: If no movement buttons are pressed
  else {
    stopMovement();
  }
}

void stopMovement() {
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  // Optional: Reset DIR pins to LOW if needed, but PWM 0 is usually sufficient to stop
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
}

/* ================= SERIAL READ ================= */
void readSerialData() {
  if (!Serial.available()) return;

  String data = Serial.readStringUntil('\n');
  data.trim();

  // Basic validation to ensure data integrity
  int a3_index = data.indexOf("A3");
  int a1_index = data.indexOf("A1");
  int b_index  = data.indexOf("B");

  if (a3_index == -1 || a1_index == -1 || b_index == -1) return;

  // ---- Parse A3 and A1 ----
  shoulder_joint = data.substring(a3_index + 3, a1_index).toInt();
  elbow_joint    = data.substring(a1_index + 3, b_index).toInt();

  // ---- Parse B values ----
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
  
  // Debug printing (Optional: comment out if too fast)
  // Serial.print("S:"); Serial.print(shoulder_joint);
  // Serial.print(" E:"); Serial.print(elbow_joint);
  // Serial.print(" B0:"); Serial.println(buttons[0]);
}

/* ================= SAFETY ================= */
void stopAllRelays() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}