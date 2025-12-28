// ================= MOTOR PINS =================
const int ENA = 11;   // Right motor PWM
const int IN1 = 13;   // Right motor direction
const int IN2 = 12;

const int ENB = 3;    // Left motor PWM
const int IN3 = 2;    // Left motor direction
const int IN4 = A1;

int testSpeed = 180;  // Adjust speed as needed

// ================= MOTOR FUNCTIONS =================
void motorA(int speed) {  // Right motor
  if (speed > 0) { // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  }
  else if (speed < 0) { // Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  }
  else { // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorB(int speed) {  // Left motor
  if (speed > 0) { // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else if (speed < 0) { // Reverse
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  }
  else { // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.println("Motor Test Starting...");
  delay(2000);
}

// ================= TEST SEQUENCE =================
void loop() {

  Serial.println("1. Right Motor FORWARD");
  motorA(testSpeed);
  motorB(0);
  delay(2000);

  Serial.println("2. Right Motor REVERSE");
  motorA(-testSpeed);
  motorB(0);
  delay(2000);

  Serial.println("3. Left Motor FORWARD");
  motorA(0);
  motorB(testSpeed);
  delay(2000);

  Serial.println("4. Left Motor REVERSE");
  motorA(0);
  motorB(-testSpeed);
  delay(2000);

  Serial.println("5. Both Motors FORWARD");
  motorA(testSpeed);
  motorB(testSpeed);
  delay(2000);

  Serial.println("6. Pivot LEFT");
  motorA(-testSpeed);
  motorB(testSpeed);
  delay(2000);

  Serial.println("7. Pivot RIGHT");
  motorA(testSpeed);
  motorB(-testSpeed);
  delay(2000);

  Serial.println("8. STOP");
  motorA(0);
  motorB(0);
  delay(3000);

  Serial.println("Repeating test...");
}
