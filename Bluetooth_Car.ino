//Including Arduino Core Library
#include <Arduino.h>

// Bluetooth robot controller using HC-05 on A2 (RX) and A3 (TX)
// ================= MOTOR & SENSOR PINS =================
const int ENA = 11;   // Right motor PWM
const int IN1 = 13;   // Right motor direction 1
const int IN2 = 12;   // Right motor direction 2

const int ENB = 3;    // Left motor PWM
const int IN3 = 2;    // Left motor direction 1
const int IN4 = A1;   // Left motor direction 2

// ================= SPEED SETTINGS =================
int currentSpeed = 200;   // default cruise speed
const int MAX_SPEED = 255;
const int MIN_SPEED = 80;
const int SPEED_STEP = 15;

// Spin speeds for in-place rotation
int spinForwardSpeed = 200;   // left wheel forward when spinning right
int spinBackwardSpeed = 100;  // right wheel backward when spinning right

// Timed-turn tuning (ms) — adjust to match ~90 degrees on your robot
const unsigned long TURN_MS = 650UL;

// Use SoftwareSerial on A2 (RX) and A3 (TX)
#include <SoftwareSerial.h>
SoftwareSerial btSerial(A2, A3); // (RX, TX) -> HC-05 TX -> A2 ; HC-05 RX <- A3

// small helper: clamp int
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ============== MOTOR CONTROL (full: forward, reverse, stop) ==============
void rightMotor(int speed) { // speed >0 forward, <0 backward, 0 stop
  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void leftMotor(int speed) { // speed >0 forward, <0 backward, 0 stop
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ================== HIGH-LEVEL MOVEMENT ==================
void moveForward(int spd = -1) {
  if (spd < 0) spd = currentSpeed;
  spd = clampInt(spd, 0, 255);
  rightMotor(spd);
  leftMotor(spd);
}

void moveBackward(int spd = -1) {
  if (spd < 0) spd = currentSpeed - 40;
  spd = clampInt(spd, 0, 255);
  rightMotor(-spd);
  leftMotor(-spd);
}

void stopMotors() {
  rightMotor(0);
  leftMotor(0);
}

void spinRight(int leftF = -1, int rightB = -1) {
  if (leftF < 0) leftF = spinForwardSpeed;
  if (rightB < 0) rightB = spinBackwardSpeed;
  leftMotor(leftF);    // left forward
  rightMotor(-rightB); // right backward
}

void spinLeft(int leftB = -1, int rightF = -1) {
  if (leftB < 0) leftB = spinBackwardSpeed;
  if (rightF < 0) rightF = spinForwardSpeed;
  leftMotor(-leftB);   // left backward
  rightMotor(rightF);  // right forward
}

// Timed 90deg-ish right turn (open-loop)
void timedTurnRight90() {
  // brief braking
  stopMotors();
  delay(80);
  // spin
  spinRight();
  delay(TURN_MS);
  stopMotors();
  delay(80);
}

// ================= SETUP =================
void setup() {
  // configure motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // init stopped
  stopMotors();

  // Start Bluetooth serial
  btSerial.begin(9600); // common HC-05 default
  Serial.begin(115200); // for USB debug if needed

  Serial.println("BT controller (A2/A3) ready");
  btSerial.println("BT: Ready");
}

// ================= LOOP =================
void loop() {
  // Process Bluetooth incoming bytes (if any)
  if (btSerial.available()) {
    char c = (char)btSerial.read();
    // ignore CR/LF
    if (c == '\r' || c == '\n') return;

    // Echo to hardware serial for debug
    Serial.print("BT Cmd: ");
    Serial.println(c);

    // Command dispatch
    switch (c) {
      case 'F': case 'f':
        moveForward();
        btSerial.println("OK: FORWARD");
        break;
      case 'B': case 'b':
        moveBackward();
        btSerial.println("OK: BACKWARD");
        break;
      case 'L': case 'l':
        spinLeft();
        btSerial.println("OK: LEFT");
        break;
      case 'R': case 'r':
        spinRight();
        btSerial.println("OK: RIGHT");
        break;
      case 'S': case 's':
        stopMotors();
        btSerial.println("OK: STOP");
        break;
      case 'T': case 't':
        btSerial.println("OK: TURN90");
        timedTurnRight90();
        btSerial.println("OK: TURN_DONE");
        break;
      case '+':
        currentSpeed = clampInt(currentSpeed + SPEED_STEP, MIN_SPEED, MAX_SPEED);
        btSerial.print("OK: SPEED=");
        btSerial.println(currentSpeed);
        break;
      case '-':
        currentSpeed = clampInt(currentSpeed - SPEED_STEP, MIN_SPEED, MAX_SPEED);
        btSerial.print("OK: SPEED=");
        btSerial.println(currentSpeed);
        break;
      case 'P': case 'p':
        btSerial.println("OK");
        break;
      default:
        btSerial.print("ERR:UNKNOWN:");
        btSerial.println(c);
        break;
    }
  }

  // tiny loop delay
  delay(10);
}