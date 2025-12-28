//including Arduino Core Library
#include <Arduino.h>

// ================= MOTOR & SENSOR PINS =================
const int ENA = 11;   // Right motor PWM
const int IN1 = 13;   // Right motor direction 1
const int IN2 = 12;   // Right motor direction 2

const int ENB = 3;    // Left motor PWM
const int IN3 = 2;    // Left motor direction 1
const int IN4 = A1;   // Left motor direction 2

const int IR_Left  = 1;   // Left IR sensor
const int IR_Right = A2;  // Right IR sensor

// ============== Encoder Pins ================
//const int Encoder_L = A4;
const int Encoder_R = A3;
//volatile long pulsesLeft  = 0;
volatile long pulsesRight = 0;

// ================= ENCODER PARAMETERS =================
const int PULSES_PER_REV = 20;           // slots per encoder disc
const float WHEEL_DIAMETER = 0.065;      // wheel diameter in meters (65 mm)
const float WHEEL_CIRC = 3.14159 * WHEEL_DIAMETER;
#include <PinChangeInterrupt.h>

// ================= SPEED SETTINGS =================
#define BASE_SPEED   200     // straight speed
#define OUTER_TURN   200     // outer wheel when correcting
#define INNER_TURN   160     // inner wheel when correcting
#define SPIN_FW      200     // spin-correction forward wheel
#define SPIN_BW      100     // spin-correction backward wheel
int lastTurn = 0;           // 0=none, 1=left, 2=right

// including library
#include <LiquidCrystal.h>
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Time tracking
unsigned long start_time = 0;
unsigned long stop_time = 0;
bool isMoving = false;

// ========== 70cm PAUSE STATE ==========
bool pausedAfter70 = false;              // only pause once
const float TARGET_DIST_CM = 70.0;       // stop distance
const unsigned long PAUSE_MS = 2000;     // 2 s pause

// ================= INTERRUPTS =================
//void leftEncoderISR() {
//  pulsesLeft++;
//}

void rightEncoderISR() {
  pulsesRight++;
}

// ============== MOTOR CONTROL (full: forward, reverse, stop) ==============
void motorA(int speed) { // Right motor
  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  } 
  else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speed);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorB(int speed) { // Left motor
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } 
  else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speed);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ================= DISTANCE FUNCTIONS =================
float getRightDistance() {
  // returns distance travelled by right wheel in cm
  return (pulsesRight / (float)PULSES_PER_REV) * WHEEL_CIRC * 100.0; 
}

//float getLeftDistance() {
  //return (pulsesLeft / (float)PULSES_PER_REV) * WHEEL_CIRC;
//}

//float getAverageDistance() {
  //return ((getLeftDistance() + getRightDistance()) / 2.0)* 100;
//}

// ================== HIGH-LEVEL MOVEMENT ==================
void moveForward() {
  motorA(BASE_SPEED);
  motorB(BASE_SPEED);
}

void turnLeft() {
  motorA(INNER_TURN);   // Right motor slow
  motorB(OUTER_TURN);   // Left motor fast
}

void turnRight() {
  motorA(OUTER_TURN);   // Right motor fast
  motorB(INNER_TURN);   // Left motor slow
}

void spinLeft() {
  motorA(-SPIN_BW);
  motorB(SPIN_FW);
}

void spinRight() {
  motorA(SPIN_FW);
  motorB(-SPIN_BW);
}

void stopMotors() {
  motorA(0);
  motorB(0);
}

// ================= SETUP =================
void setup() {
  // initialize encoder count
  pulsesRight = 0;

  pinMode(IR_Left, INPUT);
  pinMode(IR_Right, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // LCD Setup for Distance + Time
  lcd.begin(16, 2);

  // Line 1: Distance
  lcd.setCursor(0, 0);
  lcd.print("Dist: 0.00 cm");

  // Line 2: Time
  lcd.setCursor(0, 1);
  lcd.print("Time: 00:000");

  delay(1000);

  moveForward();      // start moving
  start_time = millis(); // record start time
  isMoving = true;

  //pinMode(Encoder_L, INPUT_PULLUP);
  pinMode(Encoder_R, INPUT_PULLUP);

  //attachPCINT(digitalPinToPCINT(Encoder_L), leftEncoderISR, RISING);
  attachPCINT(digitalPinToPCINT(Encoder_R), rightEncoderISR, RISING);
}

// ================= LOOP =================
void loop() {
// ---- 70cm PAUSE LOGIC ----
  float dist = getRightDistance();
  if (!pausedAfter70 && dist >= TARGET_DIST_CM) {
    stopMotors();
    delay(PAUSE_MS);        // stop for 2 seconds
    pausedAfter70 = true;   // don't do it again
    moveForward();          // resume
  }

  int L = digitalRead(IR_Left);
  int R = digitalRead(IR_Right);

  // 1. Both sensors on WHITE
  if (L == 1 && R == 1) {

    if (lastTurn == 1) {     // overshoot from a left corner
      spinLeft();
    }
    else if (lastTurn == 2) { // overshoot from a right corner
      spinRight();
    }
    else {
      moveForward();
      lastTurn = 0;
    }
  }

  // 2. (H, L) → drifting left → turn RIGHT
  else if (L == 0 && R == 1) {
    turnRight();
    lastTurn = 2;
  }

  // 3. (L, H) → drifting right → turn LEFT
  else if (L == 1 && R == 0) {
    turnLeft();
    lastTurn = 1;
  }

  else if (L == 0 && R == 0){
    stopMotors();
    delay(100);
    while(1);
  }

  if (isMoving) {
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - start_time;

    // Convert to seconds + milliseconds
    int seconds = elapsed / 1000;
    int milliseconds = elapsed % 1000;
    float totalDistance = getRightDistance();

    // --- LCD UPDATE ---
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(totalDistance, 2);
    lcd.print("cm   ");  // overwrite old characters
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    if (seconds < 10) lcd.print("0");
    lcd.print(seconds);
    lcd.print(":");

    if (milliseconds < 100) lcd.print("0");
    if (milliseconds < 10) lcd.print("0");
    lcd.print(milliseconds);

    // Stop after 60 seconds
    if (elapsed >= 60000) {
      stop_time = millis();
      stopMotors();
      isMoving = false;
      lcd.clear();
      lcd.print("Finished!");
      lcd.setCursor(0, 1);
      lcd.print("Dist:");
      lcd.print(totalDistance, 2);
      lcd.print("cm");
    }
  }
}