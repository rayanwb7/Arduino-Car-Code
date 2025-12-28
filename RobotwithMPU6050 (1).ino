// ================= MOTOR PINS =================
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = 2;
const int IN4 = A1;

// ================= TUNABLE MOTOR SPEEDS =================

// Normal driving speed
int BASE_SPEED_A = 200;
int BASE_SPEED_B = 255;

// Climbing speed (full power)
int CLIMB_SPEED_A = 255;
int CLIMB_SPEED_B = 255;

// Rotation speed
int SPIN_SPEED_A = 200;
int SPIN_SPEED_B = 200;

// ================= LCD =================
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================= MPU-6050 =================
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
Adafruit_MPU6050 mpu;

// ================= VARIABLES =================
unsigned long start_time = 0;
float currentAngle = 0;
float yawDeg = 0.0;
float maxAngle=0;

// STATE MACHINE
// 0 = Flat/Searching
// 1 = Climbing (Angle is high)
// 2 = Reached Top (Angle is flat again)
int robotState = 0; 

// THRESHOLDS
const float ANGLE_TO_START_CLIMB = 26.0; 
const float ANGLE_TO_STOP_CLIMB  = 5.0;

// ================= MOTOR FUNCTIONS =================
void motorA(int speed) {
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

void motorB(int speed) {
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

void moveForward() {
  motorA(BASE_SPEED_A);
  motorB(BASE_SPEED_B);
}

void moveClimbing() {
  motorA(CLIMB_SPEED_A);
  motorB(CLIMB_SPEED_B);
}

void spinRight() {
  motorA(SPIN_SPEED_A);
  motorB(-SPIN_SPEED_B);
}

void stopMotors() {
  motorA(0);
  motorB(0);
}

// ================= GYRO ROTATION =================
void rotate360() {
  lcd.setCursor(0, 1);
  lcd.print("Action: SPIN 360");

  yawDeg = 0.0;
  unsigned long last = micros();

  spinRight(); 

  while (abs(yawDeg) < 360.0) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = micros();
    float dt = (now - last) / 1e6;
    last = now;

    float gz_deg = g.gyro.z * (180.0 / PI); 
    yawDeg += gz_deg * dt;

    if (dt > 0.1) dt = 0; 
  }

  stopMotors();
  delay(500);
}

// ================= SETUP =================
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Init Gyro...");

  if (!mpu.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("MPU FAIL");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lcd.clear();
  lcd.print("READY TO GO");
  delay(1000);

  start_time = millis();
}

// ================= MAIN LOOP =================
void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. CALCULATE CURRENT ANGLE (ONLY LIVE ANGLE)
  float angle = (atan((a.acceleration.x / a.acceleration.z)) * 180/PI);
  currentAngle = abs(angle);

  // 2. STATE MACHINE
  switch (robotState) {

    case 0: // FLAT DRIVING
      moveForward();
      if (currentAngle > ANGLE_TO_START_CLIMB) {
        robotState = 1;
      }
      break;

    case 1: // CLIMBING
      moveClimbing();
      if (currentAngle < ANGLE_TO_STOP_CLIMB) {
        delay(100);
        mpu.getEvent(&a, &g, &temp);

        float check = abs(atan((a.acceleration.x / a.acceleration.z)) * 180/PI);
        if (check < ANGLE_TO_STOP_CLIMB) {
          robotState = 2;
        }
      }
      break;

    case 2: // TOP OF RAMP
      stopMotors();
      delay(1000);
      rotate360();
      delay(500);
      robotState = 0;
      break;
  }

  // 3. LCD DISPLAY (ONLY CURRENT ANGLE)
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 200) {

    lcd.clear();

    // Line 1: Angle + state
    lcd.setCursor(0, 0);
    lcd.print("Ang:");
    lcd.print(currentAngle, 1);

    lcd.setCursor(9, 0);
    if (robotState == 0) lcd.print("FLAT");
    if (robotState == 1) lcd.print("CLIMB");
    if (robotState == 2) lcd.print("TOP");

    // Line 2: Timer
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print((millis() - start_time) / 1000);
    lcd.print("s");

    lastLCD = millis();
  }
}
