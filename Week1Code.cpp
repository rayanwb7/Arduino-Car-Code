//including the header file for Arduino
#include <Arduino.h>

// Motor A, declaring which pin it should be connected to on the Arduino Board
const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;
#define s 255   //defining global speed variable

//Motor B, declaring which pin it should be connected to
const int ENB = 3;
const int IN3 = 2;
const int IN4 = A1;

// including library
#include <LiquidCrystal.h>
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Time tracking
unsigned long start_time = 0;
unsigned long stop_time = 0;
bool isMoving = false;  // flag to know if car is moving

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Timer setup
  lcd.begin(16, 2);
  lcd.print("Week 1: Move");
  lcd.setCursor(0, 1);
  lcd.print("Time: 00:000");

  Serial.begin(9600);
  delay(1000);

  moveForward();      // start moving
  start_time = millis(); // record start time
  isMoving = true;
}

//Forward Moving Function
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 140);      //values can still be tweaked!
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 131);    //values can still be tweaked!
}

// Stop the Car Function
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// Main Function
void loop() {
  if (isMoving) {
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - start_time;

    // Convert to seconds + milliseconds
    int seconds = elapsed / 1000;
    int milliseconds = elapsed % 1000;

    // Update LCD timer
    lcd.setCursor(6, 1);
    if (seconds < 10) lcd.print("0");
    lcd.print(seconds);
    lcd.print(":");
    if (milliseconds < 100) lcd.print("0");
    if (milliseconds < 10) lcd.print("0");
    lcd.print(milliseconds);

    // Stop after 10 seconds
    if (elapsed >= 10000) {
      stop_time = millis();
      stopCar();
      isMoving = false;
      lcd.clear();
      lcd.print("Movement done!");
      lcd.setCursor(0, 1);
      lcd.print("Time: ");
      lcd.print(seconds);
      lcd.print("s");
    }
  }
}