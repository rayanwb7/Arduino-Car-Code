// ================= MOTOR PINS =================
const int ENA = 11;   // Right motor PWM
const int IN1 = 13;   // Right motor direction 1
const int IN2 = 12;   // Right motor direction 2

const int ENB = 3;    // Left motor PWM
const int IN3 = 2;    // Left motor direction 1
const int IN4 = A1;   // Left motor direction 2

// ================= TUNABLE SPEEDS =================
//speed has been tuned low to ensure the ultrasonic sensor has enough time to react
#define BASE_SPEED   130
#define OUTER_TURN   200
#define INNER_TURN   160
#define SPIN_FW      200
#define SPIN_BW      100

// ================= ULTRASONIC PINS =================
const int TRIG_PIN = A2;
const int ECHO_PIN = A3;

const int OBSTACLE_DISTANCE = 30; // cm threshold

// ================== ULTRASONIC FUNCTION ==================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms = ~5m range

  if (duration == 0) return 9999; // timeout meaning no object

  long distance = (duration * 0.0343) / 2; // convert to cm
  return distance;
}

// ================= MOTOR CONTROL =================
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, BASE_SPEED);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, BASE_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void turnRight() {
  // Right wheel backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, SPIN_BW);

  // Left wheel forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, SPIN_FW);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(500);
}

void loop() {
  long dist = getDistance();

  if (dist < OBSTACLE_DISTANCE) {
    // Obstacle detected → turn right
    stopMotors();
    delay(100);

    turnRight();
    delay(1000);   // <-- adjust for perfect 90° depending on your robot

    stopMotors();
    delay(100);

    moveForward();
  }
  else {
    // No obstacle → keep going straight
    moveForward();
  }

  delay(50);
}
