// EG1311 robot - final sketch (4-wheel or tricycle), straight-line run
// Wiring: see "EG1311 Final Wiring" page. Pin map unchanged from the
// minimum-wiring guide. 9V battery goes straight into Vin/GND (no barrel jack).
//
// Run: drive forward -> stop at STOP_CM from the wall -> throw -> reverse.
// No turning. Separate L/R PWM pins are kept only for straight-line trim.
//
// L293D enables tied to +5V, so one PWM pin per side:
//   forward: DIR LOW,  PWM = speed
//   reverse: DIR HIGH, PWM = 255 - speed   (inverted duty)
//
// DEBUG (Serial Monitor, 9600 baud), send one character:
//   l / L  channel A (left)  forward / reverse, 1 s
//   r / R  channel B (right) forward / reverse, 1 s
//   f / b  both forward / back, 1 s (with trim)
//   t      2 s straight-line trim test (with trim)
//   s      stop
//   d      print distance (cm)
//   0..9   servo to 0..180 degrees (x20)
// Set DEBUG_MODE = false for the real run.

#include <Servo.h>

const bool DEBUG_MODE = true;
const bool TRICYCLE   = false;   // true = one front wheel + two rear

// ---- tuning ----
const int CRUISE_SPEED = 200;    // 0..255
const int SLOW_SPEED   = 130;    // final approach
const int SLOW_CM      = 40;     // start slowing at this distance
// Front-bumper stop distance you want, in cm. In the tricycle the front
// wheel leads the sensor by ~8 cm, so the sensor must stop 8 cm further out.
const int BUMPER_STOP_CM = 10;
const int STOP_CM = BUMPER_STOP_CM + (TRICYCLE ? 8 : 0);
const unsigned long REVERSE_MS = 2500;  // time to get back behind the bump
const int ARM_REST  = 90;        // holding-ball angle
const int ARM_THROW = 170;       // release angle

// Straight-line trim, added to each side's speed (can be negative).
// Tricycle: channel B drives two motors (front + rear-right), so it sags a
// little; start with R_TRIM positive and tune with the 't' test.
const int L_TRIM = 0;
const int R_TRIM = TRICYCLE ? 8 : 0;

// ---- pins ----
const int L_PWM = 5;   // L293D IN1 (pin 2)   channel A
const int L_DIR = 4;   // L293D IN2 (pin 7)
const int R_PWM = 6;   // L293D IN3 (pin 10)  channel B
const int R_DIR = 7;   // L293D IN4 (pin 15)
const int US_VCC = 11, US_TRIG = 12, US_ECHO = 13;
const int SERVO_PIN = 9;

Servo arm;

void drive(int pwmPin, int dirPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(dirPin, LOW);  analogWrite(pwmPin, speed); }
  else            { digitalWrite(dirPin, HIGH); analogWrite(pwmPin, 255 + speed); }
}

void driveBoth(int left, int right) {
  drive(L_PWM, L_DIR, left);
  drive(R_PWM, R_DIR, right);
}

// Same speed both sides, trim applied in the direction of travel.
void driveStraight(int speed) {
  if (speed == 0) { driveBoth(0, 0); return; }
  int s = (speed > 0) ? 1 : -1;
  int mag = abs(speed);
  driveBoth(s * constrain(mag + L_TRIM, 0, 255),
            s * constrain(mag + R_TRIM, 0, 255));
}

void stopMotors() { driveBoth(0, 0); }

// Short reverse pulse to kill momentum instead of coasting into the wall.
void brake(int dirSign) {
  driveStraight(-dirSign * 150);
  delay(60);
  stopMotors();
}

long readDistanceCm() {
  digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  unsigned long us = pulseIn(US_ECHO, HIGH, 25000UL);
  if (us == 0) return -1;
  return us / 58;
}

// Median of 3 readings to ignore single bad echoes off the bump/ramp.
long distanceFiltered() {
  long a = readDistanceCm(), b = readDistanceCm(), c = readDistanceCm();
  if (a < 0) a = 999; if (b < 0) b = 999; if (c < 0) c = 999;
  long hi = max(a, max(b, c)), lo = min(a, min(b, c));
  return a + b + c - hi - lo;
}

void throwBall() {
  arm.write(ARM_THROW);
  delay(400);
  arm.write(ARM_REST);
  delay(300);
}

void runMission() {
  delay(1000);                                   // hands clear
  unsigned long t0 = millis();
  while (true) {
    long cm = distanceFiltered();
    if (cm <= STOP_CM) break;
    driveStraight(cm <= SLOW_CM ? SLOW_SPEED : CRUISE_SPEED);
    if (millis() - t0 > 15000UL) break;          // safety timeout
  }
  brake(+1);
  delay(200);
  throwBall();
  driveStraight(-CRUISE_SPEED);
  delay(REVERSE_MS);
  stopMotors();
}

void runFor(int left, int right, unsigned long ms, const char *label) {
  Serial.print(label); Serial.println(" ...");
  driveBoth(left, right);
  delay(ms);
  stopMotors();
  Serial.println("stopped");
}

void handleCommand(char c) {
  switch (c) {
    case 'l': runFor( CRUISE_SPEED, 0, 1000, "A forward");  break;
    case 'L': runFor(-CRUISE_SPEED, 0, 1000, "A reverse");  break;
    case 'r': runFor(0,  CRUISE_SPEED, 1000, "B forward");  break;
    case 'R': runFor(0, -CRUISE_SPEED, 1000, "B reverse");  break;
    case 'f': Serial.println("BOTH forward (trim)"); driveStraight( CRUISE_SPEED); delay(1000); stopMotors(); break;
    case 'b': Serial.println("BOTH back (trim)");    driveStraight(-CRUISE_SPEED); delay(1000); stopMotors(); break;
    case 't': Serial.println("TRIM TEST 2 s: measure sideways drift");
              driveStraight(CRUISE_SPEED); delay(2000); stopMotors(); break;
    case 's': stopMotors(); Serial.println("stopped"); break;
    case 'd': {
      long cm = readDistanceCm();
      Serial.print("distance: ");
      if (cm < 0) Serial.println("no echo");
      else { Serial.print(cm); Serial.print(" cm  (stop at "); Serial.print(STOP_CM); Serial.println(")"); }
      break;
    }
    default:
      if (c >= '0' && c <= '9') {
        int angle = (c - '0') * 20;
        arm.write(angle);
        Serial.print("servo -> "); Serial.println(angle);
      }
  }
}

void setup() {
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_DIR, OUTPUT);
  stopMotors();

  pinMode(US_VCC, OUTPUT); digitalWrite(US_VCC, HIGH);
  pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT);
  delay(50);

  arm.attach(SERVO_PIN);
  arm.write(ARM_REST);

  Serial.begin(9600);
  if (DEBUG_MODE) {
    Serial.print("DEBUG ("); Serial.print(TRICYCLE ? "tricycle" : "4-wheel");
    Serial.println("): l/L r/R f/b t s d 0-9");
  } else {
    runMission();
  }
}

void loop() {
  if (!DEBUG_MODE) return;       // mission runs once from setup()
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') handleCommand(c);
  }
}
