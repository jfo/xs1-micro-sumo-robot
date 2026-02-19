/* JSumo Micro Sumo Robot Code
 * Model Number: XS1
 * Board Model: XMotion Micro
 *
 * Notes:
 * - Opponent sensors (JS40F) return HIGH (1) when an opponent is seen.
 * - Line sensor (ML1) returns LOW (0) when white line is seen.
 * - Buttons return LOW (0) when pressed (INPUT_PULLUP).
 * - LEDs are active HIGH.
 */

// sensors
#define Right_Op_Sensor A0
#define Front_Op_Sensor A1
#define Left_Op_Sensor  A2
#define Line_Sensor     A3

// user buttons and LEDs
#define Front_Button 8
#define Back_Button  12
#define Led1         4
#define Led2         6

// motor control pins
#define Right_Motor_Direction 10
#define Right_Motor_Speed     9
#define Left_Motor_Direction  13
#define Left_Motor_Speed      5

// Configuration constants
const int topSpeed    = 230;
const int kDefaultSpeed    = 200;
const int kLineThreshold   = 45;
const int kDebounceDelayMs = 15;
const int kRetreatDelayMs  = 300;
const int kTurnDelayMs     = 150;
const int kMinTurnDelayMs  = 100;
const int kMaxTurnDelayMs  = 200;
const unsigned long kEdgeTimeoutMs = 5000;
const int kFlipTriggerCount = 3;
const unsigned long kFlipWindowMs = 1000;
const int kFlipRecoveryBackMs = 600;
const int kFlipRecoverySpeed = 255;
const int kLedBlinkCount   = 5;
const int kLedBlinkDelayMs = 100;
const int kAccelStepDelayMs = 1;
const int kMaxSpeedHoldSeconds = 2;

// Current runtime settings (tweakable)
int baseSpeed = kDefaultSpeed;

// stores last sensor direction seen: 0=left, 1=front, 2=right
int lastDirection = 1;
bool hasAccelerated = false;
bool hasStarted = false;
unsigned long lastEdgeMillis = 0;
unsigned long flipWindowStartMs = 0;
int flipTriggerCount = 0;

// Direction constants
enum Direction {
  DIR_LEFT  = 0,
  DIR_FRONT = 1,
  DIR_RIGHT = 2,
};

// Forward declarations
void setMotors(int leftMotorValue, int rightMotorValue);
void stopMotors();
void printSensorValues();
void waitForStart();
void preMatchBlink();
bool handleLineSensor();
bool processOpponentSensors();
bool handleEdgeTimeout();
void actOnLastDirection();
void setLeds(bool led1On, bool led2On);
void quadraticAcceleration();

// Set motor outputs. Note: keeps original mapping (LeftMotorValue maps to right motor pins)
void setMotors(int LeftMotorValue, int RightMotorValue) {
  if (LeftMotorValue < 0) {
    LeftMotorValue = abs(LeftMotorValue);
    digitalWrite(Right_Motor_Direction, LOW);
    analogWrite(Right_Motor_Speed, LeftMotorValue);
  } else if (LeftMotorValue > 0) {
    digitalWrite(Right_Motor_Direction, HIGH);
    analogWrite(Right_Motor_Speed, 255 - LeftMotorValue);
  } else {
    digitalWrite(Right_Motor_Direction, HIGH);
    analogWrite(Right_Motor_Speed, 255);
  }

  if (RightMotorValue < 0) {
    RightMotorValue = abs(RightMotorValue);
    digitalWrite(Left_Motor_Direction, LOW);
    analogWrite(Left_Motor_Speed, RightMotorValue);
  } else if (RightMotorValue > 0) {
    digitalWrite(Left_Motor_Direction, HIGH);
    analogWrite(Left_Motor_Speed, 255 - RightMotorValue);
  } else {
    digitalWrite(Left_Motor_Direction, HIGH);
    analogWrite(Left_Motor_Speed, 255);
  }
}

void stopMotors() {
  digitalWrite(Left_Motor_Direction, HIGH);
  analogWrite(Left_Motor_Speed, 255);
  digitalWrite(Right_Motor_Direction, HIGH);
  analogWrite(Right_Motor_Speed, 255);
}

void setup() {
  pinMode(Back_Button, INPUT_PULLUP);
  pinMode(Front_Button, INPUT_PULLUP);
  pinMode(Front_Op_Sensor, INPUT);
  pinMode(Left_Op_Sensor, INPUT);
  pinMode(Right_Op_Sensor, INPUT);
  pinMode(Line_Sensor, INPUT);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Left_Motor_Direction, OUTPUT);
  pinMode(Right_Motor_Direction, OUTPUT);
  pinMode(Left_Motor_Speed, OUTPUT);
  pinMode(Right_Motor_Speed, OUTPUT);
  Serial.begin(9600);
}

// Print current sensor values to Serial
void printSensorValues() {
  Serial.print("Line_Sensor:");
  Serial.print(analogRead(Line_Sensor));
  Serial.print(",");
  Serial.print("Front_Op_Sensor:");
  Serial.print(analogRead(Front_Op_Sensor));
  Serial.print(",");
  Serial.print("Right_Op_Sensor:");
  Serial.print(analogRead(Right_Op_Sensor));
  Serial.print(",");
  Serial.print("Left_Op_Sensor:");
  Serial.print(analogRead(Left_Op_Sensor));
  Serial.println();
}

// Wait for start button push while printing sensor values
void waitForStart() {
  while (digitalRead(Front_Button) == 1 && digitalRead(Back_Button) == 1) {
    printSensorValues();
    stopMotors();
  }
}

// LED helpers
void setLeds(bool led1On, bool led2On) {
  digitalWrite(Led1, led1On ? HIGH : LOW);
  digitalWrite(Led2, led2On ? HIGH : LOW);
}

// Blink LEDs during pre-match countdown
void preMatchBlink() {
  for (int x = 0; x < kLedBlinkCount; x++) {
    if (x % 2 == 0)
      setLeds(true, false);
    else
      setLeds(false, true);
    delay(kLedBlinkDelayMs);
  }
}

void quadraticAcceleration() {
  for (int step = 100; step <= kDefaultSpeed; step++) {
    long scaled = static_cast<long>(step) * step;
    int speed = static_cast<int>(scaled / kDefaultSpeed);
    setMotors(speed, speed);
    delay(kAccelStepDelayMs);
  }
  // delay(kMaxSpeedHoldSeconds * 1000L);
}

bool handleLineSensor() {
  if (analogRead(Line_Sensor) < kLineThreshold) {
    delay(kDebounceDelayMs);
    if (analogRead(Line_Sensor) < kLineThreshold) {
      lastEdgeMillis = millis();
      unsigned long now = millis();
      if (now - flipWindowStartMs > kFlipWindowMs) {
        flipWindowStartMs = now;
        flipTriggerCount = 0;
      }
      flipTriggerCount++;
      if (flipTriggerCount >= kFlipTriggerCount) {
        flipTriggerCount = 0;
        setMotors(0,0);
        delay(kFlipRecoveryBackMs);
        setMotors(-kFlipRecoverySpeed, -kFlipRecoverySpeed);
        delay(kFlipRecoveryBackMs);
        setMotors(baseSpeed, baseSpeed);
        return true;
      }
      baseSpeed = random(100, 201);
      setMotors(-baseSpeed, -baseSpeed);
      delay(kRetreatDelayMs);
      int turnDelayMs = random(kMinTurnDelayMs, kMaxTurnDelayMs + 1);
      setMotors(-baseSpeed, baseSpeed);
      delay(turnDelayMs);
      setMotors(baseSpeed, baseSpeed);
      return true;
    }
  }
  return false;
}

bool handleEdgeTimeout() {
  if (millis() - lastEdgeMillis >= kEdgeTimeoutMs) {
    lastEdgeMillis = millis();
    baseSpeed = random(100, 201);
    setMotors(-baseSpeed, -baseSpeed);
    delay(kRetreatDelayMs);
    int turnDelayMs = random(kMinTurnDelayMs, kMaxTurnDelayMs + 1);
    bool turnLeft = random(0, 2) == 0;
    if (turnLeft)
      setMotors(-baseSpeed, baseSpeed);
    else
      setMotors(baseSpeed, -baseSpeed);
    delay(turnDelayMs);
    setMotors(baseSpeed, baseSpeed);
    return true;
  }
  return false;
}

bool processOpponentSensors() {
  if (digitalRead(Front_Op_Sensor) == 1) {
    setMotors(topSpeed, topSpeed);
    return true;
  }
  if (digitalRead(Left_Op_Sensor) == 1 && digitalRead(Right_Op_Sensor) == 0) {
    setMotors(-baseSpeed, baseSpeed);
    return true;
  }
  if (digitalRead(Left_Op_Sensor) == 0 && digitalRead(Right_Op_Sensor) == 1) {
    setMotors(baseSpeed, -baseSpeed);
    return true;
  }
  return false;
}

void loop() {
  if (!hasStarted) {
    waitForStart();
    hasStarted = true;
    preMatchBlink();
  }

  if (!hasAccelerated) {
    quadraticAcceleration();
    hasAccelerated = true;
  }

  if (lastEdgeMillis == 0) lastEdgeMillis = millis();

  while (true) {
    if (handleLineSensor()) continue;
    if (handleEdgeTimeout()) continue;
    if (processOpponentSensors()) continue;
  }
}
