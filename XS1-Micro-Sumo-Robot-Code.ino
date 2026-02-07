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
const int kDefaultSpeed    = 200;
const int kLineThreshold   = 45;
const int kDebounceDelayMs = 15;
const int kRetreatDelayMs  = 150;
const int kTurnDelayMs     = 200;
const int kLedBlinkCount   = 10;
const int kLedBlinkDelayMs = 500;

// Current runtime settings (tweakable)
int baseSpeed = kDefaultSpeed;

// stores last sensor direction seen: 0=left, 1=front, 2=right
int lastDirection = 1;

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
void actOnLastDirection();
void setLeds(bool led1On, bool led2On);

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

// Handle line sensor reading; returns true if line was detected and handled
bool handleLineSensor() {
  if (analogRead(Line_Sensor) < kLineThreshold) {
    delay(kDebounceDelayMs);
    if (analogRead(Line_Sensor) < kLineThreshold) {
      setMotors(-baseSpeed, -baseSpeed);
      delay(kRetreatDelayMs);
      setMotors(-baseSpeed, baseSpeed);
      delay(kTurnDelayMs);
      setMotors(baseSpeed, baseSpeed);
      return true;
    }
  }
  return false;
}

// Process opponent sensors; returns true if any opponent sensor fired and action taken
bool processOpponentSensors() {
  if (digitalRead(Front_Op_Sensor) == 1) {
    setMotors(baseSpeed, baseSpeed);
    lastDirection = DIR_FRONT;
    return true;
  }
  if (digitalRead(Left_Op_Sensor) == 1 && digitalRead(Right_Op_Sensor) == 0) {
    setMotors(-baseSpeed, baseSpeed);
    lastDirection = DIR_LEFT;
    return true;
  }
  if (digitalRead(Left_Op_Sensor) == 0 && digitalRead(Right_Op_Sensor) == 1) {
    setMotors(baseSpeed, -baseSpeed);
    lastDirection = DIR_RIGHT;
    return true;
  }
  return false;
}

void actOnLastDirection() {
  if (lastDirection == DIR_LEFT)
    setMotors(-baseSpeed, baseSpeed);
  else if (lastDirection == DIR_FRONT)
    setMotors(baseSpeed, baseSpeed);
  else if (lastDirection == DIR_RIGHT)
    setMotors(baseSpeed, -baseSpeed);
}

/////////////////////////////////
//// Main Robot Code Routine ////
/////////////////////////////////
void loop() {
  waitForStart();

  // depending on which button was pressed, set preferred turning direction
  if (digitalRead(Front_Button) == 0) lastDirection = DIR_RIGHT;
  if (digitalRead(Back_Button) == 0) lastDirection = DIR_LEFT;

  // pre-match LED countdown
  preMatchBlink();

  // Move forward
  setMotors(baseSpeed, baseSpeed);

  while (digitalRead(Front_Button) == 1 && digitalRead(Back_Button) == 1) {
    // Line handling has highest priority
    if (handleLineSensor()) continue;

    // Opponent sensors
    if (processOpponentSensors()) continue;

    // Otherwise act on last remembered direction
    actOnLastDirection();
  }

  // in case button is pressed during match, stop and wait
  stopMotors();
  delay(100);
  while (digitalRead(Front_Button) == 0 || digitalRead(Back_Button) == 0);
  delay(100);
}
