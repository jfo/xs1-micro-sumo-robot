/*
 * JSumo Micro Sumo Robot Code
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
#define Front_Button    8
#define Back_Button     12
#define Led1            4
#define Led2            6

// motor control pins
#define Right_Motor_Direction 10
#define Right_Motor_Speed     9
#define Left_Motor_Direction  13
#define Left_Motor_Speed      5

// speed setting (tune for robot performance)
int Speed2 = 200;

// stores last sensor direction seen: 0=left, 1=front, 2=right
int Last_Value = 1;

void Motor(int LeftMotorValue, int RightMotorValue) {
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

void MotorStop() {
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
  pinMode(Left_Motor_Direction, OUTPUT);
  pinMode(Right_Motor_Direction, OUTPUT);
  pinMode(Left_Motor_Speed, OUTPUT);
  pinMode(Right_Motor_Speed, OUTPUT);
}

/////////////////////////////////
//// Main Robot Code Routine ////
/////////////////////////////////
void loop() {
  // while waiting for a start button push
  while(digitalRead(Front_Button) == 1 && digitalRead(Back_Button) == 1) {
    // sensor calidation routine, you can see the values of the sensors in the serial monitor 
    // to understand how they work and what values they give when they see something
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
    MotorStop();
  }

  // depending on which button was pressed, set Last_Value to prefer turning
  if (digitalRead(Front_Button) == 0) Last_Value = 2;
  if (digitalRead(Back_Button) == 0) Last_Value = 0;

  // 5-second countdown flashing LEDs (500ms x 10)
  for (int x = 0; x < 10; x++) {
    if (x % 2 == 0) {
      digitalWrite(Led1, HIGH);
      digitalWrite(Led2, LOW);
    } else {
      digitalWrite(Led1, LOW);
      digitalWrite(Led2, HIGH);
    }
    delay(500);
  }

  // Move forward
  Motor(Speed2, Speed2);

  while (digitalRead(Front_Button) == 1 && digitalRead(Back_Button) == 1) {
    // line sensor: retreat and turn if line detected
    if (analogRead(Line_Sensor) < 45) {
      delay(15); // helps ensure that we don't react to thin scratches
      if (analogRead(Line_Sensor) < 45) {
        Motor(-Speed2, -Speed2);
        delay(150); // 150ms retreat
        Motor(-Speed2, Speed2);
        delay(200); // 200ms turning
        Motor(Speed2, Speed2);
      }
    }
    // Opponent sensor handling
    else if (digitalRead(Front_Op_Sensor) == 1) {
      // Front sensor sees opponent: full forward
      Motor(Speed2, Speed2);
      Last_Value = 1;
    } else if (digitalRead(Left_Op_Sensor) == 1 && digitalRead(Right_Op_Sensor) == 0) {
      // Left sensor sees opponent: turn left
      Motor(-Speed2, Speed2);
      Last_Value = 0;
    } else if (digitalRead(Left_Op_Sensor) == 0 && digitalRead(Right_Op_Sensor) == 1) {
      // Right sensor sees opponent: turn right
      Motor(Speed2, -Speed2);
      Last_Value = 2;
    }
    // No sensor data: act based on last seen direction
    else if (Last_Value == 0) Motor(-Speed2, Speed2);
    else if (Last_Value == 1) Motor(Speed2, Speed2);
    else if (Last_Value == 2) Motor(Speed2, -Speed2);
  }

  // in case button is pressed during match, stop and wait
  delay(100);
  while (digitalRead(Front_Button) == 0 || digitalRead(Back_Button) == 0)
    ;
  delay(100);
}
