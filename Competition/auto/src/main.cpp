#include <Arduino.h>
#include <Servo.h>

const int LEFT_PWM_PIN = 9;
const int LEFT_IN1_PIN = 7;
const int LEFT_IN2_PIN = 8;
const int RIGHT_PWM_PIN = 10;
const int RIGHT_IN3_PIN = 5;
const int RIGHT_IN4_PIN = 6;

const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;

const int ULTRA_TRIG_PIN = 11;
const int ULTRA_ECHO_PIN = 12;

const int IR_LEFT_PIN = A0;
const int IR_RIGHT_PIN = A1;
const int SIDE_SELECT_PIN = A2;

const int SERVO_BASE_PIN = 4;
const int SERVO_GRIP_PIN = 13;

const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159f * WHEEL_DIAMETER_CM;
const long ENCODER_COUNTS_PER_REV = 360;
const float WHEEL_BASE_CM = 14.0;
const float COUNTS_PER_CM =
    (float)ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_CM;

const int DEFAULT_SPEED = 200;
const int TURN_SPEED = 180;
const int STOP_BRAKE_MS = 40;
const int AFTER_MOVE_SETTLE_MS = 40;

const int IR_FLOOR_THRESHOLD = 500;
const int BASE_TILT_UP_ANGLE = 90;
const int BASE_TILT_DOWN_ANGLE = 35;
const int GRIP_OPEN_ANGLE = 40;
const int GRIP_CLOSED_ANGLE = 120;
const unsigned long ULTRA_TIMEOUT_US = 25000UL;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
Servo servoBase;
Servo servoGrip;

typedef struct {
  float x, y;
  int heading;
  bool hasCube;
  bool hasPlaced;
} RobotState;

RobotState robot = {0, 0, 0, false, false};

const float CELL_CM = 20.0f;
int heading = 0;
bool cubePicked = false;
bool cubePlaced = false;
bool startOnLeftSide = true;

const float OPENING_DISTANCE_CM = 15.0f;
const float CUBE_DETECT_CM = 8.0f;
const int TARGET_IR_THRESHOLD = 600;
const int EXIT_IR_THRESHOLD = 700;

void placeCubeStructPtr(RobotState *s);
void (*robotAction)(RobotState *) = NULL;

void onLeftEncoder() { encoderLeftCount++; }
void onRightEncoder() { encoderRightCount++; }

long cmToCounts(float cm) { return (long)(cm * COUNTS_PER_CM); }

void setMotorRaw(int in1, int in2, int pwmPin, int pwm) {
  int mag = abs(pwm);
  if (mag > 255)
    mag = 255;
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, mag);
}

void stopMotors() {
  setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, 0);
  setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, 0);
}

void brakeMotors() {
  setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, -40);
  setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, -40);
  delay(STOP_BRAKE_MS);
  stopMotors();
}

void resetEncoders() {
  noInterrupts();
  encoderLeftCount = 0;
  encoderRightCount = 0;
  interrupts();
}

void driveStraightCm(float cm, int speed) {
  Serial.print("Driving forward: ");
  Serial.print(cm);
  Serial.println(" cm");
  resetEncoders();
  const long target = cmToCounts(cm);
  unsigned long t0 = millis();
  while (true) {
    noInterrupts();
    long l = encoderLeftCount;
    long r = encoderRightCount;
    interrupts();
    if (l >= target && r >= target)
      break;
    if (millis() - t0 > 4000) {
      Serial.println("Timeout during forward move");
      break;
    }

    long error = l - r;
    int adjust = (int)(error * 0.8f);
    int leftCmd = speed - adjust;
    int rightCmd = speed + adjust;
    leftCmd = constrain(leftCmd, -255, 255);
    rightCmd = constrain(rightCmd, -255, 255);
    setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, leftCmd);
    setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, rightCmd);
  }
  brakeMotors();
  delay(AFTER_MOVE_SETTLE_MS);
}

void driveBackwardCm(float cm, int speed) {
  Serial.print("Driving backward: ");
  Serial.print(cm);
  Serial.println(" cm");
  resetEncoders();
  const long target = cmToCounts(cm);
  unsigned long t0 = millis();
  while (true) {
    noInterrupts();
    long l = encoderLeftCount;
    long r = encoderRightCount;
    interrupts();
    if (l >= target && r >= target)
      break;
    if (millis() - t0 > 4000) {
      Serial.println("Timeout during backward move");
      break;
    }

    long error = l - r;
    int adjust = (int)(error * 0.8f);
    int leftCmd = -speed - adjust;
    int rightCmd = -speed + adjust;
    leftCmd = constrain(leftCmd, -255, 255);
    rightCmd = constrain(rightCmd, -255, 255);
    setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, leftCmd);
    setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, rightCmd);
  }
  brakeMotors();
  delay(AFTER_MOVE_SETTLE_MS);
}

void turnRightDegrees(float degrees) {
  Serial.print("Turning right ");
  Serial.print(degrees);
  Serial.println("°");
  float arcCm = 3.14159f * WHEEL_BASE_CM * (degrees / 360.0f);
  long target = cmToCounts(arcCm);
  resetEncoders();
  unsigned long t0 = millis();
  while (true) {
    noInterrupts();
    long l = encoderLeftCount;
    long r = -encoderRightCount;
    interrupts();
    if (l >= target && r >= target)
      break;
    if (millis() - t0 > 3000) {
      Serial.println("Timeout during right turn");
      break;
    }
    setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, TURN_SPEED);
    setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, -TURN_SPEED);
  }
  brakeMotors();
  delay(AFTER_MOVE_SETTLE_MS);
}

void turnLeftDegrees(float degrees) {
  Serial.print("Turning left ");
  Serial.print(degrees);
  Serial.println("°");
  float arcCm = 3.14159f * WHEEL_BASE_CM * (degrees / 360.0f);
  long target = cmToCounts(arcCm);
  resetEncoders();
  unsigned long t0 = millis();
  while (true) {
    noInterrupts();
    long l = -encoderLeftCount;
    long r = encoderRightCount;
    interrupts();
    if (l >= target && r >= target)
      break;
    if (millis() - t0 > 3000) {
      Serial.println("Timeout during left turn");
      break;
    }
    setMotorRaw(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, -TURN_SPEED);
    setMotorRaw(RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN, TURN_SPEED);
  }
  brakeMotors();
  delay(AFTER_MOVE_SETTLE_MS);
}

float readUltrasonicCm() {
  delay(30);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0)
    return 9999.0f;
  return duration * 0.0343f / 2.0f;
}

bool detectColoredArea(int threshold) {
  int irL = analogRead(IR_LEFT_PIN);
  int irR = analogRead(IR_RIGHT_PIN);
  Serial.print("IR L=");
  Serial.print(irL);
  Serial.print(" R=");
  Serial.println(irR);

  return (irL > threshold) && (irR > threshold);
}

void gripperOpen() {
  servoGrip.write(GRIP_OPEN_ANGLE);
  delay(140);
}
void gripperClose() {
  servoGrip.write(GRIP_CLOSED_ANGLE);
  delay(160);
}
void gripperTiltUp() {
  servoBase.write(BASE_TILT_UP_ANGLE);
  delay(160);
}
void gripperTiltDown() {
  servoBase.write(BASE_TILT_DOWN_ANGLE);
  delay(180);
}

void rotateToHeading(int targetHeading) {
  int diff = (targetHeading - heading + 4) % 4;
  if (diff == 1)
    turnRightDegrees(90);
  else if (diff == 2)
    turnRightDegrees(180);
  else if (diff == 3)
    turnLeftDegrees(90);
  heading = targetHeading;
}

void pickCube() {
  Serial.println("Picking cube...");
  for (int i = 0; i < 4; i++) {
    float d = readUltrasonicCm();
    if (d < 6.0f)
      break;
    driveStraightCm(2.0f, 160);
  }
  gripperTiltDown();
  gripperOpen();
  driveStraightCm(3.0f, 160);
  gripperClose();
  gripperTiltUp();
  cubePicked = true;
  Serial.println("Cube picked successfully!");
}

void placeCubeStructPtr(RobotState *s) {
  Serial.println("Placing cube...");
  gripperTiltDown();
  driveStraightCm(3.0f, 160);
  gripperOpen();
  driveBackwardCm(8.0f, 160);
  gripperTiltUp();
  s->hasPlaced = true;
  Serial.println("Cube placed successfully!");
}

float distanceAhead() {
  float d = readUltrasonicCm();
  Serial.print("Distance ahead: ");
  Serial.print(d);
  Serial.println(" cm");
  return d;
}
float distanceLeft() {
  int oldHeading = heading;
  turnLeftDegrees(90);
  float d = readUltrasonicCm();
  turnRightDegrees(90);
  heading = oldHeading;
  Serial.print("Distance left: ");
  Serial.print(d);
  Serial.println(" cm");
  return d;
}
float distanceRight() {
  int oldHeading = heading;
  turnRightDegrees(90);
  float d = readUltrasonicCm();
  turnLeftDegrees(90);
  heading = oldHeading;
  Serial.print("Distance right: ");
  Serial.print(d);
  Serial.println(" cm");
  return d;
}

bool aheadOpen() { return distanceAhead() > OPENING_DISTANCE_CM; }
bool leftOpen() { return distanceLeft() > OPENING_DISTANCE_CM; }
bool rightOpen() { return distanceRight() > OPENING_DISTANCE_CM; }

void moveOneCell() { driveStraightCm(CELL_CM, DEFAULT_SPEED); }

void wallFollowMission() {
  if (startOnLeftSide)
    robot.heading = 1;
  else
    robot.heading = 3;
  heading = robot.heading;

  gripperTiltUp();
  gripperOpen();

  robot.hasCube = false;
  robot.hasPlaced = false;
  robotAction = &placeCubeStructPtr;

  Serial.println("=== Starting Maze Mission ===");
  Serial.print("Start side: ");
  Serial.println(startOnLeftSide ? "LEFT (A)" : "RIGHT (B)");

  while (true) {
    if (!robot.hasCube) {
      float d = distanceAhead();
      if (d > 1.0f && d < CUBE_DETECT_CM + 1.0f) {
        pickCube();
        robot.hasCube = true;
        continue;
      }
    }

    if (robot.hasCube && !robot.hasPlaced) {
      if (detectColoredArea(TARGET_IR_THRESHOLD)) {
        if (robotAction)
          robotAction(&robot);
        cubePlaced = true;
        continue;
      }
    }

    if (robot.hasPlaced && detectColoredArea(EXIT_IR_THRESHOLD)) {
      Serial.println("Exit area detected! Mission complete.");
      stopMotors();
      return;
    }

    if (leftOpen()) {
      Serial.println("Path open left -> turning left");
      rotateToHeading((heading + 3) % 4);
      moveOneCell();
      continue;
    }
    if (aheadOpen()) {
      Serial.println("Path open ahead -> moving forward");
      moveOneCell();
      continue;
    }
    if (rightOpen()) {
      Serial.println("Path open right -> turning right");
      rotateToHeading((heading + 1) % 4);
      moveOneCell();
      continue;
    }

    Serial.println("Dead end -> turning around");
    rotateToHeading((heading + 2) % 4);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Robot initializing...");

  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_IN3_PIN, OUTPUT);
  pinMode(RIGHT_IN4_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  stopMotors();

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftEncoder,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightEncoder,
                  RISING);

  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  Immersive content pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(SIDE_SELECT_PIN, INPUT_PULLUP);

  startOnLeftSide = digitalRead(SIDE_SELECT_PIN) == HIGH;

  servoBase.attach(SERVO_BASE_PIN);
  servoGrip.attach(SERVO_GRIP_PIN);
  delay(500);
  gripperOpen();
  gripperTiltUp();

  Serial.println("Initialization complete. Ready!");
}

void loop() {
  static bool missionStarted = false;
  if (!missionStarted) {
    missionStarted = true;
    wallFollowMission();
  }

  while (true) {
    stopMotors();
    delay(500);
  }
}
