#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ============================================================
// QTR-8A
// ============================================================

const uint8_t sensorPins[SensorCount] = {
  A0, A2, A4, A6,
  A8, A10, A12, A14
};

// ============================================================
// L298N
// ============================================================

const uint8_t LEFT_IN1 = 3;
const uint8_t LEFT_IN2 = 5;
const uint8_t LEFT_PWM = 7;

const uint8_t RIGHT_IN1 = 2;
const uint8_t RIGHT_IN2 = 4;
const uint8_t RIGHT_PWM = 6;

// ============================================================
// LINE
// ============================================================

const int POSITION_CENTER = 3500;

// ============================================================
// SPEED
// ============================================================

const int BASE_SPEED = 120;
const int MIN_SPEED = 55;
const int MAX_SPEED = 200;

const int MAX_CORRECTION = 65;

// ============================================================
// PID
// ============================================================

float Kp = -1;
float Ki = 0.0;
float Kd = -0.015;

float previousError = 0;
float integral = 0;

const float INTEGRAL_LIMIT = 2000;

// ============================================================
// LINE LOST DETECTION
// ============================================================

// If all sensors are below this value,
// we consider the line lost.
const uint16_t LINE_THRESHOLD = 150;

// Remember which direction the line was last seen.
int lastDirection = 0;

// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotor(int leftSpeed, int rightSpeed)
{
  // ----------------------------------------------------------
  // BOTH MOTORS ALWAYS FORWARD
  // ----------------------------------------------------------

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);

  // ----------------------------------------------------------
  // SAFETY LIMITS
  // ----------------------------------------------------------

  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // ----------------------------------------------------------
  // QTR
  // ----------------------------------------------------------

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  Serial.println();
  Serial.println("=================================");
  Serial.println("      QTR-8A LINE FOLLOWER");
  Serial.println("=================================");

  Serial.println("Move sensors across WHITE + BLACK.");
  Serial.println("Calibration begins in 2 seconds.");

  delay(2000);

  // ----------------------------------------------------------
  // CALIBRATION
  // ----------------------------------------------------------

  unsigned long startTime = millis();

  while (millis() - startTime < 10000)
  {
    qtr.calibrate();
    delay(20);
  }

  Serial.println("Calibration complete!");
  Serial.println("Starting in 2 seconds...");

  delay(2000);

  Serial.println("GO!");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // READ SENSORS
  // ==========================================================

  uint16_t position = qtr.readLineBlack(sensorValues);

  // ==========================================================
  // CHECK WHETHER LINE EXISTS
  // ==========================================================

  uint32_t sensorTotal = 0;

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    sensorTotal += sensorValues[i];
  }

  // ----------------------------------------------------------
  // LINE LOST
  // ----------------------------------------------------------

  if (sensorTotal < LINE_THRESHOLD * SensorCount)
  {
    // Slowly turn in the direction where the line
    // was last detected.

    if (lastDirection < 0)
    {
      // Last saw line on LEFT
      setMotor(110, 70);
    }
    else if (lastDirection > 0)
    {
      // Last saw line on RIGHT
      setMotor(70, 110);
    }
    else
    {
      // Don't know where it went
      setMotor(BASE_SPEED, BASE_SPEED);
    }

    return;
  }

  // ==========================================================
  // CALCULATE ERROR
  // ==========================================================

  float error = (float)position - POSITION_CENTER;

  // Remember direction
  if (error < -200)
  {
    lastDirection = -1;
  }
  else if (error > 200)
  {
    lastDirection = 1;
  }

  // ==========================================================
  // PID
  // ==========================================================

  integral += error;

  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  float derivative = error - previousError;

  float correction =
      (Kp * error) +
      (Ki * integral) +
      (Kd * derivative);

  previousError = error;

  // ==========================================================
  // LIMIT CORRECTION
  // ==========================================================

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  // ==========================================================
  // MOTOR SPEEDS
  // ==========================================================

  int leftSpeed =
      BASE_SPEED + correction;

  int rightSpeed =
      BASE_SPEED - correction;

  // ==========================================================
  // FORWARD ONLY
  // ==========================================================

  leftSpeed = constrain(
    leftSpeed,
    MIN_SPEED,
    MAX_SPEED
  );

  rightSpeed = constrain(
    rightSpeed,
    MIN_SPEED,
    MAX_SPEED
  );

  // ==========================================================
  // DRIVE
  // ==========================================================

  setMotor(
    leftSpeed,
    rightSpeed
  );

  // ==========================================================
  // DEBUG
  // ==========================================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 150)
  {
    lastPrint = millis();

    Serial.print("POS=");
    Serial.print(position);

    Serial.print(" ERROR=");
    Serial.print(error);

    Serial.print(" CORR=");
    Serial.print(correction);

    Serial.print(" L=");
    Serial.print(leftSpeed);

    Serial.print(" R=");
    Serial.print(rightSpeed);

    Serial.print(" TOTAL=");
    Serial.println(sensorTotal);
  }
}
