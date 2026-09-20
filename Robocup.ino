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

// LEFT MOTOR
const uint8_t LEFT_IN1 = 3;
const uint8_t LEFT_IN2 = 5;
const uint8_t LEFT_PWM = 7;

// RIGHT MOTOR
const uint8_t RIGHT_IN1 = 2;
const uint8_t RIGHT_IN2 = 4;
const uint8_t RIGHT_PWM = 6;

// ============================================================
// LINE POSITION
// ============================================================

const int POSITION_CENTER = 3500;

// ============================================================
// PID
// ============================================================

float Kp = -0.08;
float Ki = 0.00;
float Kd = 0.00;

int baseSpeed = 150;

float integral = 0;
float previousError = 0;

const float INTEGRAL_LIMIT = 3000;

// Maximum steering correction
const int MAX_CORRECTION = 120;

// Minimum motor speed
const int MIN_SPEED = 30;

// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotor(int leftSpeed, int rightSpeed)
{
  // ----------------------------------------------------------
  // BOTH MOTORS ARE ALWAYS FORWARD
  // ----------------------------------------------------------

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);

  // ----------------------------------------------------------
  // ABSOLUTELY NO REVERSE
  // ----------------------------------------------------------

  leftSpeed = constrain(leftSpeed, MIN_SPEED, 255);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);

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

  // Motors OFF during startup
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // ----------------------------------------------------------
  // QTR-8A SETUP
  // ----------------------------------------------------------

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  Serial.println();
  Serial.println("====================================");
  Serial.println("      QTR-8A LINE FOLLOWER");
  Serial.println("====================================");

  Serial.println();
  Serial.println("Move the QTR sensor across");
  Serial.println("both WHITE and BLACK.");
  Serial.println();

  Serial.println("Calibration starts in 2 seconds...");

  delay(2000);

  // ----------------------------------------------------------
  // 10 SECOND CALIBRATION
  // ----------------------------------------------------------

  unsigned long calibrationStart = millis();

  while (millis() - calibrationStart < 10000)
  {
    qtr.calibrate();
    delay(20);
  }

  Serial.println();
  Serial.println("Calibration complete!");

  Serial.println("Starting in 2 seconds...");

  delay(2000);

  Serial.println("GO!");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // READ QTR
  // ==========================================================

  uint16_t position = qtr.readLineBlack(sensorValues);

  // ==========================================================
  // ERROR
  // ==========================================================

  float error = (float)position - POSITION_CENTER;

  // ==========================================================
  // INTEGRAL
  // ==========================================================

  integral += error;

  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  // ==========================================================
  // DERIVATIVE
  // ==========================================================

  float derivative = error - previousError;

  // ==========================================================
  // PID
  // ==========================================================

  float P = Kp * error;
  float I = Ki * integral;
  float D = Kd * derivative;

  int correction = (int)(P + I + D);

  // Limit steering correction
  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  previousError = error;

  // ==========================================================
  // MOTOR SPEEDS
  // ==========================================================

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // ==========================================================
  // FORWARD ONLY
  // ==========================================================

  leftSpeed = constrain(
    leftSpeed,
    MIN_SPEED,
    255
  );

  rightSpeed = constrain(
    rightSpeed,
    MIN_SPEED,
    255
  );

  // ==========================================================
  // DRIVE
  // ==========================================================

  setMotor(leftSpeed, rightSpeed);

  // ==========================================================
  // SERIAL DEBUG
  // ==========================================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 100)
  {
    lastPrint = millis();

    Serial.print("SENSORS: ");

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);

      if (i < SensorCount - 1)
        Serial.print(",");
    }

    Serial.println();

    Serial.print("POSITION: ");
    Serial.print(position);

    Serial.print(" | ERROR: ");
    Serial.print(error);

    Serial.print(" | CORRECTION: ");
    Serial.print(correction);

    Serial.print(" | LEFT: ");
    Serial.print(leftSpeed);

    Serial.print(" | RIGHT: ");
    Serial.println(rightSpeed);

    Serial.println();
  }
}
