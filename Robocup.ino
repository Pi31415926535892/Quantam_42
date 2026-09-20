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

// Start with P only
float Kp = 0.12;
float Ki = 0.00;
float Kd = 0.00;

int baseSpeed = 150;

float integral = 0;
float previousError = 0;

const float INTEGRAL_LIMIT = 3000;

// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotor(int leftSpeed, int rightSpeed)
{
  // ----------------------------------------------------------
  // FORCE BOTH MOTORS TO FORWARD
  // ----------------------------------------------------------

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);

  // ----------------------------------------------------------
  // NEVER ALLOW REVERSE
  // ----------------------------------------------------------

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

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

  // Motors OFF
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // ----------------------------------------------------------
  // QTR
  // ----------------------------------------------------------

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  Serial.println("================================");
  Serial.println("QTR LINE FOLLOWER");
  Serial.println("================================");

  Serial.println("Move sensor across WHITE and BLACK.");
  Serial.println("Calibration starts in 2 seconds.");

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
  // ----------------------------------------------------------
  // READ QTR
  // ----------------------------------------------------------

  uint16_t position = qtr.readLineBlack(sensorValues);

  // ----------------------------------------------------------
  // ERROR
  // ----------------------------------------------------------

  float error = (float)position - POSITION_CENTER;

  // ----------------------------------------------------------
  // PID
  // ----------------------------------------------------------

  integral += error;

  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  float derivative = error - previousError;

  float correction =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  previousError = error;

  // ----------------------------------------------------------
  // MOTOR SPEED
  // ----------------------------------------------------------

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // ABSOLUTELY NO REVERSE
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotor(leftSpeed, rightSpeed);

  // ----------------------------------------------------------
  // DEBUG
  // ----------------------------------------------------------

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 100)
  {
    lastPrint = millis();

    Serial.print("Sensors: ");

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);

      if (i < SensorCount - 1)
        Serial.print(",");
    }

    Serial.print(" | POS: ");
    Serial.print(position);

    Serial.print(" | ERROR: ");
    Serial.print(error);

    Serial.print(" | CORRECTION: ");
    Serial.print(correction);

    Serial.print(" | L: ");
    Serial.print(leftSpeed);

    Serial.print(" | R: ");
    Serial.println(rightSpeed);
  }
}
