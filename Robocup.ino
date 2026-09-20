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

// Left motor
const uint8_t LEFT_IN1 = 3;
const uint8_t LEFT_IN2 = 5;
const uint8_t LEFT_PWM = 7;

// Right motor
const uint8_t RIGHT_IN1 = 2;
const uint8_t RIGHT_IN2 = 4;
const uint8_t RIGHT_PWM = 6;

// ============================================================
// LINE POSITION
// ============================================================

const int POSITION_CENTER = 3500;

// Start with NO averaging.
// Once everything works, you can increase this to 3 or 5.
const uint8_t TRAIL_SIZE = 1;

uint16_t positionHistory[TRAIL_SIZE];
uint8_t historyIndex = 0;

// ============================================================
// PID CONTROLLER
// ============================================================

// Start with P only.
// If the robot turns the WRONG direction, change Kp to -0.15.
float Kp = -0.15;
float Ki = 0.00;
float Kd = 0.00;

int baseSpeed = 150;

float integral = 0;
float previousError = 0;

const float INTEGRAL_LIMIT = 5000;

// ============================================================
// ASCII SENSOR DISPLAY
// ============================================================

char sensorToASCII(uint16_t value)
{
  if (value < 200)
    return ' ';

  else if (value < 400)
    return '.';

  else if (value < 600)
    return '+';

  else if (value < 800)
    return '#';

  else
    return '@';
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  // ----------------------------------------------------------
  // Motor pins
  // ----------------------------------------------------------

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // Stop motors
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // ----------------------------------------------------------
  // QTR setup
  // ----------------------------------------------------------

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  Serial.println();
  Serial.println("================================");
  Serial.println("QTR-8A LINE FOLLOWER");
  Serial.println("================================");

  Serial.println("Move the sensor across WHITE and BLACK.");
  Serial.println("Calibration starts in 2 seconds...");

  delay(2000);

  // ----------------------------------------------------------
  // 10 second calibration
  // ----------------------------------------------------------

  unsigned long startTime = millis();

  while (millis() - startTime < 10000)
  {
    qtr.calibrate();
    delay(20);
  }

  // ----------------------------------------------------------
  // Initialize position history
  // ----------------------------------------------------------

  for (uint8_t i = 0; i < TRAIL_SIZE; i++)
  {
    positionHistory[i] = POSITION_CENTER;
  }

  Serial.println();
  Serial.println("Calibration complete!");
  Serial.println("Starting in 2 seconds...");

  delay(2000);

  Serial.println("GO!");
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotor(int leftSpeed, int rightSpeed)
{
  // ----------------------------------------------------------
  // LEFT MOTOR
  // ----------------------------------------------------------

  if (leftSpeed >= 0)
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  }
  else
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);

    leftSpeed = -leftSpeed;
  }

  // ----------------------------------------------------------
  // RIGHT MOTOR
  // ----------------------------------------------------------

  if (rightSpeed >= 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  }
  else
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);

    rightSpeed = -rightSpeed;
  }

  // ----------------------------------------------------------
  // Limit PWM AFTER checking direction
  // ----------------------------------------------------------

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // READ LINE POSITION
  // ==========================================================

  // readLineBlack() also fills sensorValues[]
  uint16_t position = qtr.readLineBlack(sensorValues);

  // ==========================================================
  // TRAILING AVERAGE
  // ==========================================================

  positionHistory[historyIndex] = position;

  historyIndex++;

  if (historyIndex >= TRAIL_SIZE)
    historyIndex = 0;

  uint32_t total = 0;

  for (uint8_t i = 0; i < TRAIL_SIZE; i++)
  {
    total += positionHistory[i];
  }

  uint16_t averagedPosition = total / TRAIL_SIZE;

  // ==========================================================
  // ERROR
  // ==========================================================

  float error = (float)averagedPosition - POSITION_CENTER;

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

  float P = Kp * error;
  float I = Ki * integral;
  float D = Kd * derivative;

  int correction = (int)(P + I + D);

  previousError = error;

  // ==========================================================
  // MOTOR SPEEDS
  // ==========================================================

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotor(leftSpeed, rightSpeed);

  // ==========================================================
  // SERIAL DEBUG
  // ==========================================================

  static unsigned long lastPrint = 0;

  // Only print every 100 ms so Serial doesn't slow the robot
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

    Serial.print("ASCII: ");

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorToASCII(sensorValues[i]));
    }

    Serial.println();

    Serial.print("POS: ");
    Serial.print(position);

    Serial.print("  AVG: ");
    Serial.print(averagedPosition);

    Serial.print("  ERR: ");
    Serial.print(error);

    Serial.print("  P: ");
    Serial.print(P);

    Serial.print("  L: ");
    Serial.print(leftSpeed);

    Serial.print("  R: ");
    Serial.print(rightSpeed);

    Serial.println();
  }
}
