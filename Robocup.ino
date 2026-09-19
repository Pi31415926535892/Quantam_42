#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ============================================================
// QTR-8A
// ============================================================

const uint8_t sensorPins[SensorCount] = {
  PA_0, PA_1, PA_2, PA_3,
  PA_4, PA_5, PA_6, PA_7
};

// ============================================================
// L298N
// ============================================================

// Left motor
const uint8_t LEFT_IN1 = PB_10;
const uint8_t LEFT_IN2 = PB_9;
const uint8_t LEFT_PWM = PB_4;

// Right motor
const uint8_t RIGHT_IN1 = PB_12;
const uint8_t RIGHT_IN2 = PB_13;
const uint8_t RIGHT_PWM = PB_8;

// ============================================================
// LINE POSITION
// ============================================================

const int POSITION_CENTER = 3500;

const uint8_t TRAIL_SIZE = 5;
uint16_t positionHistory[TRAIL_SIZE];
uint8_t historyIndex = 0;

// ============================================================
// PID CONTROLLER
// ============================================================

float Kp = 0.27;
float Ki = 0.00;
float Kd = 1.73;

int baseSpeed = 192;

float integral = 0;

const float INTEGRAL_LIMIT = 5000;

float previousError = 0;

// ============================================================
// ASCII SENSOR DISPLAY
// ============================================================

// Converts a QTR reading into:
// " "  = very low
// "░"  = low
// "▒"  = medium-low
// "▓"  = medium-high
// "█"  = high

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

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // ----------------------------------------------------------
  // QTR setup
  // ----------------------------------------------------------

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  Serial.println("QTR-8A CALIBRATION");
  Serial.println("Move the sensor across WHITE and BLACK!");
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
  // Initialize trailing average
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
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

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

  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // READ QTR SENSORS
  // ==========================================================

  // The QTR library fills sensorValues[].
  // No analogRead() is needed.

  qtr.readCalibrated(sensorValues);

  // ==========================================================
  // NUMERICAL SENSOR VALUES
  // ==========================================================

  Serial.print("NUM: ");

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);

    if (i < SensorCount - 1)
      Serial.print(",");
  }

  Serial.println();

  // ==========================================================
  // ASCII SENSOR DISPLAY
  // ==========================================================

  Serial.print("ASCII: ");

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorToASCII(sensorValues[i]));
  }

  Serial.println();

  // ==========================================================
  // READ LINE POSITION
  // ==========================================================

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

  int correction = P + I + D;

  previousError = error;

  // ==========================================================
  // MOTOR SPEEDS
  // ==========================================================

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotor(leftSpeed, rightSpeed);

  // ==========================================================
  // POSITION / PID DEBUG
  // ==========================================================

  Serial.print("POS: ");
  Serial.print(position);

  Serial.print("  AVG: ");
  Serial.print(averagedPosition);

  Serial.print("  ERR: ");
  Serial.print(error);

  Serial.print("  P: ");
  Serial.print(P);

  Serial.print("  I: ");
  Serial.print(I);

  Serial.print("  D: ");
  Serial.print(D);

  Serial.print("  L: ");
  Serial.print(leftSpeed);

  Serial.print("  R: ");
  Serial.print(rightSpeed);
  Serial.print("    ");

  Serial.println();

  delay(10);
}