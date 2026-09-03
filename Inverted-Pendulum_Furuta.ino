#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const int PIN_STEP = 7;
const int PIN_DIR = 4;
const int PIN_ENABLE = 10;

const int PIN_MS1 = 8;
const int PIN_MS2 = 9;
const int PIN_MS3 = 11;

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEP_DIV = 16;
const long ENCODER_COUNTS_PER_REV = 2400L;

const float MOTOR_STEPS_PER_DEG =
  (MOTOR_STEPS_PER_REV * MICROSTEP_DIV) / 360.0f;

const float SETPOINT_DEG = 180.0f;

const float GUARD_HALF_WIDTH = 80.0f;

const float MAX_STEPPER_ABS_DEG = 360.0f;

const float MAX_CONTROL_SPEED_DEG_S = 2500.0f;

const float INTEGRAL_CLAMP = 2000.0f;

const unsigned long CONTROL_PERIOD_US = 2000UL;
const unsigned long STATUS_PERIOD_US = 100000UL;

const float DERIVATIVE_FILTER_ALPHA = 0.82f;

float Kp = 0.009f;
float Ki = 3.00f;
float Kd = 0.002f;
float Bias = 0.00f;

volatile long encoderCount = 0;

long encoderOffset = 0;

int encoderDir = 1;
int motorDir = 1;

bool calibrated = false;
bool controlEnabled = false;

float integral = 0.0f;
float previousAngle = 0.0f;
float filteredVelocity = 0.0f;

unsigned long lastControlUs = 0;
unsigned long lastStatusUs = 0;

String serialBuffer;

AccelStepper motor(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

long roundLong(float value) {
  if (value >= 0.0f)
    return (long)(value + 0.5f);

  return (long)(value - 0.5f);
}

long degToSteps(float deg) {
  return roundLong(deg * MOTOR_STEPS_PER_DEG);
}

float stepsToDeg(long steps) {
  return (float)steps / MOTOR_STEPS_PER_DEG;
}

float wrap360(float angle) {
  while (angle >= 360.0f)
    angle -= 360.0f;

  while (angle < 0.0f)
    angle += 360.0f;

  return angle;
}

float shortestAngleError(float setpoint, float position) {
  float error = setpoint - position;

  while (error > 180.0f)
    error -= 360.0f;

  while (error <= -180.0f)
    error += 360.0f;

  return error;
}

long getEncoderCount() {
  long value;

  noInterrupts();
  value = encoderCount;
  interrupts();

  return value;
}

float getEncoderAngle() {
  long raw = getEncoderCount();
  long relative = raw - encoderOffset;

  float angle =
    ((float)relative * 360.0f) /
    (float)ENCODER_COUNTS_PER_REV;

  angle *= (float)encoderDir;

  return wrap360(angle);
}

float getMotorAngle() {
  return stepsToDeg(motor.currentPosition());
}

void encoderA_ISR() {
  if (digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

void encoderB_ISR() {
  if (digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

void enableMotor() {
  digitalWrite(PIN_ENABLE, LOW);
}

void disableMotor() {
  digitalWrite(PIN_ENABLE, HIGH);
}

void resetControllerState(float angle) {
  integral = 0.0f;
  previousAngle = angle;
  filteredVelocity = 0.0f;
  lastControlUs = micros();
}

void stopControl(const char* reason) {
  controlEnabled = false;

  motor.setSpeed(0.0f);
  resetControllerState(getEncoderAngle());

  disableMotor();

  Serial.print("STOP: ");
  Serial.println(reason);
}

void startControl() {
  if (!calibrated) {
    Serial.println("ERRO: execute ZERO_DOWN ou SET_UP antes de START");
    return;
  }

  float angle = getEncoderAngle();
  float error = shortestAngleError(SETPOINT_DEG, angle);

  if (fabsf(error) > GUARD_HALF_WIDTH) {
    Serial.print("ERRO: haste fora da janela de inicio. Angulo=");
    Serial.println(angle, 2);
    return;
  }

  motor.setCurrentPosition(0);
  motor.setSpeed(0.0f);

  resetControllerState(angle);

  enableMotor();
  controlEnabled = true;

  Serial.println("START OK");
}

void setZeroDown() {
  stopControl("ZERO_DOWN");

  long raw = getEncoderCount();

  encoderOffset = raw;

  motor.setCurrentPosition(0);

  calibrated = true;

  Serial.println("ZERO_DOWN OK");
  Serial.println("ENCODER=0.00");
}

void setUp180() {
  stopControl("SET_UP");

  long raw = getEncoderCount();

  encoderOffset =
    raw - (long)(1200L * encoderDir);

  motor.setCurrentPosition(0);

  calibrated = true;

  Serial.println("SET_UP OK");
  Serial.println("ENCODER=180.00");
}

void printStatus() {
  float angle = getEncoderAngle();
  float error = shortestAngleError(SETPOINT_DEG, angle);

  Serial.println();
  Serial.println("STATUS");

  Serial.print("CONTROL=");
  Serial.println(controlEnabled ? "ON" : "OFF");

  Serial.print("CALIBRATED=");
  Serial.println(calibrated ? "YES" : "NO");

  Serial.print("ENCODER=");
  Serial.print(angle, 3);
  Serial.println(" deg");

  Serial.print("SETPOINT=");
  Serial.print(SETPOINT_DEG, 3);
  Serial.println(" deg");

  Serial.print("ERROR=");
  Serial.print(error, 3);
  Serial.println(" deg");

  Serial.print("ENC_VEL=");
  Serial.print(filteredVelocity, 2);
  Serial.println(" deg/s");

  Serial.print("MOTOR=");
  Serial.print(getMotorAngle(), 2);
  Serial.println(" deg");

  Serial.print("MOTOR_SPEED=");
  Serial.print(motor.speed(), 2);
  Serial.println(" step/s");

  Serial.print("KP=");
  Serial.println(Kp, 5);

  Serial.print("KI=");
  Serial.println(Ki, 5);

  Serial.print("KD=");
  Serial.println(Kd, 5);

  Serial.print("BIAS=");
  Serial.println(Bias, 5);

  Serial.print("ENC_DIR=");
  Serial.println(encoderDir);

  Serial.print("MOTOR_DIR=");
  Serial.println(motorDir);

  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("COMANDOS");
  Serial.println("ZERO_DOWN");
  Serial.println("SET_UP");
  Serial.println("START");
  Serial.println("STOP");
  Serial.println("STATUS");
  Serial.println("PID Kp Ki Kd Bias");
  Serial.println("KP valor");
  Serial.println("KI valor");
  Serial.println("KD valor");
  Serial.println("BIAS valor");
  Serial.println("ENC_DIR 1|-1");
  Serial.println("MOTOR_DIR 1|-1");
  Serial.println("HELP");
  Serial.println();
}

void processPIDCommand(String cmd) {
  char buffer[80];

  cmd.toCharArray(buffer, sizeof(buffer));

  char* token = strtok(buffer, " ");

  token = strtok(NULL, " ");
  if (!token) {
    Serial.println("ERRO: use PID Kp Ki Kd Bias");
    return;
  }

  float newKp = atof(token);

  token = strtok(NULL, " ");
  if (!token) {
    Serial.println("ERRO: use PID Kp Ki Kd Bias");
    return;
  }

  float newKi = atof(token);

  token = strtok(NULL, " ");
  if (!token) {
    Serial.println("ERRO: use PID Kp Ki Kd Bias");
    return;
  }

  float newKd = atof(token);

  token = strtok(NULL, " ");

  float newBias = 0.0f;

  if (token)
    newBias = atof(token);

  Kp = newKp;
  Ki = newKi;
  Kd = newKd;
  Bias = newBias;

  integral = 0.0f;

  Serial.print("PID=");
  Serial.print(Kp, 5);
  Serial.print(" ");
  Serial.print(Ki, 5);
  Serial.print(" ");
  Serial.print(Kd, 5);
  Serial.print(" ");
  Serial.println(Bias, 5);
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0)
    return;

  if (cmd == "ZERO_DOWN") {
    setZeroDown();
    return;
  }

  if (cmd == "SET_UP") {
    setUp180();
    return;
  }

  if (cmd == "START") {
    startControl();
    return;
  }

  if (cmd == "STOP") {
    stopControl("USER");
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  if (cmd == "HELP") {
    printHelp();
    return;
  }

  if (cmd.startsWith("PID ")) {
    processPIDCommand(cmd);
    return;
  }

  if (cmd.startsWith("KP ")) {
    Kp = cmd.substring(3).toFloat();
    integral = 0.0f;

    Serial.print("KP=");
    Serial.println(Kp, 5);

    return;
  }

  if (cmd.startsWith("KI ")) {
    Ki = cmd.substring(3).toFloat();
    integral = 0.0f;

    Serial.print("KI=");
    Serial.println(Ki, 5);

    return;
  }

  if (cmd.startsWith("KD ")) {
    Kd = cmd.substring(3).toFloat();

    Serial.print("KD=");
    Serial.println(Kd, 5);

    return;
  }

  if (cmd.startsWith("BIAS ")) {
    Bias = cmd.substring(5).toFloat();

    Serial.print("BIAS=");
    Serial.println(Bias, 5);

    return;
  }

  if (cmd.startsWith("ENC_DIR ")) {
    int value = cmd.substring(8).toInt();

    if (value == 1 || value == -1) {
      encoderDir = value;

      Serial.print("ENC_DIR=");
      Serial.println(encoderDir);
    } else {
      Serial.println("ERRO: ENC_DIR deve ser 1 ou -1");
    }

    return;
  }

  if (cmd.startsWith("MOTOR_DIR ")) {
    int value = cmd.substring(10).toInt();

    if (value == 1 || value == -1) {
      motorDir = value;

      Serial.print("MOTOR_DIR=");
      Serial.println(motorDir);
    } else {
      Serial.println("ERRO: MOTOR_DIR deve ser 1 ou -1");
    }

    return;
  }

  Serial.println("ERRO: comando desconhecido. Use HELP");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      if (serialBuffer.length() < 79)
        serialBuffer += c;
    }
  }
}

void controlLoop() {
  unsigned long now = micros();

  if ((unsigned long)(now - lastControlUs) < CONTROL_PERIOD_US)
    return;

  float dt =
    (float)(now - lastControlUs) /
    1000000.0f;

  lastControlUs = now;

  if (dt <= 0.0f)
    return;

  float angle = getEncoderAngle();

  float error =
    shortestAngleError(SETPOINT_DEG, angle);

  if (fabsf(error) > GUARD_HALF_WIDTH) {
    stopControl("GUARD");
    return;
  }

  float deltaAngle =
    shortestAngleError(angle, previousAngle);

  float measuredVelocity =
    deltaAngle / dt;

  filteredVelocity =
    (DERIVATIVE_FILTER_ALPHA * filteredVelocity) +
    ((1.0f - DERIVATIVE_FILTER_ALPHA) * measuredVelocity);

  previousAngle = angle;

  float proportional =
    Kp * error;

  float derivative =
    -Kd * filteredVelocity;

  float integralTerm =
    Ki * integral;

  float outputDeg =
    proportional +
    integralTerm +
    derivative +
    Bias;

  float outputLimit =
    MAX_CONTROL_SPEED_DEG_S * dt;

  if (outputLimit < 0.001f)
    outputLimit = 0.001f;

  if (outputDeg > outputLimit)
    outputDeg = outputLimit;

  if (outputDeg < -outputLimit)
    outputDeg = -outputLimit;

  bool saturatedHigh =
    (outputDeg >= outputLimit);

  bool saturatedLow =
    (outputDeg <= -outputLimit);

  bool integralAllowed =
    (!saturatedHigh && !saturatedLow) ||
    (saturatedHigh && error < 0.0f) ||
    (saturatedLow && error > 0.0f);

  if (integralAllowed) {
    integral += error * dt;

    if (integral > INTEGRAL_CLAMP)
      integral = INTEGRAL_CLAMP;

    if (integral < -INTEGRAL_CLAMP)
      integral = -INTEGRAL_CLAMP;
  }

  outputDeg =
    (Kp * error) +
    (Ki * integral) -
    (Kd * filteredVelocity) +
    Bias;

  float speedDegPerSecond =
    outputDeg / dt;

  if (speedDegPerSecond > MAX_CONTROL_SPEED_DEG_S)
    speedDegPerSecond = MAX_CONTROL_SPEED_DEG_S;

  if (speedDegPerSecond < -MAX_CONTROL_SPEED_DEG_S)
    speedDegPerSecond = -MAX_CONTROL_SPEED_DEG_S;

  speedDegPerSecond *= (float)motorDir;

  float motorAngle =
    fabsf(getMotorAngle());

  if (MAX_STEPPER_ABS_DEG > 0.0f &&
      motorAngle >= MAX_STEPPER_ABS_DEG) {

    stopControl("MOTOR_LIMIT");
    return;
  }

  motor.setSpeed(
    speedDegPerSecond *
    MOTOR_STEPS_PER_DEG
  );

  if ((unsigned long)(now - lastStatusUs) >= STATUS_PERIOD_US) {
    lastStatusUs = now;

    Serial.print("E=");
    Serial.print(angle, 2);

    Serial.print(" ERR=");
    Serial.print(error, 2);

    Serial.print(" V=");
    Serial.print(filteredVelocity, 1);

    Serial.print(" OUT=");
    Serial.print(speedDegPerSecond, 1);

    Serial.print(" M=");
    Serial.println(getMotorAngle(), 1);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);

  digitalWrite(PIN_MS1, HIGH);
  digitalWrite(PIN_MS2, HIGH);
  digitalWrite(PIN_MS3, HIGH);

  pinMode(PIN_ENABLE, OUTPUT);
  disableMotor();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_A),
    encoderA_ISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_B),
    encoderB_ISR,
    CHANGE
  );

  motor.setMaxSpeed(30000);
  motor.setMinPulseWidth(2);
  motor.setCurrentPosition(0);
  motor.setSpeed(0);

  lastControlUs = micros();
  lastStatusUs = micros();

  Serial.println();
  Serial.println("PENDULO UNO R3");
  Serial.print("PID INICIAL: kp: "); Serial.print(Kp, 4);
  Serial.print(" Ki: "); Serial.print(Ki, 4);
  Serial.print(" Kd: "); Serial.print(Kd, 4);
  Serial.print(" Bias: "); Serial.println(Bias, 4);
  Serial.println("Digite HELP");
  Serial.println();
}

void loop() {
  readSerial();

  if (controlEnabled) {
    controlLoop();
  }

  motor.runSpeed();
}
