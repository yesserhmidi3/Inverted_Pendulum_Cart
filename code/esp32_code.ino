#include "motor.h"
#include "mpu.h"
#include "pid.h"

static PidBlock sBalancePid;
static const float TARGET_ANGLE_DEG = 0.0f;
static const float MOTOR_POLARITY = -1.0f;
static const float FALL_ANGLE_LIMIT_DEG = 45.0f;
static const float UPRIGHT_DEADBAND_DEG = 0.5f;
static uint32_t sLastControlMicros = 0;
static uint32_t sLastPrintMs = 0;

static void printTelemetry(float thetaDeg, float pidCmd, float reqCmd, const char *state) {
  uint32_t nowMs = millis();
  if (nowMs - sLastPrintMs < 100) {
    return;
  }

  sLastPrintMs = nowMs;
  Serial.print("state=");
  Serial.print(state);
  Serial.print(" theta=");
  Serial.print(thetaDeg, 2);
  Serial.print(" pid=");
  Serial.print(pidCmd, 3);
  Serial.print(" req=");
  Serial.print(reqCmd, 3);
  Serial.print(" applied=");
  Serial.println(motorGetLastAppliedCommand(), 3);
}

void setup() {
  Serial.begin(115200);

  bool motorOk = motorBegin();
  bool imuOk = pendulumAngleBegin();

  pidInit(sBalancePid, 0.7f, 0.05f, 0.0f, -1.0f, 1.0f);
  pidSetIntegratorLimits(sBalancePid, -0.4f, 0.4f);
  sLastControlMicros = micros();

  if (!imuOk) {
    Serial.println("MPU init failed");
  }
  if (!motorOk) {
    Serial.println("Motor init failed");
  }
}

void loop() {
  float thetaDeg = 0.0f;
  if (!pendulumAngleWork(thetaDeg)) {
    motorStop();
    printTelemetry(thetaDeg, 0.0f, 0.0f, "IMU_FAIL");
    delay(5);
    return;
  }

  if (thetaDeg > FALL_ANGLE_LIMIT_DEG || thetaDeg < -FALL_ANGLE_LIMIT_DEG) {
    motorStop();
    pidReset(sBalancePid);
    sLastControlMicros = micros();
    printTelemetry(thetaDeg, 0.0f, 0.0f, "CUT_OFF");
    delay(5);
    return;
  }

  if (thetaDeg >= -UPRIGHT_DEADBAND_DEG && thetaDeg <= UPRIGHT_DEADBAND_DEG) {
    motorStop();
    pidReset(sBalancePid);
    sLastControlMicros = micros();
    printTelemetry(thetaDeg, 0.0f, 0.0f, "ZERO_HOLD");
    delay(5);
    return;
  }

  uint32_t now = micros();
  float dt = (now - sLastControlMicros) * 1e-6f;
  sLastControlMicros = now;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  float motorCmd = pidUpdate(sBalancePid, TARGET_ANGLE_DEG, thetaDeg, dt);
  float motorDriveCmd = MOTOR_POLARITY * motorCmd;
  motorSetNormalized(motorDriveCmd);

  printTelemetry(thetaDeg, motorCmd, motorDriveCmd, "RUN");

  delay(5);
}