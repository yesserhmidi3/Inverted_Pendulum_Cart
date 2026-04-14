#include "mpu.h"

#include <Wire.h>
#include <math.h>

static uint8_t sMpuAddr = 0x68;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const float ACCEL_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float ALPHA = 0.99f;

static float sThetaDeg = 0.0f;
static float sGyroZBiasDps = 0.0f;
static uint32_t sLastMicros = 0;
static bool sFilterInit = false;
static bool sReady = false;

static bool isSupportedWhoAmI(uint8_t whoAmI) {
  return whoAmI == 0x68 || whoAmI == 0x69 || whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x73;
}

static bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(sMpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

static bool mpuReadReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(sMpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)sMpuAddr, 1, true) != 1) {
    return false;
  }

  value = (uint8_t)Wire.read();
  return true;
}

static bool mpuReadRaw(
  int16_t &rawAX,
  int16_t &rawAY,
  int16_t &rawAZ,
  int16_t &rawGX,
  int16_t &rawGY,
  int16_t &rawGZ
) {
  Wire.beginTransmission(sMpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)sMpuAddr, 14, true) != 14) {
    return false;
  }

  rawAX = (Wire.read() << 8) | Wire.read();
  rawAY = (Wire.read() << 8) | Wire.read();
  rawAZ = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  rawGX = (Wire.read() << 8) | Wire.read();
  rawGY = (Wire.read() << 8) | Wire.read();
  rawGZ = (Wire.read() << 8) | Wire.read();
  return true;
}

bool pendulumAngleBegin(uint8_t sda, uint8_t scl, int biasSamples) {
  Wire.begin(sda, scl);
  delay(50);

  const uint8_t candidateAddrs[] = {0x68, 0x69, 0x70};
  uint8_t whoAmI = 0;
  bool found = false;
  for (uint8_t i = 0; i < sizeof(candidateAddrs); i++) {
    sMpuAddr = candidateAddrs[i];
    if (mpuReadReg(REG_WHO_AM_I, whoAmI) && isSupportedWhoAmI(whoAmI)) {
      found = true;
      break;
    }
  }

  if (!found) {
    return false;
  }

  if (!mpuWriteReg(REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  if (!mpuWriteReg(REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }
  if (!mpuWriteReg(REG_GYRO_CONFIG, 0x00)) {
    return false;
  }

  if (biasSamples <= 0) {
    biasSamples = 1;
  }

  long sumGz = 0;
  int valid = 0;

  for (int i = 0; i < biasSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      sumGz += gz;
      valid++;
    }
    delay(2);
  }

  if (valid == 0) {
    return false;
  }

  sGyroZBiasDps = ((float)sumGz / (float)valid) / GYRO_LSB_PER_DPS;
  sThetaDeg = 0.0f;
  sFilterInit = false;
  sLastMicros = micros();
  sReady = true;
  return true;
}

bool pendulumAngleWork(float &thetaDeg) {
  if (!sReady) {
    return false;
  }

  int16_t rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
  if (!mpuReadRaw(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ)) {
    return false;
  }

  float accX = rawAX / ACCEL_LSB_PER_G;
  float accY = rawAY / ACCEL_LSB_PER_G;
  float thetaAcc = atan2f(accX, accY) * RAD_TO_DEG;

  float gyroZ = rawGZ / GYRO_LSB_PER_DPS;
  float omega = gyroZ - sGyroZBiasDps;

  uint32_t now = micros();
  float dt = (now - sLastMicros) * 1e-6f;
  sLastMicros = now;
  if (dt <= 0.0f || dt > 0.2f) {
    dt = 0.01f;
  }

  if (!sFilterInit) {
    sThetaDeg = thetaAcc;
    sFilterInit = true;
  } else {
    sThetaDeg = ALPHA * (sThetaDeg + omega * dt) + (1.0f - ALPHA) * thetaAcc;
  }

  thetaDeg = sThetaDeg - 3;
  return true;
}