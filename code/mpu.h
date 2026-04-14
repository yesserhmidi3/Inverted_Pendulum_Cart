#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

bool pendulumAngleBegin(uint8_t sda = 21, uint8_t scl = 22, int biasSamples = 300);
bool pendulumAngleWork(float &thetaDeg);

#endif
