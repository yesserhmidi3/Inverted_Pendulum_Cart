#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PidBlock {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prevMeasurement;
	bool hasPrevMeasurement;
	float outMin;
	float outMax;
	float iMin;
	float iMax;
};

void pidInit(PidBlock &pid, float kp, float ki, float kd, float outMin, float outMax);
void pidSetIntegratorLimits(PidBlock &pid, float iMin, float iMax);
void pidReset(PidBlock &pid);
float pidUpdate(PidBlock &pid, float setpoint, float measurement, float dt);

#endif
