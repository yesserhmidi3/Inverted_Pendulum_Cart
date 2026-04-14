#include "pid.h"

static float clampf(float value, float minValue, float maxValue) {
	if (value < minValue) {
		return minValue;
	}
	if (value > maxValue) {
		return maxValue;
	}
	return value;
}

void pidInit(PidBlock &pid, float kp, float ki, float kd, float outMin, float outMax) {
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.integrator = 0.0f;
	pid.prevMeasurement = 0.0f;
	pid.hasPrevMeasurement = false;
	pid.outMin = outMin;
	pid.outMax = outMax;
	if (pid.outMin > pid.outMax) {
		float temp = pid.outMin;
		pid.outMin = pid.outMax;
		pid.outMax = temp;
	}
	pid.iMin = pid.outMin;
	pid.iMax = pid.outMax;
}

void pidSetIntegratorLimits(PidBlock &pid, float iMin, float iMax) {
	if (iMin > iMax) {
		float temp = iMin;
		iMin = iMax;
		iMax = temp;
	}
	pid.iMin = iMin;
	pid.iMax = iMax;
	pid.integrator = clampf(pid.integrator, pid.iMin, pid.iMax);
}

void pidReset(PidBlock &pid) {
	pid.integrator = 0.0f;
	pid.prevMeasurement = 0.0f;
	pid.hasPrevMeasurement = false;
}

float pidUpdate(PidBlock &pid, float setpoint, float measurement, float dt) {
	float error = setpoint - measurement;

	if (dt <= 0.0f) {
		return clampf(pid.kp * error, pid.outMin, pid.outMax);
	}

	float pTerm = pid.kp * error;

	pid.integrator += pid.ki * error * dt;
	pid.integrator = clampf(pid.integrator, pid.iMin, pid.iMax);

	float dTerm = 0.0f;
	if (pid.hasPrevMeasurement) {
		float measurementRate = (measurement - pid.prevMeasurement) / dt;
		dTerm = -pid.kd * measurementRate;
	}

	pid.prevMeasurement = measurement;
	pid.hasPrevMeasurement = true;

	float output = pTerm + pid.integrator + dTerm;
	return clampf(output, pid.outMin, pid.outMax);
}
