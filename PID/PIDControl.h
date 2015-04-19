#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <math.h>

#define I_ZERO_THRESHOLD (0.001f)

typedef struct
{
	float integral;
	float prev_error_x;
} PIDControlState_t;

typedef struct
{
	float p, i, d;
	float idf;
} PIDControlCoefficients_t;

typedef struct
{
    PIDControlState_t state;
    PIDControlCoefficients_t coefficients;
} PIDController_t;

void PID_PIDController(PIDController_t* controller, float pCoeff, float iCoeff, float dCoeff, float iDecayFactor);
float PID_calculate(PIDController_t* controller, float x, float target_x, float dt);

#endif // PIDCONTROL_H
