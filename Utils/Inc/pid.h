#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "gd32f30x.h"

typedef struct
{
  /* data */
  float Kp;             /* Proportional gain */
  float Ki;             /* Integral gain */
  float Kd;             /* Derivative gain */
  float integral;       /* Integral term */
  float previous_error; /* Previous error for derivative calculation */
  float MaxOutput;      /* Maximum output limit */
  float MinOutput;      /* Minimum output limit */
  float output;         /* PID output value */
  float IntegralLimit;  /* Integral limit to prevent windup */
  float Ts;             /* Sample time */
} PID_Controller_t;

static inline void PID_Compute(float Ref, float Fdbk, bool Reset, PID_Controller_t* handler)
{
  if (Reset)
  {
    handler->integral = 0.0F;
    handler->previous_error = 0.0F;
    handler->output = 0.0F;
  }

  float error = Ref - Fdbk;
  handler->integral += error * handler->Ts;

  // Anti-windup
  if (handler->integral > handler->IntegralLimit)
    handler->integral = handler->IntegralLimit;
  else if (handler->integral < -handler->IntegralLimit)
    handler->integral = -handler->IntegralLimit;

  float derivative = error - handler->previous_error;
  handler->previous_error = error;

  // PID output calculation
  handler->output =
      handler->Kp * error + handler->Ki * handler->integral + handler->Kd * derivative;

  // Clamp output to limits
  if (handler->output > handler->MaxOutput)
    handler->output = handler->MaxOutput;
  else if (handler->output < handler->MinOutput)
    handler->output = handler->MinOutput;
}

#endif /* _PID_H_ */