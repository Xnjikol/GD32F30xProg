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
  bool Reset;           /* Flag to check if the PID controller is reset */
} PID_Controller_t;

/**
 * @brief 初始化PID控制器。
 *
 * 本函数初始化PID控制器的参数和状态。
 * 如果Reset为true，则重置PID控制器的内部状态（积分项、前一误差、输出）。
 * 该函数操作传入的PID_Controller_t结构体指针handler。
 *
 * @param handler  指向PID_Controller_t结构体的指针，包含控制器参数和状态。
 * @param Reset    如果为true，则重置PID控制器的内部状态。
 */
void PID_SetIntegral(PID_Controller_t* handler, bool Reset, float value);

/**
 * @brief 更新PID控制器的输出。
 *
 * 本函数根据当前误差（error）更新PID控制器的输出值。
 * 如果Reset为true，则重置PID控制器的内部状态（积分项、前一误差、输出）。
 * 该函数操作传入的PID_Controller_t结构体指针handler。
 *
 * @param error    当前误差（设定值 - 实际值）。
 * @param Reset    如果为true，则重置PID控制器的内部状态。
 * @param handler  指向PID_Controller_t结构体的指针，包含控制器参数和状态。
 */
static inline void Pid_Update(float error, bool Reset, PID_Controller_t* handler)
{
  if (Reset)
  {
    handler->integral = 0.0F;
    handler->previous_error = 0.0F;
    handler->output = 0.0F;
    handler->Reset = true;
  }

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