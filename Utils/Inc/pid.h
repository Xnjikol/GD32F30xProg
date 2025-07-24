#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>
#include <stdint.h>

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
} PID_Handler_t;

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
void PID_SetIntegral(PID_Handler_t* handler, bool Reset, float value);

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
static inline void Pid_Update(float error, bool Reset, PID_Handler_t* handler)
{
  if (Reset)
  {
    handler->integral = 0.0F;
    handler->previous_error = 0.0F;
    handler->output = 0.0F;
    handler->Reset = true;
    return;
  }

  // 计算比例项
  const float proportional = handler->Kp * error;

  // 计算未限幅输出（用于条件积分判断）
  const float output_unclamped = proportional + handler->Ki * handler->integral;
  const bool is_output_limited =
      (output_unclamped > handler->MaxOutput) || (output_unclamped < handler->MinOutput);

  // 条件积分抗饱和：仅当输出未限幅且Ki有效时才累加积分
  if (!is_output_limited && handler->Ki != 0.0F && handler->Ts > 0.0F)
  {
    handler->integral += error * handler->Ts;

    // 积分限幅保护
    if (handler->integral > handler->IntegralLimit)
      handler->integral = handler->IntegralLimit;
    else if (handler->integral < -handler->IntegralLimit)
      handler->integral = -handler->IntegralLimit;
  }

  // 计算微分项（考虑采样时间，避免除零）
  float derivative = 0.0F;
  if (handler->Kd != 0.0F && handler->Ts > 0.0F)
  {
    derivative = handler->Kd * (error - handler->previous_error) / handler->Ts;
    handler->previous_error = error;
  }

  // 计算完整输出
  const float total_output = proportional + handler->Ki * handler->integral + derivative;

  // 输出限幅
  if (total_output > handler->MaxOutput)
    handler->output = handler->MaxOutput;
  else if (total_output < handler->MinOutput)
    handler->output = handler->MinOutput;
  else
    handler->output = total_output;
}

#endif /* _PID_H_ */