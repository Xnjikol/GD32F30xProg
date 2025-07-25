#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdbool.h>

// 通用斜坡生成器
typedef struct
{
  float value;      // 当前输出值
  float slope;      // 变化率 (单位/s)
  float limit_min;  // 最小值限制
  float limit_max;  // 最大值限制
  float target;     // 目标值
  float Ts;         // 采样周期
} RampGenerator_t;

// 转速斜坡生成器 (基于通用斜坡)
typedef RampGenerator_t Speed_Ramp_t;

// 正弦波生成器
typedef struct
{
  float amplitude;  // 幅值
  float frequency;  // 频率 [Hz]
  float phase;      // 相位偏移 [rad]
  float theta;      // 当前相位角 [rad]
  float Ts;         // 采样周期
} SineWave_t;

// 方波生成器
typedef struct
{
  float amplitude;   // 幅值
  float frequency;   // 频率 [Hz]
  float duty_cycle;  // 占空比 [0.0-1.0]
  float counter;     // 内部计数器
  float Ts;          // 采样周期
  bool state;        // 当前状态
} SquareWave_t;

/**
 * @brief  通用斜坡生成器
 * @param  ramp   指向 RampGenerator_t 结构体的指针
 * @param  reset  是否复位斜坡（true: 复位，false: 正常运行）
 * @retval 当前输出值
 */
static inline float RampGenerator(RampGenerator_t* ramp, bool reset)
{
  if (reset)
  {
    ramp->value = 0.0f;
    ramp->target = 0.0f;
  }

  float delta = ramp->target - ramp->value;
  float step = ramp->slope * ramp->Ts;

  // 限制步进，防止越界
  if (delta > step)
  {
    ramp->value += step;
  }
  else if (delta < -step)
  {
    ramp->value -= step;
  }
  else
  {
    ramp->value = ramp->target;
  }

  // 输出限幅
  if (ramp->value > ramp->limit_max)
  {
    ramp->value = ramp->limit_max;
  }
  else if (ramp->value < ramp->limit_min)
  {
    ramp->value = ramp->limit_min;
  }

  return ramp->value;
}

#endif