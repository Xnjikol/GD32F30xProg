#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdbool.h>

#ifndef SQRT3
#define SQRT3 1.73205080757F
#endif

#ifndef SQRT3_2
#define SQRT3_2 0.86602540378F /* √3/2 */
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692F /* 2π */
#endif

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

// 锯齿波生成器
typedef struct
{
  float amplitude;   // 幅值
  float frequency;   // 频率 [Hz] (正数递增，负数递减)
  float offset;      // 直流偏移
  float counter;     // 内部计数器 [0.0-1.0]
  float Ts;          // 采样周期
  bool initialized;  // 是否已初始化
} SawtoothWave_t;

// 函数声明
void RampGenerator_Init(RampGenerator_t* ramp, float slope, float limit_min, float limit_max,
                        float Ts);
void SineWave_Init(SineWave_t* sine, float amplitude, float frequency, float phase, float Ts);
void SquareWave_Init(SquareWave_t* square, float amplitude, float frequency, float duty_cycle,
                     float Ts);
void SawtoothWave_Init(SawtoothWave_t* sawtooth, float amplitude, float frequency, float offset,
                       float Ts);

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

/**
 * @brief  锯齿波生成器
 * @param  sawtooth  指向 SawtoothWave_t 结构体的指针
 * @param  reset     是否复位锯齿波（true: 复位，false: 正常运行）
 * @retval 当前输出值
 */
static inline float SawtoothWaveGenerator(SawtoothWave_t* sawtooth, bool reset)
{
  if (reset)
  {
    sawtooth->counter = 0.0f;
  }

  // 计算每个采样周期的步进量
  float step = sawtooth->frequency * sawtooth->Ts;

  // 更新计数器
  sawtooth->counter += step;

  // 保持计数器在[0, 1]范围内
  if (sawtooth->frequency > 0.0f)
  {
    // 正向锯齿波：0→1然后跳回0
    if (sawtooth->counter >= 1.0f)
    {
      sawtooth->counter = sawtooth->counter - 1.0f;  // 取小数部分
    }
    else if (sawtooth->counter < 0.0f)
    {
      sawtooth->counter = 0.0f;
    }
  }
  else
  {
    // 反向锯齿波：1→0然后跳回1
    if (sawtooth->counter <= 0.0f)
    {
      sawtooth->counter = sawtooth->counter + 1.0f;  // 从1开始递减
    }
    else if (sawtooth->counter > 1.0f)
    {
      sawtooth->counter = 1.0f;
    }
  }

  // 生成锯齿波输出：offset + amplitude * counter
  return sawtooth->offset + sawtooth->amplitude * sawtooth->counter;
}

#endif