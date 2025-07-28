#include "signal.h"
#include <stdbool.h>

/**
 * @brief  初始化通用斜坡生成器
 * @param  ramp       指向 RampGenerator_t 结构体的指针
 * @param  slope      变化率 (单位/s)
 * @param  limit_min  最小值限制
 * @param  limit_max  最大值限制
 * @param  Ts         采样周期 (s)
 * @retval None
 */
void RampGenerator_Init(RampGenerator_t* ramp, float slope, float limit_min, float limit_max, float Ts)
{
  ramp->value = 0.0f;
  ramp->slope = slope;
  ramp->limit_min = limit_min;
  ramp->limit_max = limit_max;
  ramp->target = 0.0f;
  ramp->Ts = Ts;
}

/**
 * @brief  初始化正弦波生成器
 * @param  sine       指向 SineWave_t 结构体的指针
 * @param  amplitude  幅值
 * @param  frequency  频率 [Hz]
 * @param  phase      相位偏移 [rad]
 * @param  Ts         采样周期 (s)
 * @retval None
 */
void SineWave_Init(SineWave_t* sine, float amplitude, float frequency, float phase, float Ts)
{
  sine->amplitude = amplitude;
  sine->frequency = frequency;
  sine->phase = phase;
  sine->theta = phase;
  sine->Ts = Ts;
}

/**
 * @brief  初始化方波生成器
 * @param  square     指向 SquareWave_t 结构体的指针
 * @param  amplitude  幅值
 * @param  frequency  频率 [Hz]
 * @param  duty_cycle 占空比 [0.0-1.0]
 * @param  Ts         采样周期 (s)
 * @retval None
 */
void SquareWave_Init(SquareWave_t* square, float amplitude, float frequency, float duty_cycle, float Ts)
{
  square->amplitude = amplitude;
  square->frequency = frequency;
  square->duty_cycle = duty_cycle;
  square->counter = 0.0f;
  square->Ts = Ts;
  square->state = false;
}

/**
 * @brief  初始化锯齿波生成器
 * @param  sawtooth   指向 SawtoothWave_t 结构体的指针
 * @param  amplitude  幅值
 * @param  frequency  频率 [Hz] (正数递增，负数递减)
 * @param  offset     直流偏移
 * @param  Ts         采样周期 (s)
 * @retval None
 */
void SawtoothWave_Init(SawtoothWave_t* sawtooth, float amplitude, float frequency, float offset, float Ts)
{
  sawtooth->amplitude = amplitude;
  sawtooth->frequency = frequency;
  sawtooth->offset = offset;
  sawtooth->counter = 0.0f;
  sawtooth->Ts = Ts;
}