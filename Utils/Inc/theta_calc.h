#ifndef _THETA_CALC_H_
#define _THETA_CALC_H_

#include <stdbool.h>
#include <stdint.h>

#include "gd32f30x.h"

#ifndef SQRT3
#define SQRT3 1.73205080757F
#endif

#ifndef SQRT3_2
#define SQRT3_2 0.86602540378F        /* √3/2 */
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692F /* 2π */
#endif

/*  DSP math function    */
#ifndef ARM_DSP
#define ARM_DSP
#endif

#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)

#else
#include <math.h>

#define COS(x) cosf(x)
#define SIN(x) sinf(x)

#endif

/**
 * @brief 将角度限制在 [-π, π) 区间
 * @param theta 输入角度（弧度）
 * @return 限幅后的角度（弧度）
 */
static inline float_t wrap_theta_pi(float_t theta)
{
  // 先将角度归一化到 [-2π, 2π) 范围
  theta = fmodf(theta, M_2PI);

  // 如果角度大于π，则减去2π使其落入[-π, π)
  if (theta >= M_PI)
  {
    theta -= M_2PI;
  }
  // 如果角度小于-π，则加上2π使其落入[-π, π)
  else if (theta < -M_PI)
  {
    theta += M_2PI;
  }

  return theta;
}

/**
 * @brief 将角度限制在 [0, 2π) 区间
 * @param theta 输入角度（弧度）
 * @return 限幅后的角度（弧度）
 */
static inline float_t wrap_theta_2pi(float_t theta)
{
  theta = fmodf(theta, M_2PI);
  if (theta < 0.0F)
  {
    theta += M_2PI;
  }
  return theta;
}

/**
 * @brief 角度转弧度
 * @param deg 角度值
 * @return 弧度值
 */
static inline float_t deg2rad(float_t deg)
{
  return deg * (M_2PI / 360.0F);
}

/**
 * @brief 弧度转角度
 * @param rad 弧度值
 * @return 角度值
 */
static inline float_t rad2deg(float_t rad)
{
  return rad * (360.0F / M_2PI);
}

/**
 * @brief 转速(rpm)转角速度(rad/s)
 * @param rpm 转速（每分钟转数）
 * @return 角速度（弧度/秒）
 */
static inline float_t rpm2radps(float_t rpm)
{
  return rpm * (M_2PI / 60.0F);
}

/**
 * @brief 角速度(rad/s)转转速(rpm)
 * @param radps 角速度（弧度/秒）
 * @return 转速（每分钟转数）
 */
static inline float_t radps2rpm(float_t radps)
{
  return radps * (60.0F / M_2PI);
}

#endif /* _THETA_CALC_H_ */