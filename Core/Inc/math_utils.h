#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include <math.h>
#include <stdint.h>

/* 数学常数 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

/* 数学宏定义 */
#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)
#define MOD(x, y) fmodf((x), (y))
#define ABS(x) arm_abs_f32(x)

#else

#define COS(x) cosf(x)
#define SIN(x) sinf(x)
#define MOD(x, y) fmodf((x), (y))
#define ABS(x) fabsf(x)

#endif

/* 常用数学常数 */
#define SQRT3 1.73205080757f
#define SQRT2 1.41421356237f
#define INV_SQRT3 0.57735026919f  /* 1/sqrt(3) */

/*======================*/
/*  Function Prototypes */
/*======================*/

/**
 * @brief 角度归一化到[0, 2π)
 * @param angle 输入角度 (rad)
 * @return 归一化后的角度 (rad)
 */
float MathUtils_WrapAngle2Pi(float angle);

/**
 * @brief 角度归一化到[-π, π)
 * @param angle 输入角度 (rad)
 * @return 归一化后的角度 (rad)
 */
float MathUtils_WrapAnglePi(float angle);

/**
 * @brief 计算两个角度的差值
 * @param angle1 角度1 (rad)
 * @param angle2 角度2 (rad)
 * @return 角度差值 (rad)，范围在[-π, π)
 */
float MathUtils_AngleDifference(float angle1, float angle2);

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限幅后的值
 */
float MathUtils_Clamp(float value, float min_val, float max_val);

/**
 * @brief 符号函数
 * @param value 输入值
 * @return 1.0f (value > 0), -1.0f (value < 0), 0.0f (value == 0)
 */
float MathUtils_Sign(float value);

/**
 * @brief 死区函数
 * @param value 输入值
 * @param deadband 死区范围
 * @return 去除死区后的值
 */
float MathUtils_Deadband(float value, float deadband);

/**
 * @brief 线性插值
 * @param x0 起始点x坐标
 * @param y0 起始点y坐标
 * @param x1 结束点x坐标
 * @param y1 结束点y坐标
 * @param x 插值点x坐标
 * @return 插值结果y
 */
float MathUtils_LinearInterpolate(float x0, float y0, float x1, float y1, float x);

/**
 * @brief 角度到弧度转换
 * @param degrees 角度值
 * @return 弧度值
 */
static inline float MathUtils_DegreesToRadians(float degrees)
{
    return degrees * M_PI / 180.0f;
}

/**
 * @brief 弧度到角度转换
 * @param radians 弧度值
 * @return 角度值
 */
static inline float MathUtils_RadiansToDegrees(float radians)
{
    return radians * 180.0f / M_PI;
}

/**
 * @brief 平方函数
 * @param x 输入值
 * @return x的平方
 */
static inline float MathUtils_Square(float x)
{
    return x * x;
}

/**
 * @brief 计算向量模长
 * @param x x分量
 * @param y y分量
 * @return 向量模长
 */
static inline float MathUtils_VectorMagnitude(float x, float y)
{
    return sqrtf(x * x + y * y);
}

#endif /* _MATH_UTILS_H_ */
