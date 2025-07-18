#include "math_utils.h"

/*======================*/
/*  Angle Functions     */
/*======================*/

/**
 * @brief 角度归一化到[0, 2π)
 * @param angle 输入角度 (rad)
 * @return 归一化后的角度 (rad)
 */
float MathUtils_WrapAngle2Pi(float angle)
{
    angle = MOD(angle, 2.0f * M_PI);
    if (angle < 0.0f)
    {
        angle += 2.0f * M_PI;
    }
    return angle;
}

/**
 * @brief 角度归一化到[-π, π)
 * @param angle 输入角度 (rad)
 * @return 归一化后的角度 (rad)
 */
float MathUtils_WrapAnglePi(float angle)
{
    angle = MOD(angle + M_PI, 2.0f * M_PI) - M_PI;
    return angle;
}

/**
 * @brief 计算两个角度的差值
 * @param angle1 角度1 (rad)
 * @param angle2 角度2 (rad)
 * @return 角度差值 (rad)，范围在[-π, π)
 */
float MathUtils_AngleDifference(float angle1, float angle2)
{
    float diff = angle1 - angle2;

    if (diff > M_PI)
    {
        diff -= 2.0f * M_PI;
    }
    else if (diff < -M_PI)
    {
        diff += 2.0f * M_PI;
    }

    return diff;
}

/*======================*/
/*  Utility Functions   */
/*======================*/

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限幅后的值
 */
float MathUtils_Clamp(float value, float min_val, float max_val)
{
    if (value > max_val)
    {
        return max_val;
    }
    else if (value < min_val)
    {
        return min_val;
    }
    else
    {
        return value;
    }
}

/**
 * @brief 符号函数
 * @param value 输入值
 * @return 1.0f (value > 0), -1.0f (value < 0), 0.0f (value == 0)
 */
float MathUtils_Sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    else if (value < 0.0f)
    {
        return -1.0f;
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief 死区函数
 * @param value 输入值
 * @param deadband 死区范围
 * @return 去除死区后的值
 */
float MathUtils_Deadband(float value, float deadband)
{
    if (value > deadband)
    {
        return value - deadband;
    }
    else if (value < -deadband)
    {
        return value + deadband;
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief 线性插值
 * @param x0 起始点x坐标
 * @param y0 起始点y坐标
 * @param x1 结束点x坐标
 * @param y1 结束点y坐标
 * @param x 插值点x坐标
 * @return 插值结果y
 */
float MathUtils_LinearInterpolate(float x0, float y0, float x1, float y1, float x)
{
    if (x1 == x0)
    {
        return y0; /* 避免除零 */
    }
    
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}
