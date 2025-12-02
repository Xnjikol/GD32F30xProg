/**
 * @file pll.h
 * @brief 通用PLL(Phase-Locked Loop)模块头文件
 * @author zfy
 * @date 2025年11月24日
 * @version 2.0
 *
 * 该文件定义了通用的PLL模块接口，可用于位置跟踪、频率跟踪等应用
 * 内部使用PID控制器实现闭环控制
 */

#ifndef _PLL_H_
#define _PLL_H_

#include <stdbool.h>
#include "pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
   * @brief PLL参数结构体
   */
    typedef struct
    {
        float kp;  /**< 比例增益 */
        float ki;  /**< 积分增益 */
        float kd;  /**< 微分增益 */
        float ts;  /**< 采样周期 (s) */
        float max; /**< 最大输出限制 (rad/s) */
        float min; /**< 最小输出限制 (rad/s) */
    } pll_params_t;

    /**
   * @brief PLL控制器结构体
   */
    typedef struct
    {
        PID_Handler_t pid;   /**< PID控制器 */
        float         theta; /* < 当前估计位置 (rad) */
        float         omega; /* < 当前估计速度 (rad/s) */
        float         error; /* < 位置误差 (rad) */
    } Pll_Handler_t;

    /**
   * @brief 初始化PLL控制器
   *
   * @param pll PLL控制器指针
   * @param params PLL参数指针
   * @return int 0:成功 -1:失败
   */
    bool Pll_Init(Pll_Handler_t* pll, const pll_params_t* params);

    /**
   * @brief 更新PLL控制器
   *
   * @param pll PLL控制器指针
   * @param error 位置误差输入 (rad)
   * @return float 估计的位置 (rad)
   */
    float Pll_Update(Pll_Handler_t* pll, float error, bool reset);

#ifdef __cplusplus
}
#endif

#endif /* _PLL_H_ */
