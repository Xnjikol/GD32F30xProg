/**
 * @file pll.h
 * @brief 通用PLL(Phase-Locked Loop)模块头文件
 * @author FRECON
 * @date 2025年7月28日
 * @version 1.0
 *
 * 该文件定义了通用的PLL模块接口，可用于位置跟踪、频率跟踪等应用
 * 内部使用PID控制器实现闭环控制
 */

#ifndef _PLL_H_
#define _PLL_H_

#include "gd32f30x.h"
#include "pid.h"
#include "theta_calc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PLL参数结构体
 */
typedef struct
{
  float kp;            /**< 比例增益 */
  float ki;            /**< 积分增益 */
  float kd;            /**< 微分增益 */
  float ts;            /**< 采样周期 (s) */
  float max_output;    /**< 最大输出限制 (rad/s) */
  float min_output;    /**< 最小输出限制 (rad/s) */
  float integral_limit; /**< 积分限幅 (rad/s) */
} pll_params_t;

/**
 * @brief PLL状态结构体
 */
typedef struct
{
  float theta;          /**< 当前估计位置 (rad) */
  float omega;          /**< 当前估计速度 (rad/s) */
  float theta_prev;     /**< 前一次位置 (rad) */
  float error;          /**< 位置误差 (rad) */
  bool is_initialized;  /**< 初始化标志 */
  bool is_enabled;      /**< 使能标志 */
} pll_state_t;

/**
 * @brief PLL控制器结构体
 */
typedef struct
{
  pll_params_t params;  /**< PLL参数 */
  pll_state_t state;    /**< PLL状态 */
  PID_Handler_t pid;    /**< PID控制器 */
} pll_t;

/**
 * @brief 初始化PLL控制器
 * 
 * @param pll PLL控制器指针
 * @param params PLL参数指针
 * @return int 0:成功 -1:失败
 */
int PLL_Init(pll_t* pll, const pll_params_t* params);

/**
 * @brief 反初始化PLL控制器
 * 
 * @param pll PLL控制器指针
 */
void PLL_DeInit(pll_t* pll);

/**
 * @brief 更新PLL控制器
 * 
 * @param pll PLL控制器指针
 * @param error 位置误差输入 (rad)
 * @return float 估计的位置 (rad)
 */
float PLL_Update(pll_t* pll, float error);

/**
 * @brief 获取估计的位置
 * 
 * @param pll PLL控制器指针
 * @return float 估计位置 (rad)
 */
float PLL_GetPosition(const pll_t* pll);

/**
 * @brief 获取估计的速度
 * 
 * @param pll PLL控制器指针
 * @return float 估计速度 (rad/s)
 */
float PLL_GetSpeed(const pll_t* pll);

/**
 * @brief 使能/禁用PLL控制器
 * 
 * @param pll PLL控制器指针
 * @param enable 使能标志 (true:使能 false:禁用)
 */
void PLL_Enable(pll_t* pll, bool enable);

/**
 * @brief 重置PLL控制器状态
 * 
 * @param pll PLL控制器指针
 */
void PLL_Reset(pll_t* pll);

/**
 * @brief 设置初始位置
 * 
 * @param pll PLL控制器指针
 * @param initial_theta 初始位置 (rad)
 */
void PLL_SetInitialPosition(pll_t* pll, float initial_theta);

/**
 * @brief 更新PLL参数
 * 
 * @param pll PLL控制器指针
 * @param params 新的PLL参数
 */
void PLL_UpdateParams(pll_t* pll, const pll_params_t* params);

/**
 * @brief 检查PLL是否已初始化
 * 
 * @param pll PLL控制器指针
 * @return bool true:已初始化 false:未初始化
 */
bool PLL_IsInitialized(const pll_t* pll);

/**
 * @brief 检查PLL是否已使能
 * 
 * @param pll PLL控制器指针
 * @return bool true:已使能 false:未使能
 */
bool PLL_IsEnabled(const pll_t* pll);

#ifdef __cplusplus
}
#endif

#endif /* _PLL_H_ */
