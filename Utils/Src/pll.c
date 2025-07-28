/**
 * @file pll.c
 * @brief 通用PLL(Phase-Locked Loop)模块实现
 * @author FRECON
 * @date 2025年7月28日
 * @version 1.0
 *
 * 该文件实现了通用的PLL模块，可用于位置跟踪、频率跟踪等应用
 */

#include "pll.h"
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846F /* π */
#endif

/**
 * @brief 初始化PLL控制器
 */
int PLL_Init(pll_t* pll, const pll_params_t* params)
{
  if (pll == NULL || params == NULL)
  {
    return -1;
  }

  /* 参数检查 */
  if (params->ts <= 0.0f || params->max_output <= params->min_output)
  {
    return -1;
  }

  /* 复制参数 */
  memcpy(&pll->params, params, sizeof(pll_params_t));

  /* 初始化PID控制器 */
  memset(&pll->pid, 0, sizeof(PID_Handler_t));
  pll->pid.Kp = params->kp;
  pll->pid.Ki = params->ki;
  pll->pid.Kd = params->kd;
  pll->pid.Ts = params->ts;
  pll->pid.MaxOutput = params->max_output;
  pll->pid.MinOutput = params->min_output;
  pll->pid.IntegralLimit = params->integral_limit;
  pll->pid.Reset = false;

  /* 初始化状态 */
  memset(&pll->state, 0, sizeof(pll_state_t));
  pll->state.is_initialized = true;
  pll->state.is_enabled = false;

  return 0;
}

/**
 * @brief 反初始化PLL控制器
 */
void PLL_DeInit(pll_t* pll)
{
  if (pll == NULL)
  {
    return;
  }

  /* 清零状态 */
  memset(&pll->state, 0, sizeof(pll_state_t));
}

/**
 * @brief 更新PLL控制器
 */
float PLL_Update(pll_t* pll, float error)
{
  if (pll == NULL || !pll->state.is_initialized || !pll->state.is_enabled)
  {
    return pll ? pll->state.theta : 0.0f;
  }

  /* 保存误差 */
  pll->state.error = error;

  /* 使用PID控制器计算速度估计 */
  Pid_Update(-error, false, &pll->pid); /* 负反馈 */
  pll->state.omega = pll->pid.output;

  /* 位置积分 */
  pll->state.theta += pll->state.omega * pll->params.ts;
  
  /* 角度归一化到 [-π, π] */
  pll->state.theta = wrap_theta_pi(pll->state.theta);

  return pll->state.theta;
}

/**
 * @brief 获取估计的位置
 */
float PLL_GetPosition(const pll_t* pll)
{
  if (pll == NULL)
  {
    return 0.0f;
  }

  return pll->state.theta;
}

/**
 * @brief 获取估计的速度
 */
float PLL_GetSpeed(const pll_t* pll)
{
  if (pll == NULL)
  {
    return 0.0f;
  }

  return pll->state.omega;
}

/**
 * @brief 使能/禁用PLL控制器
 */
void PLL_Enable(pll_t* pll, bool enable)
{
  if (pll == NULL || !pll->state.is_initialized)
  {
    return;
  }

  pll->state.is_enabled = enable;

  if (!enable)
  {
    /* 禁用时重置PID控制器 */
    Pid_Update(0.0f, true, &pll->pid);
  }
}

/**
 * @brief 重置PLL控制器状态
 */
void PLL_Reset(pll_t* pll)
{
  if (pll == NULL || !pll->state.is_initialized)
  {
    return;
  }

  /* 重置状态变量 */
  pll->state.theta = 0.0f;
  pll->state.omega = 0.0f;
  pll->state.theta_prev = 0.0f;
  pll->state.error = 0.0f;

  /* 重置PID控制器 */
  Pid_Update(0.0f, true, &pll->pid);
}

/**
 * @brief 设置初始位置
 */
void PLL_SetInitialPosition(pll_t* pll, float initial_theta)
{
  if (pll == NULL || !pll->state.is_initialized)
  {
    return;
  }

  pll->state.theta = wrap_theta_pi(initial_theta);
  pll->state.theta_prev = pll->state.theta;
}

/**
 * @brief 更新PLL参数
 */
void PLL_UpdateParams(pll_t* pll, const pll_params_t* params)
{
  if (pll == NULL || params == NULL || !pll->state.is_initialized)
  {
    return;
  }

  /* 参数检查 */
  if (params->ts <= 0.0f || params->max_output <= params->min_output)
  {
    return;
  }

  /* 更新参数 */
  memcpy(&pll->params, params, sizeof(pll_params_t));

  /* 更新PID参数 */
  pll->pid.Kp = params->kp;
  pll->pid.Ki = params->ki;
  pll->pid.Kd = params->kd;
  pll->pid.Ts = params->ts;
  pll->pid.MaxOutput = params->max_output;
  pll->pid.MinOutput = params->min_output;
  pll->pid.IntegralLimit = params->integral_limit;
}

/**
 * @brief 检查PLL是否已初始化
 */
bool PLL_IsInitialized(const pll_t* pll)
{
  if (pll == NULL)
  {
    return false;
  }

  return pll->state.is_initialized;
}

/**
 * @brief 检查PLL是否已使能
 */
bool PLL_IsEnabled(const pll_t* pll)
{
  if (pll == NULL)
  {
    return false;
  }

  return pll->state.is_enabled;
}
