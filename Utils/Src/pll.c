/**
 * @file pll.c
 * @brief 通用PLL(Phase-Locked Loop)模块实现
 * @author zfy
 * @date 2025年11月24日
 * @version 2.0
 *
 * 该文件实现了通用的PLL模块，可用于位置跟踪、频率跟踪等应用
 */

#include "pll.h"
#include <stdbool.h>
#include <string.h>
#include "motor.h"
#include "theta_calc.h"

/**
 * @brief 初始化PLL控制器
 */
bool Pll_Init(Pll_Handler_t* pll, const pll_params_t* params)
{
    if (pll == NULL || params == NULL)
    {
        return -1;
    }

    /* 参数检查 */
    if (params->ts <= 0.0f || params->max <= params->min)
    {
        return false;
    }

    /* 初始化PID控制器 */
    memset(&pll->pid, 0, sizeof(PID_Handler_t));
    pll->pid.Kp            = params->kp;
    pll->pid.Ki            = params->ki;
    pll->pid.Kd            = params->kd;
    pll->pid.Ts            = params->ts;
    pll->pid.Max           = params->max;
    pll->pid.Min           = params->min;
    pll->pid.IntegralLimit = params->max;
    pll->pid.Reset         = false;

    return true;
}

/**
 * @brief 更新PLL控制器
 */
float Pll_Update(Pll_Handler_t* pll, float error, bool reset)
{
    if (pll == NULL)
    {
        return false;
    }

    /* 使用PID控制器计算速度估计 */
    Pid_Update(error, reset, &pll->pid); /* 负反馈 */
    pll->omega = pll->pid.output;

    /* 位置积分 */
    pll->theta += pll->omega * SampleTime;

    /* 角度归一化到 [0, 2π] */
    pll->theta = wrap_theta_2pi(pll->theta);

    return pll->theta;
}
