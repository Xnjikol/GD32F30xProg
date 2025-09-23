/**
 * @file hf_injection.h
 * @brief 脉振高频注入无传感器控制头文件
 * @author ZFY
 * @date 2025年7月25日
 * @version 1.0
 *
 * 该文件定义了脉振高频注入无传感器控制的数据结构和函数接口，
 * 用于低速和零速下的位置和速度估计
 */

#ifndef __HF_INJECTION_H__
#define __HF_INJECTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "pid.h"
#include "reciprocal.h"
#include "theta_calc.h"
#include "transformation.h"

/**
 * @brief 高频注入参数结构体
 */
typedef struct {
    float injection_freq;    /*!< 高频注入频率 (Hz) */
    float injection_voltage; /*!< 注入电压幅值 (V) */
    float Ld;                /*!< d轴电感 (H) */
    float Lq;                /*!< q轴电感 (H) */
    float delta_L;           /*!< 电感变化量 (H) */
    float inv_Pn;            /*!< 极对数的倒数 (1/Pn) */
} hf_injection_params_t;

/* 函数声明 */

/**
 * @brief 设置高频注入采样时间配置
 *
 * 此函数根据提供的系统时间配置结构体，设置高频注入操作的采样时间。
 *
 * @param[in] time_config 指向 SystemTimeConfig_t 结构体的指针，包含所需的采样时间设置。
 * @return 设置成功返回 true，否则返回 false。
 */
bool Hfi_Set_SampleTime(const SystemTimeConfig_t* time_config);

/**
 * @brief 设置高频注入观测器的参数
 * 
 * 此函数用于初始化高频注入观测器的相关参数，包括注入频率、注入电压、d轴电感、q轴电感以及电感差值。
 * 
 * @param params 指向高频注入参数结构体的指针，包含所需的各项参数。
 * @return 无返回值。如果参数指针为NULL，则函数直接返回。
 */
bool Hfi_Initialization(const hf_injection_params_t* params);

/**
 * @brief 设置高频注入锁相环（PLL）参数
 *
 * 此函数用于配置高频注入算法中的锁相环（PLL）相关参数。
 *
 * @param pll_params 指向PLL参数结构体的指针，包含所需的配置参数。
 */
void Hfi_Set_PidParams(const PID_Handler_t* pid_params);

void Hfi_Set_Current(Clark_t current);

void Hfi_Set_Enabled(bool enabled);

bool Hfi_Get_Enabled(void);

void Hfi_Set_Theta(float theta);

void Hfi_Calc_ThetaErr(float ref);

void Hfi_Calc_SpeedErr(float ref);

Clark_t Hfi_Process_Current(Clark_t current);

Park_t Hfi_Get_Inject_Voltage(void);

void Hfi_Update(void);

void Hfi_Set_InitialPosition(float theta);

AngleResult_t Hfi_Get_Result(void);

float Hfi_Get_PllErr(void);

#ifdef __cplusplus
}
#endif

#endif /* __HF_INJECTION_H__ */
