/**
 * @file hf_injection.h
 * @brief 脉振高频注入无传感器控制头文件
 * @author FRECON
 * @date 2025年7月25日
 * @version 1.0
 *
 * 该文件定义了脉振高频注入无传感器控制的数据结构和函数接口，
 * 用于低速和零速下的位置和速度估计
 */

#ifndef __HF_INJECTION_H__
#define __HF_INJECTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <math.h>

#include "filter.h"
#include "theta_calc.h"
#include "transformation.h"
#include "signal.h"

/**
 * @brief 高频注入参数结构体
 */
typedef struct
{
    float injection_freq;    /*!< 高频注入频率 (Hz) */
    float injection_voltage; /*!< 注入电压幅值 (V) */
    float Ts;               /*!< 采样周期 (s) */
    float Ld;               /*!< d轴电感 (H) */
    float Lq;               /*!< q轴电感 (H) */
    float delta_L;          /*!< 电感差值 Ld-Lq (H) */
    float cutoff_freq_hf;   /*!< 高频滤波器截止频率 (Hz) */
    float cutoff_freq_lf;   /*!< 低频滤波器截止频率 (Hz) */
    float speed_threshold;  /*!< 切换到高频注入的速度阈值 (rad/s) */
} hf_injection_params_t;

/**
 * @brief 高频注入状态结构体
 */
typedef struct
{
    /* 输入量 */
    Clark_t* current_ab;    /*!< αβ轴电流 (A) */
    float theta_est;              /*!< 估计的转子位置 (rad) */
    
    /* 高频注入信号 */
    Park_t v_hf_dq;          /*!< dq轴高频注入电压 (V) */
    Clark_t v_hf_ab;        /*!< αβ轴高频注入电压 (V) */

    /* 高频电流响应 */
    Park_t i_hf_dq;          /*!< dq轴高频电流 (A) */
    Clark_t i_hf_ab;        /*!< αβ轴高频电流 (A) */

    /* 位置误差信号 */
    float epsilon;                /*!< 位置误差信号 */
    float epsilon_filtered;       /*!< 滤波后的位置误差信号 */
    
    /* 估计输出 */
    float theta_hf;               /*!< 高频注入估计位置 (rad) */
    float omega_hf;               /*!< 高频注入估计速度 (rad/s) */
    
    /* 内部状态 */
    SineWave_t hf_sine_gen;       /*!< 高频正弦波生成器 */
    float hf_phase_current;       /*!< 当前高频相位值 (rad) */
    float theta_integral;         /*!< 位置积分值 (rad) */
    float theta_prev;             /*!< 前一次位置估计值 (rad) */
    
    /* 滤波器 */
    LowPassFilter_t* lpf_epsilon;      /*!< 位置误差低通滤波器 */
    BandPassFilter_t* bpf_current;     /*!< 电流带通滤波器 */
    HighPassFilter_t* hpf_current;     /*!< 电流高通滤波器 */
    
    /* PI控制器用于位置跟踪 */
    float kp_track;               /*!< 位置跟踪比例增益 */
    float ki_track;               /*!< 位置跟踪积分增益 */
    float integral_track;         /*!< 位置跟踪积分项 */
    
    /* 状态标志 */
    uint8_t is_enabled;           /*!< 高频注入使能标志 */
    uint8_t is_converged;         /*!< 收敛标志 */
    
} hf_injection_state_t;

/**
 * @brief 高频注入控制器结构体
 */
typedef struct
{
    hf_injection_params_t params; /*!< 参数 */
    hf_injection_state_t state;   /*!< 状态 */
} hf_injection_t;

/* 函数声明 */

/**
 * @brief 初始化高频注入观测器
 * @param hf_inj 高频注入观测器指针
 * @param params 参数结构体指针
 * @retval 0: 成功, -1: 失败
 */
int HF_Injection_Init(hf_injection_t* hf_inj, const hf_injection_params_t* params);

/**
 * @brief 反初始化高频注入观测器
 * @param hf_inj 高频注入观测器指针
 */
void HF_Injection_DeInit(hf_injection_t* hf_inj);

/**
 * @brief 生成高频注入信号
 * @param hf_inj 高频注入观测器指针
 * @param v_inj_ab 输出的αβ轴注入电压指针
 */
void HF_Injection_GenerateSignal(hf_injection_t* hf_inj, Clark_t* v_inj_ab);

/**
 * @brief 处理高频电流响应并估计位置
 * @param hf_inj 高频注入观测器指针
 * @param current_ab αβ轴电流
 */
void HF_Injection_ProcessResponse(hf_injection_t* hf_inj, const Clark_t* current_ab);

/**
 * @brief 获取估计的转子位置
 * @param hf_inj 高频注入观测器指针
 * @retval 估计的转子位置 (rad)
 */
float HF_Injection_GetPosition(const hf_injection_t* hf_inj);

/**
 * @brief 获取估计的转子速度
 * @param hf_inj 高频注入观测器指针
 * @retval 估计的转子速度 (rad/s)
 */
float HF_Injection_GetSpeed(const hf_injection_t* hf_inj);

/**
 * @brief 使能/禁用高频注入
 * @param hf_inj 高频注入观测器指针
 * @param enable 使能标志 (1: 使能, 0: 禁用)
 */
void HF_Injection_Enable(hf_injection_t* hf_inj, uint8_t enable);

/**
 * @brief 检查高频注入是否收敛
 * @param hf_inj 高频注入观测器指针
 * @retval 1: 已收敛, 0: 未收敛
 */
uint8_t HF_Injection_IsConverged(const hf_injection_t* hf_inj);

/**
 * @brief 重置高频注入观测器状态
 * @param hf_inj 高频注入观测器指针
 */
void HF_Injection_Reset(hf_injection_t* hf_inj);

/**
 * @brief 设置初始位置
 * @param hf_inj 高频注入观测器指针
 * @param initial_theta 初始位置 (rad)
 */
void HF_Injection_SetInitialPosition(hf_injection_t* hf_inj, float initial_theta);

/**
 * @brief 更新控制器参数
 * @param hf_inj 高频注入观测器指针
 * @param params 新的参数结构体指针
 */
void HF_Injection_UpdateParams(hf_injection_t* hf_inj, const hf_injection_params_t* params);

#ifdef __cplusplus
}
#endif

#endif /* __HF_INJECTION_H__ */
