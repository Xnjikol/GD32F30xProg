/**
 * @file sensorless_interface.h
 * @brief 无传感器控制接口层
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 *
 * 该文件提供无传感器控制的全局变量接口，类似FOC算法的设计模式
 */

#ifndef __SENSORLESS_INTERFACE_H__
#define __SENSORLESS_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"
#include "main.h"
#include "transformation.h"

/**
 * @brief 无传感器控制方法枚举
 */
typedef enum {
    SENSORLESS_METHOD_FLUX_OBSERVER = 0, /*!< 磁链观测器 */
    SENSORLESS_METHOD_HF_INJECTION,      /*!< 高频注入 */
    SENSORLESS_METHOD_HYBRID             /*!< 混合方法 */
} sensorless_method_t;

/**
 * @brief 无传感器控制状态枚举
 */
typedef enum {
    SENSORLESS_STATE_STOPPED = 0, /*!< 停止状态 */
    SENSORLESS_STATE_STARTING,    /*!< 启动状态 */
    SENSORLESS_STATE_RUNNING,     /*!< 运行状态 */
    SENSORLESS_STATE_ERROR        /*!< 错误状态 */
} sensorless_state_t;

/**
 * @brief 无传感器控制配置参数
 */
typedef struct {
    /* 控制方法选择 */
    sensorless_method_t method; /*!< 无传感器控制方法 */

    /* 电机参数 (指向Motor全局变量的引用) */
    float* motor_rs_ptr;         /*!< 指向定子电阻 (Ω) */
    float* motor_ld_ptr;         /*!< 指向d轴电感 (H) */
    float* motor_lq_ptr;         /*!< 指向q轴电感 (H) */
    float* motor_flux_ptr;       /*!< 指向额定磁链 (Wb) */
    float* motor_pole_pairs_ptr; /*!< 指向极对数 */

    /* 控制参数 (指向Device全局变量的引用) */
    float* control_ts_ptr;   /*!< 指向控制周期 (s) */
    float* control_freq_ptr; /*!< 指向控制频率 (Hz) */
    float* speed_ts_ptr;     /*!< 指向速度环采样周期 (s) */
    float* speed_freq_ptr;   /*!< 指向速度环频率 (Hz) */

    /* 滤波器参数 */
    float lpf_cutoff_freq; /*!< 低通滤波器截止频率 (Hz) */

    /* 高频注入参数 */
    float hf_injection_freq;    /*!< 高频注入频率 (Hz) */
    float hf_injection_voltage; /*!< 高频注入电压幅值 (V) */
    float hf_cutoff_freq_hf;    /*!< 高频滤波器截止频率 (Hz) */
    float hf_cutoff_freq_lf;    /*!< 低频滤波器截止频率 (Hz) */

    /* 阈值参数 */
    float min_speed_threshold; /*!< 最小速度阈值 (rad/s) */
    float min_flux_threshold;  /*!< 最小磁链阈值 (Wb) */
    float hf_switch_speed;     /*!< 高频注入切换速度阈值 (rad/s) */
} sensorless_config_t;

/**
 * @brief 无传感器控制输出结构体
 */
typedef struct {
    float               rotor_angle;      /*!< 转子电角度 (rad) */
    float               rotor_speed;      /*!< 转子电角速度 (rad/s) */
    float               rotor_speed_mech; /*!< 转子机械角速度 (rad/s) */
    float               flux_magnitude;   /*!< 磁链幅值 (Wb) */
    float               flux_angle;       /*!< 磁链角度 (rad) */
    sensorless_state_t  state;            /*!< 当前状态 */
    sensorless_method_t active_method;    /*!< 当前激活的方法 */
    uint8_t             valid;            /*!< 数据有效标志 */
    uint8_t             hf_converged;     /*!< 高频注入收敛标志 */
} sensorless_output_t;

/**
 * @brief 无传感器控制全局变量结构体 (类似FOC_Parameter_t)
 */
typedef struct {
    bool initialized; /*!< 初始化标志 */
    bool enabled;     /*!< 总使能标志 */
    bool hf_enabled;  /*!< 高频注入使能标志 */

    sensorless_config_t* config; /*!< 配置参数指针 */
    sensorless_output_t* output; /*!< 输出数据指针 */

    /* 输入数据指针 */
    Clark_t* vol_ab_ptr;            /*!< αβ轴电压输入指针 */
    Clark_t* cur_ab_ptr;            /*!< αβ轴电流输入指针 */
    Clark_t* injection_voltage_ptr; /*!< 高频注入电压输出指针 */

    /* 中间变量 (外部可访问) */
    float flux_alpha;      /*!< α轴磁链 */
    float flux_beta;       /*!< β轴磁链 */
    float estimated_angle; /*!< 估计角度 */
    float estimated_speed; /*!< 估计速度 */
    float hf_position;     /*!< 高频注入位置 */
    float hf_speed;        /*!< 高频注入速度 */
    float observer_angle;  /*!< 观测器角度 */
    float observer_speed;  /*!< 观测器速度 */

    /* 诊断信息 */
    float    angle_error;         /*!< 角度误差 */
    float    speed_filter_output; /*!< 速度滤波输出 */
    uint32_t switch_counter;      /*!< 方法切换计数器 */
    uint32_t convergence_counter; /*!< 收敛计数器 */
} Sensorless_Parameter_t;

/* 全局变量声明 (在main_int.c中定义) */
extern Sensorless_Parameter_t* g_sl_hnd_ptr;
extern sensorless_config_t*    g_sl_cfg_ptr;
extern sensorless_output_t*    g_sl_out_ptr;

/* 内联函数接口 (类似FOC接口) */

/**
 * @brief 更新无传感器输入电压
 * @param v_alpha α轴电压
 * @param v_beta β轴电压
 */
static inline void Sensorless_UpdateVoltage(float v_alpha, float v_beta) {
    if (g_sl_hnd_ptr && g_sl_hnd_ptr->vol_ab_ptr != NULL) {
        g_sl_hnd_ptr->vol_ab_ptr->a = v_alpha;
        g_sl_hnd_ptr->vol_ab_ptr->b = v_beta;
    }
}

/**
 * @brief 更新无传感器输入电流
 * @param i_alpha α轴电流
 * @param i_beta β轴电流
 */
static inline void Sensorless_UpdateCurrent(float i_alpha, float i_beta) {
    if (g_sl_hnd_ptr && g_sl_hnd_ptr->cur_ab_ptr != NULL) {
        g_sl_hnd_ptr->cur_ab_ptr->a = i_alpha;
        g_sl_hnd_ptr->cur_ab_ptr->b = i_beta;
    }
}

/**
 * @brief 获取估计的转子角度
 * @retval 转子电角度 (rad)
 */
static inline float Sensorless_GetRotorAngle(void) {
    return (g_sl_out_ptr) ? g_sl_out_ptr->rotor_angle : 0.0f;
}

/**
 * @brief 获取估计的转子速度
 * @retval 转子电角速度 (rad/s)
 */
static inline float Sensorless_GetRotorSpeed(void) {
    return (g_sl_out_ptr) ? g_sl_out_ptr->rotor_speed : 0.0f;
}

/**
 * @brief 获取高频注入电压
 * @param v_alpha_inj α轴注入电压输出指针
 * @param v_beta_inj β轴注入电压输出指针
 */
static inline void Sensorless_GetInjectionVoltage(float* v_alpha_inj,
                                                  float* v_beta_inj) {
    if (g_sl_hnd_ptr && g_sl_hnd_ptr->injection_voltage_ptr != NULL
        && v_alpha_inj != NULL && v_beta_inj != NULL) {
        *v_alpha_inj = g_sl_hnd_ptr->injection_voltage_ptr->a;
        *v_beta_inj  = g_sl_hnd_ptr->injection_voltage_ptr->b;
    } else if (v_alpha_inj != NULL && v_beta_inj != NULL) {
        *v_alpha_inj = 0.0f;
        *v_beta_inj  = 0.0f;
    }
}

/* Clark_t类型的重载函数 - 提供更加规范的接口 */

/**
 * @brief 更新无传感器输入电压 (Clark_t版本)
 * @param v_ab αβ轴电压结构体指针
 */
static inline void Sensorless_UpdateVoltageAB_Clark(const Clark_t* v_ab) {
    if (v_ab != NULL) {
        Sensorless_UpdateVoltage(v_ab->a, v_ab->b);
    }
}

/**
 * @brief 更新无传感器输入电流 (Clark_t版本)
 * @param i_ab αβ轴电流结构体指针
 */
static inline void Sensorless_UpdateCurrentAB_Clark(const Clark_t* i_ab) {
    if (i_ab != NULL) {
        Sensorless_UpdateCurrent(i_ab->a, i_ab->b);
    }
}

/**
 * @brief 获取高频注入电压 (Clark_t版本)
 * @param v_inj_ab αβ轴注入电压结构体指针
 */
static inline void Sensorless_GetInjectionVoltageAB_Clark(Clark_t* v_inj_ab) {
    if (v_inj_ab != NULL) {
        Sensorless_GetInjectionVoltage(&v_inj_ab->a, &v_inj_ab->b);
    }
}

/**
 * @brief 设置无传感器使能状态
 * @param enable 使能标志
 */
static inline void Sensorless_SetEnable(bool enable) {
    if (g_sl_hnd_ptr) {
        g_sl_hnd_ptr->enabled = enable;
    }
}

/**
 * @brief 检查无传感器是否有效
 * @retval true: 有效, false: 无效
 */
static inline bool Sensorless_IsValid(void) {
    return (g_sl_out_ptr) ? (g_sl_out_ptr->valid != 0) : false;
}

/**
 * @brief 检查无传感器是否已初始化
 * @retval true: 已初始化, false: 未初始化
 */
static inline bool Sensorless_IsInitialized(void) {
    return (g_sl_hnd_ptr) ? g_sl_hnd_ptr->initialized : false;
}

/**
 * @brief 获取当前状态
 * @retval 无传感器控制状态
 */
static inline sensorless_state_t Sensorless_GetState(void) {
    return (g_sl_out_ptr) ? g_sl_out_ptr->state : SENSORLESS_STATE_STOPPED;
}

/**
 * @brief 获取当前方法
 * @retval 无传感器控制方法
 */
static inline sensorless_method_t Sensorless_GetMethod(void) {
    return (g_sl_out_ptr) ? g_sl_out_ptr->active_method
                          : SENSORLESS_METHOD_FLUX_OBSERVER;
}

/* 封装的算法接口函数 */

/**
 * @brief 无传感器控制系统初始化
 * @param sensorless_ptr 无传感器控制结构体指针
 * @param config_ptr 配置参数结构体指针
 * @param output_ptr 输出数据结构体指针
 * @param motor_ptr 电机参数结构体指针
 * @param device_ptr 设备状态结构体指针
 * @retval 0: 成功, -1: 失败
 */
int Sensorless_Pre_Initialize(Sensorless_Parameter_t* sensorless_ptr,
                              sensorless_config_t*    config_ptr,
                              sensorless_output_t*    output_ptr,
                              Motor_Parameter_t*      motor_ptr,
                              DeviceState_t*          device_ptr);

int Sensorless_Initialize(void);

/**
 * @brief 无传感器控制系统复位
 */
void Sensorless_Reset(void);

/**
 * @brief 无传感器控制主更新函数 (在主中断中调用)
 */
void Sensorless_Update(void);

/**
 * @brief 获取默认配置参数
 * @param config 配置参数输出指针
 */
void Sensorless_GetDefaultConfig(sensorless_config_t* config);

/**
 * @brief 设置无传感器控制方法
 * @param method 控制方法
 * @retval 0: 成功, -1: 失败
 */
int Sensorless_SetMethod(sensorless_method_t method);

/**
 * @brief 强制切换到指定方法
 * @param method 目标方法
 * @param force 强制切换标志
 * @retval 0: 成功, -1: 失败
 */
int Sensorless_SwitchMethod(sensorless_method_t method, bool force);

/**
 * @brief 使能/禁用高频注入
 * @param enable 使能标志
 */
void Sensorless_HfInjectionEnable(bool enable);

/**
 * @brief 检查高频注入是否收敛
 * @retval true: 已收敛, false: 未收敛
 */
bool Sensorless_HfInjectionIsConverged(void);

/**
 * @brief 设置高频注入初始位置
 * @param initial_angle 初始角度 (rad)
 */
void Sensorless_HfInjectionSetInitialPosition(float initial_angle);

/**
 * @brief 更新配置参数
 * @param config 新的配置参数
 * @retval 0: 成功, -1: 失败
 */
int Sensorless_UpdateConfig(const sensorless_config_t* config);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORLESS_INTERFACE_H__ */