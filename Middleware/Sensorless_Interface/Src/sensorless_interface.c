/**
 * @file sensorless_interface.c
 * @brief 无传感器控制接口层实现
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 */

#include "sensorless_interface.h"

#include <math.h>
#include <string.h>

#include "filter.h"
#include "flux_observer.h"
#include "hf_injection.h"

/* 全局变量指针 (在main_int.c中定义实际变量) */
Sensorless_Parameter_t* g_sl_hnd_ptr = NULL;
sensorless_config_t*    g_sl_cfg_ptr = NULL;
sensorless_output_t*    g_sl_out_ptr = NULL;

/* 私有算法实例 */
static flux_observer_t g_flux_observer; /*!< 磁链观测器实例 */
static hf_injection_t  g_hf_injection;  /*!< 高频注入观测器实例 */

/* 内部变量 (Clark变换输入输出缓冲区) */
static Clark_t g_vol_ab;
static Clark_t g_cur_ab;
static Clark_t g_vol_inj_ab;

/* 速度计算相关 */
static float           g_prev_angle = 0.0f; /*!< 上一次角度 */
static LowPassFilter_t g_speed_filter;      /*!< 速度低通滤波器 */

/* 私有函数声明 */
static void sensorless_update_state(void);
static void sensorless_calculate_speed(void);
static void sensorless_validate_output(void);
static void sensorless_method_switching(void);
static int  sensorless_init_hf_injection(void);
static void sensorless_execute_flux_observer(void);
static void sensorless_execute_hf_injection(void);
static void sensorless_execute_hybrid(void);
static void sensorless_link_global_variables(Motor_Parameter_t* motor_ptr,
                                             DeviceState_t*     device_ptr);

/**
 * @brief 无传感器控制系统初始化
 */
int Sensorless_Pre_Initialize(Sensorless_Parameter_t* sensorless_ptr,
                              sensorless_config_t*    config_ptr,
                              sensorless_output_t*    output_ptr,
                              Motor_Parameter_t*      motor_ptr,
                              DeviceState_t*          device_ptr) {
    if (sensorless_ptr == NULL || config_ptr == NULL || output_ptr == NULL
        || motor_ptr == NULL || device_ptr == NULL) {
        return -1;
    }

    /* 设置全局指针 */
    g_sl_hnd_ptr = sensorless_ptr;
    g_sl_cfg_ptr = config_ptr;
    g_sl_out_ptr = output_ptr;

    /* 初始化全局结构体 */
    memset(g_sl_hnd_ptr, 0, sizeof(Sensorless_Parameter_t));
    memset(g_sl_cfg_ptr, 0, sizeof(sensorless_config_t));
    memset(g_sl_out_ptr, 0, sizeof(sensorless_output_t));

    /* 设置指针关联 */
    g_sl_hnd_ptr->config                = g_sl_cfg_ptr;
    g_sl_hnd_ptr->output                = g_sl_out_ptr;
    g_sl_hnd_ptr->vol_ab_ptr            = &g_vol_ab;
    g_sl_hnd_ptr->cur_ab_ptr            = &g_cur_ab;
    g_sl_hnd_ptr->injection_voltage_ptr = &g_vol_inj_ab;

    /* 关联全局变量 */
    sensorless_link_global_variables(motor_ptr, device_ptr);
    /* 获取默认配置 */
    Sensorless_GetDefaultConfig(g_sl_cfg_ptr);

    return 0;
}

int Sensorless_Initialize(void) {
    /* 配置磁链观测器参数 */
    // flux_observer_params_t flux_params
    //     = {.Rs = *g_sl_cfg_ptr->motor_rs_ptr,
    //        .Ls = (*g_sl_cfg_ptr->motor_ld_ptr + *g_sl_cfg_ptr->motor_lq_ptr)
    //              / 2.0f,  // 使用平均电感 .Ts =
    //              *g_sl_cfg_ptr->control_ts_ptr,
    //                       // .flux_rated =
    //        *g_sl_cfg_ptr->motor_flux_ptr,
    //        .cutoff_freq = g_sl_cfg_ptr->lpf_cutoff_freq};

    /* 初始化磁链观测器 */
    // if (flux_observer_init(&g_flux_observer, &flux_params) != 0) {
    //     return -1;
    // }

    /* 根据配置初始化高频注入 */
    if (g_sl_cfg_ptr->method == SENSORLESS_METHOD_HF_INJECTION
        || g_sl_cfg_ptr->method == SENSORLESS_METHOD_HYBRID) {
        if (sensorless_init_hf_injection() != 0) {
            return -1;
        }
    }

    /* 初始化速度滤波器 */
    LowPassFilter_t filter_init = {.alpha = 0.1f,  // 对应原来的SPEED_LPF_COEFF
                                   .prev_output = 0.0f,
                                   .initialized = false};
    LowPassFilter_Init(
        &filter_init,
        10.0F,
        *g_sl_cfg_ptr->speed_ts_ptr);  // 10Hz截止频率，10000Hz采样频率
    g_speed_filter = filter_init;

    /* 初始化输出结构 */
    g_sl_out_ptr->state         = SENSORLESS_STATE_STOPPED;
    g_sl_out_ptr->active_method = g_sl_cfg_ptr->method;
    g_sl_out_ptr->valid         = 0;

    /* 重置内部变量 */
    g_prev_angle              = 0.0f;
    g_sl_hnd_ptr->enabled     = false;
    g_sl_hnd_ptr->hf_enabled  = false;
    g_sl_hnd_ptr->initialized = true;

    return 0;
}

/**
 * @brief 无传感器控制系统复位
 */
void Sensorless_Reset(void) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return;
    }

    /* 重置磁链观测器 */
    flux_observer_reset(&g_flux_observer);

    /* 重置高频注入观测器 */
    if (g_sl_cfg_ptr->method == SENSORLESS_METHOD_HF_INJECTION
        || g_sl_cfg_ptr->method == SENSORLESS_METHOD_HYBRID) {
        HF_Injection_Reset(&g_hf_injection);
    }

    /* 重置输出 */
    memset(g_sl_out_ptr, 0, sizeof(sensorless_output_t));
    g_sl_out_ptr->state         = SENSORLESS_STATE_STOPPED;
    g_sl_out_ptr->active_method = g_sl_cfg_ptr->method;

    /* 重置中间变量 */
    g_sl_hnd_ptr->flux_alpha          = 0.0f;
    g_sl_hnd_ptr->flux_beta           = 0.0f;
    g_sl_hnd_ptr->estimated_angle     = 0.0f;
    g_sl_hnd_ptr->estimated_speed     = 0.0f;
    g_sl_hnd_ptr->hf_position         = 0.0f;
    g_sl_hnd_ptr->hf_speed            = 0.0f;
    g_sl_hnd_ptr->observer_angle      = 0.0f;
    g_sl_hnd_ptr->observer_speed      = 0.0f;
    g_sl_hnd_ptr->angle_error         = 0.0f;
    g_sl_hnd_ptr->speed_filter_output = 0.0f;
    g_sl_hnd_ptr->switch_counter      = 0;
    g_sl_hnd_ptr->convergence_counter = 0;

    /* 重置内部变量 */
    g_prev_angle               = 0.0f;
    g_speed_filter.prev_output = 0.0f;
    g_speed_filter.initialized = false;
    g_sl_hnd_ptr->enabled      = false;
    g_sl_hnd_ptr->hf_enabled   = false;
}

/**
 * @brief 无传感器控制主更新函数 (在主中断中调用)
 */
void Sensorless_Update(void) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized || !g_sl_hnd_ptr->enabled) {
        if (g_sl_out_ptr) {
            g_sl_out_ptr->valid = 0;
        }
        /* 清空注入电压 */
        if (g_sl_hnd_ptr && g_sl_hnd_ptr->injection_voltage_ptr != NULL) {
            g_sl_hnd_ptr->injection_voltage_ptr->a = 0.0f;
            g_sl_hnd_ptr->injection_voltage_ptr->b = 0.0f;
        }
        return;
    }

    /* 方法切换逻辑 */
    sensorless_method_switching();

    /* 根据当前方法执行相应算法 */
    switch (g_sl_out_ptr->active_method) {
    case SENSORLESS_METHOD_FLUX_OBSERVER:
        sensorless_execute_flux_observer();
        /* 清空注入电压 */
        if (g_sl_hnd_ptr->injection_voltage_ptr != NULL) {
            g_sl_hnd_ptr->injection_voltage_ptr->a = 0.0f;
            g_sl_hnd_ptr->injection_voltage_ptr->b = 0.0f;
        }
        break;

    case SENSORLESS_METHOD_HF_INJECTION:
        sensorless_execute_hf_injection();
        break;

    case SENSORLESS_METHOD_HYBRID:
        sensorless_execute_hybrid();
        break;

    default:
        sensorless_execute_flux_observer();
        /* 清空注入电压 */
        if (g_sl_hnd_ptr->injection_voltage_ptr != NULL) {
            g_sl_hnd_ptr->injection_voltage_ptr->a = 0.0f;
            g_sl_hnd_ptr->injection_voltage_ptr->b = 0.0f;
        }
        break;
    }

    /* 计算速度 */
    sensorless_calculate_speed();

    /* 更新状态 */
    sensorless_update_state();

    /* 验证输出有效性 */
    sensorless_validate_output();
}

/**
 * @brief 获取默认配置参数
 */
void Sensorless_GetDefaultConfig(sensorless_config_t* config) {
    // if (config == NULL) {
    //     return;
    // }

    /* 设置默认参数 */
    config->method = SENSORLESS_METHOD_HF_INJECTION; /* 默认使用高频注入方法 */

    // 前一步link的时候已经设置了
    // /* 电机参数指针将在初始化时设置 */
    // config->motor_rs_ptr = NULL;
    // config->motor_ld_ptr = NULL;
    // config->motor_lq_ptr = NULL;
    // config->motor_flux_ptr = NULL;
    // config->motor_pole_pairs_ptr = NULL;

    // /* 控制参数指针将在初始化时设置 */
    // config->control_ts_ptr = NULL;
    // config->control_freq_ptr = NULL;

    /* 滤波器参数 */
    config->lpf_cutoff_freq = 5.0f; /* 低通滤波器截止频率 */

    /* 高频注入参数 */
    config->hf_injection_freq    = 1000.0f; /* 高频注入频率 */
    config->hf_injection_voltage = 15.0f;   /* 高频注入电压 */
    config->hf_cutoff_freq_hf    = 2000.0f; /* 高频滤波器截止频率 */
    config->hf_cutoff_freq_lf    = 10.0f;   /* 低频滤波器截止频率 */

    /* 阈值参数 */
    config->min_speed_threshold = 10.0f;  /* 最小速度阈值 */
    config->min_flux_threshold  = 0.001f; /* 最小磁链阈值 */
    config->hf_switch_speed     = 50.0f;  /* 高频注入切换速度阈值 */
}

/**
 * @brief 设置无传感器控制方法
 */
int Sensorless_SetMethod(sensorless_method_t method) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return -1;
    }

    g_sl_cfg_ptr->method        = method;
    g_sl_out_ptr->active_method = method;

    /* 如果切换到高频注入或混合方法，需要初始化高频注入 */
    if ((method == SENSORLESS_METHOD_HF_INJECTION
         || method == SENSORLESS_METHOD_HYBRID)
        && !g_sl_hnd_ptr->hf_enabled) {
        return sensorless_init_hf_injection();
    }

    return 0;
}

/**
 * @brief 强制切换到指定方法
 */
int Sensorless_SwitchMethod(sensorless_method_t method, bool force) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return -1;
    }

    if (force) {
        /* 强制切换 */
        g_sl_out_ptr->active_method = method;
        g_sl_hnd_ptr->switch_counter++;
        return 0;
    } else {
        /* 按条件切换 - 在sensorless_method_switching()中处理 */
        g_sl_cfg_ptr->method = method;
        return 0;
    }
}

/**
 * @brief 使能/禁用高频注入
 */
void Sensorless_HfInjectionEnable(bool enable) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return;
    }

    g_sl_hnd_ptr->hf_enabled = enable;

    if (g_sl_cfg_ptr->method == SENSORLESS_METHOD_HF_INJECTION
        || g_sl_cfg_ptr->method == SENSORLESS_METHOD_HYBRID) {
        HF_Injection_Enable(&g_hf_injection, enable ? 1 : 0);
    }
}

/**
 * @brief 检查高频注入是否收敛
 */
bool Sensorless_HfInjectionIsConverged(void) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized
        || !g_sl_hnd_ptr->hf_enabled) {
        return false;
    }

    bool converged = (HF_Injection_IsConverged(&g_hf_injection) != 0);
    g_sl_out_ptr->hf_converged = converged ? 1 : 0;
    return converged;
}

/**
 * @brief 设置高频注入初始位置
 */
void Sensorless_HfInjectionSetInitialPosition(float initial_angle) {
    if (!g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return;
    }

    if (g_sl_cfg_ptr->method == SENSORLESS_METHOD_HF_INJECTION
        || g_sl_cfg_ptr->method == SENSORLESS_METHOD_HYBRID) {
        HF_Injection_SetInitialPosition(&g_hf_injection, initial_angle);
    }
}

/**
 * @brief 更新配置参数
 */
int Sensorless_UpdateConfig(const sensorless_config_t* config) {
    if (config == NULL || !g_sl_hnd_ptr || !g_sl_hnd_ptr->initialized) {
        return -1;
    }

    /* 保存新配置 */
    memcpy(g_sl_cfg_ptr, config, sizeof(sensorless_config_t));

    /* 更新磁链观测器参数 */
    if (g_sl_cfg_ptr->motor_rs_ptr && g_sl_cfg_ptr->motor_ld_ptr
        && g_sl_cfg_ptr->motor_lq_ptr && g_sl_cfg_ptr->motor_flux_ptr
        && g_sl_cfg_ptr->control_ts_ptr) {
        flux_observer_params_t flux_params
            = {.Rs = *g_sl_cfg_ptr->motor_rs_ptr,
               .Ls = (*g_sl_cfg_ptr->motor_ld_ptr + *g_sl_cfg_ptr->motor_lq_ptr)
                     / 2.0f,
               .Ts          = *g_sl_cfg_ptr->control_ts_ptr,
               .flux_rated  = *g_sl_cfg_ptr->motor_flux_ptr,
               .cutoff_freq = g_sl_cfg_ptr->lpf_cutoff_freq};

        return flux_observer_set_params(&g_flux_observer, &flux_params);
    }

    return 0;
}

/* 私有函数实现 */

/**
 * @brief 关联全局变量指针
 */
static void sensorless_link_global_variables(Motor_Parameter_t* motor_ptr,
                                             DeviceState_t*     device_ptr) {
    /* 关联电机参数 */
    g_sl_cfg_ptr->motor_rs_ptr         = &motor_ptr->Rs;
    g_sl_cfg_ptr->motor_ld_ptr         = &motor_ptr->Ld;
    g_sl_cfg_ptr->motor_lq_ptr         = &motor_ptr->Lq;
    g_sl_cfg_ptr->motor_flux_ptr       = &motor_ptr->Flux;
    g_sl_cfg_ptr->motor_pole_pairs_ptr = &motor_ptr->Pn;

    /* 关联设备参数 */
    g_sl_cfg_ptr->control_ts_ptr   = &device_ptr->main_Ts;
    g_sl_cfg_ptr->control_freq_ptr = &device_ptr->main_Freq;
    g_sl_cfg_ptr->speed_ts_ptr     = &device_ptr->speed_Ts;
    g_sl_cfg_ptr->speed_freq_ptr   = &device_ptr->speed_Freq;
}

/**
 * @brief 更新系统状态
 */
static void sensorless_update_state(void) {
    if (!g_sl_hnd_ptr->enabled) {
        g_sl_out_ptr->state = SENSORLESS_STATE_STOPPED;
    } else {
        /* 根据磁链幅值和速度判断状态 */
        if (g_sl_out_ptr->flux_magnitude < g_sl_cfg_ptr->min_flux_threshold) {
            g_sl_out_ptr->state = SENSORLESS_STATE_STARTING;
        } else if (fabsf(g_sl_out_ptr->rotor_speed)
                   < g_sl_cfg_ptr->min_speed_threshold) {
            g_sl_out_ptr->state = SENSORLESS_STATE_STARTING;
        } else {
            g_sl_out_ptr->state = SENSORLESS_STATE_RUNNING;
        }
    }
}

/**
 * @brief 计算转子速度
 */
static void sensorless_calculate_speed(void) {
    float angle_diff = g_sl_out_ptr->rotor_angle - g_prev_angle;

    /* 处理角度跳变（-π到π的跳变） */
    if (angle_diff > M_PI) {
        angle_diff -= 2.0f * M_PI;
    } else if (angle_diff < -M_PI) {
        angle_diff += 2.0f * M_PI;
    }

    /* 计算瞬时角速度 */
    float instant_speed = angle_diff * (*g_sl_cfg_ptr->control_freq_ptr);

    /* 低通滤波 */
    float filtered_speed = LowPassFilter_Update(&g_speed_filter, instant_speed);

    /* 更新输出 */
    g_sl_out_ptr->rotor_speed = filtered_speed;
    g_sl_out_ptr->rotor_speed_mech
        = filtered_speed / (*g_sl_cfg_ptr->motor_pole_pairs_ptr);

    /* 更新中间变量 */
    g_sl_hnd_ptr->estimated_speed     = filtered_speed;
    g_sl_hnd_ptr->speed_filter_output = filtered_speed;

    /* 保存当前角度 */
    g_prev_angle = g_sl_out_ptr->rotor_angle;
}

/**
 * @brief 验证输出有效性
 */
static void sensorless_validate_output(void) {
    /* 检查磁链幅值 */
    if (g_sl_out_ptr->flux_magnitude
        < g_sl_cfg_ptr->min_flux_threshold * 0.1f) {
        g_sl_out_ptr->valid = 0;
        return;
    }

    /* 检查角度范围 */
    if (g_sl_out_ptr->rotor_angle < 0.0f
        || g_sl_out_ptr->rotor_angle > 2.0f * M_PI) {
        g_sl_out_ptr->valid = 0;
        return;
    }

    /* 检查速度合理性 */
    if (fabsf(g_sl_out_ptr->rotor_speed) > 10000.0f) { /* 假设最大速度限制 */
        g_sl_out_ptr->valid = 0;
        return;
    }

    /* 如果所有检查通过 */
    g_sl_out_ptr->valid
        = (g_sl_out_ptr->state == SENSORLESS_STATE_RUNNING) ? 1 : 0;
}

/**
 * @brief 初始化高频注入观测器
 */
static int sensorless_init_hf_injection(void) {
    /* 检查指针有效性 */
    if (!g_sl_cfg_ptr->motor_ld_ptr || !g_sl_cfg_ptr->motor_lq_ptr
        || !g_sl_cfg_ptr->control_ts_ptr) {
        return -1;
    }

    /* 配置高频注入参数 */
    hf_injection_params_t hf_params
        = {.injection_freq    = g_sl_cfg_ptr->hf_injection_freq,
           .injection_voltage = g_sl_cfg_ptr->hf_injection_voltage,
           .Ts                = *g_sl_cfg_ptr->control_ts_ptr,
           .Ld                = *g_sl_cfg_ptr->motor_ld_ptr,
           .Lq                = *g_sl_cfg_ptr->motor_lq_ptr,
           .delta_L = *g_sl_cfg_ptr->motor_ld_ptr - *g_sl_cfg_ptr->motor_lq_ptr,
           .cutoff_freq_hf  = g_sl_cfg_ptr->hf_cutoff_freq_hf,
           .cutoff_freq_lf  = g_sl_cfg_ptr->hf_cutoff_freq_lf,
           .speed_threshold = g_sl_cfg_ptr->hf_switch_speed};

    /* 初始化高频注入观测器 */
    if (HF_Injection_Init(&g_hf_injection, &hf_params) != 0) {
        return -1;
    }

    g_sl_hnd_ptr->hf_enabled = true;
    return 0;
}

/**
 * @brief 执行磁链观测器方法
 */
static void sensorless_execute_flux_observer(void) {
    if (g_sl_hnd_ptr->vol_ab_ptr == NULL || g_sl_hnd_ptr->cur_ab_ptr == NULL) {
        return;
    }

    /* 执行磁链观测 */
    flux_observer_execute(
        &g_flux_observer, g_sl_hnd_ptr->vol_ab_ptr, g_sl_hnd_ptr->cur_ab_ptr);

    /* 获取磁链观测结果 */
    flux_observer_get_flux(
        &g_flux_observer, &g_sl_hnd_ptr->flux_alpha, &g_sl_hnd_ptr->flux_beta);
    flux_observer_get_polar(&g_flux_observer,
                            &g_sl_out_ptr->flux_magnitude,
                            &g_sl_out_ptr->flux_angle);

    /* 更新转子角度（对于表贴式PMSM，磁链角度近似等于转子角度） */
    g_sl_out_ptr->rotor_angle     = g_sl_out_ptr->flux_angle;
    g_sl_hnd_ptr->estimated_angle = g_sl_out_ptr->flux_angle;
    g_sl_hnd_ptr->observer_angle  = g_sl_out_ptr->flux_angle;

    g_sl_out_ptr->hf_converged = 0;
}

/**
 * @brief 执行高频注入方法
 */
static void sensorless_execute_hf_injection(void) {
    if (!g_sl_hnd_ptr->hf_enabled || g_sl_hnd_ptr->cur_ab_ptr == NULL) {
        return;
    }

    /* 生成高频注入信号 */
    Clark_t v_inj = {0};
    HF_Injection_GenerateSignal(&g_hf_injection, &v_inj);

    if (g_sl_hnd_ptr->injection_voltage_ptr != NULL) {
        /* 将基础电压和高频注入电压相加 */
        if (g_sl_hnd_ptr->vol_ab_ptr != NULL) {
            g_sl_hnd_ptr->injection_voltage_ptr->a
                = g_sl_hnd_ptr->vol_ab_ptr->a + v_inj.a;
            g_sl_hnd_ptr->injection_voltage_ptr->b
                = g_sl_hnd_ptr->vol_ab_ptr->b + v_inj.b;
        } else {
            g_sl_hnd_ptr->injection_voltage_ptr->a = v_inj.a;
            g_sl_hnd_ptr->injection_voltage_ptr->b = v_inj.b;
        }
    }

    /* 处理高频电流响应 */
    HF_Injection_ProcessResponse(&g_hf_injection, g_sl_hnd_ptr->cur_ab_ptr);

    /* 获取估计结果 */
    g_sl_out_ptr->rotor_angle     = HF_Injection_GetPosition(&g_hf_injection);
    g_sl_hnd_ptr->hf_position     = g_sl_out_ptr->rotor_angle;
    g_sl_hnd_ptr->hf_speed        = HF_Injection_GetSpeed(&g_hf_injection);
    g_sl_hnd_ptr->estimated_angle = g_sl_out_ptr->rotor_angle;

    g_sl_out_ptr->hf_converged = HF_Injection_IsConverged(&g_hf_injection);
    if (g_sl_out_ptr->hf_converged) {
        g_sl_hnd_ptr->convergence_counter++;
    }

    /* 高频注入时磁链信息不可用 */
    g_sl_out_ptr->flux_magnitude = 0.0f;
    g_sl_out_ptr->flux_angle     = g_sl_out_ptr->rotor_angle;
    g_sl_hnd_ptr->flux_alpha     = 0.0f;
    g_sl_hnd_ptr->flux_beta      = 0.0f;
}

/**
 * @brief 执行混合方法
 */
static void sensorless_execute_hybrid(void) {
    float current_speed = fabsf(g_sl_out_ptr->rotor_speed);

    if (current_speed < g_sl_cfg_ptr->hf_switch_speed) {
        /* 低速使用高频注入 */
        sensorless_execute_hf_injection();
    } else {
        /* 高速使用磁链观测器 */
        sensorless_execute_flux_observer();
        if (g_sl_hnd_ptr->injection_voltage_ptr != NULL) {
            g_sl_hnd_ptr->injection_voltage_ptr->a = 0.0f;
            g_sl_hnd_ptr->injection_voltage_ptr->b = 0.0f;
        }
    }
}

/**
 * @brief 方法切换逻辑
 */
static void sensorless_method_switching(void) {
    if (g_sl_cfg_ptr->method != SENSORLESS_METHOD_HYBRID) {
        /* 非混合方法，直接设置为配置的方法 */
        g_sl_out_ptr->active_method = g_sl_cfg_ptr->method;
        return;
    }

    float current_speed = fabsf(g_sl_out_ptr->rotor_speed);

    /* 混合方法的切换逻辑 */
    if (current_speed < g_sl_cfg_ptr->hf_switch_speed * 0.8f) /* 20%滞环 */
    {
        /* 切换到高频注入 */
        if (g_sl_out_ptr->active_method != SENSORLESS_METHOD_HF_INJECTION) {
            /* 如果高频注入未初始化，先初始化 */
            if (!g_sl_hnd_ptr->hf_enabled) {
                sensorless_init_hf_injection();
            }
            HF_Injection_Enable(&g_hf_injection, 1);
            g_sl_out_ptr->active_method = SENSORLESS_METHOD_HF_INJECTION;
            g_sl_hnd_ptr->switch_counter++;
        }
    } else if (current_speed
               > g_sl_cfg_ptr->hf_switch_speed * 1.2f) /* 20%滞环 */
    {
        /* 切换到磁链观测器 */
        if (g_sl_out_ptr->active_method != SENSORLESS_METHOD_FLUX_OBSERVER) {
            if (g_sl_hnd_ptr->hf_enabled) {
                HF_Injection_Enable(&g_hf_injection, 0);
            }
            g_sl_out_ptr->active_method = SENSORLESS_METHOD_FLUX_OBSERVER;
            g_sl_hnd_ptr->switch_counter++;
        }
    }
}