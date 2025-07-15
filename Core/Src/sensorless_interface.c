#include "sensorless_interface.h"
#include <stdio.h>
#include <string.h>

/*======================*/
/*   Global Variables   */
/*======================*/
SensorlessInterface_t SensorlessInterface;

/* 回调函数指针 */
static SensorlessCallback_t position_callback = NULL;
static SensorlessCallback_t speed_callback = NULL;
static SensorlessCallback_t fault_callback = NULL;

/*======================*/
/*   Private Functions  */
/*======================*/
static void SensorlessInterface_UpdateStatus(void);
static void SensorlessInterface_FaultDetection(void);
static void SensorlessInterface_PerformanceMonitor(void);
static float SensorlessInterface_CalculateConfidence(void);

/*======================*/
/*   User Interface     */
/*======================*/

/**
 * @brief 无位置传感器接口初始化
 */
void SensorlessInterface_Init(void)
{
    memset(&SensorlessInterface, 0, sizeof(SensorlessInterface_t));
    
    /* 设置默认配置 */
    SensorlessInterface.algorithm = SENSORLESS_DEFAULT_ALGORITHM;
    SensorlessInterface.enabled = SENSORLESS_ENABLED;
    SensorlessInterface.auto_switch = SENSORLESS_AUTO_SWITCH_ENABLED;
    SensorlessInterface.motor_state = MOTOR_STOP;
    SensorlessInterface.estimation_confidence = 0.0f;
    SensorlessInterface.fault_flags = SENSORLESS_FAULT_NONE;
    
    /* 初始化底层无位置传感器算法 */
    Sensorless_Init();
    
    if (SensorlessInterface.enabled) {
        Sensorless_SetAlgorithm(SensorlessInterface.algorithm);
        Sensorless.enabled = 1;
    }
}

/**
 * @brief 无位置传感器接口更新
 */
void SensorlessInterface_Update(void)
{
    if (!SensorlessInterface.enabled) {
        return;
    }
    
    /* 更新状态信息 */
    SensorlessInterface_UpdateStatus();
    
    /* 故障检测 */
    SensorlessInterface_FaultDetection();
    
    /* 性能监控 */
    SensorlessInterface_PerformanceMonitor();
    
    /* 调用回调函数 */
    if (position_callback) {
        position_callback(&SensorlessInterface);
    }
    
    if (speed_callback) {
        speed_callback(&SensorlessInterface);
    }
    
    /* 故障回调 */
    if (SensorlessInterface.fault_flags != SENSORLESS_FAULT_NONE && fault_callback) {
        fault_callback(&SensorlessInterface);
    }
}

/**
 * @brief 启用/禁用无位置传感器
 * @param enable 1-启用，0-禁用
 */
void SensorlessInterface_Enable(uint8_t enable)
{
    SensorlessInterface.enabled = enable;
    Sensorless.enabled = enable;
    
    if (!enable) {
        SensorlessInterface.motor_state = MOTOR_STOP;
        SensorlessInterface.fault_flags = SENSORLESS_FAULT_NONE;
    }
}

/**
 * @brief 设置无位置传感器算法
 * @param algorithm 算法类型
 */
void SensorlessInterface_SetAlgorithm(SensorlessAlgorithm_t algorithm)
{
    SensorlessInterface.algorithm = algorithm;
    Sensorless_SetAlgorithm(algorithm);
}

/**
 * @brief 启动序列
 */
void SensorlessInterface_StartupSequence(void)
{
    if (!SensorlessInterface.enabled) {
        return;
    }
    
    /* 启动状态转换 */
    SensorlessInterface.motor_state = MOTOR_STARTUP;
    Sensorless_StateTransition(MOTOR_STARTUP);
    
#if SENSORLESS_STARTUP_USE_TWELVE_PULSE
    /* 使用十二脉冲确定初始位置 */
    TwelvePulse_Start(&Sensorless.twelve_pulse);
#endif
}

/**
 * @brief 重置无位置传感器
 */
void SensorlessInterface_Reset(void)
{
    SensorlessInterface.fault_flags = SENSORLESS_FAULT_NONE;
    SensorlessInterface.fault_counter = 0.0f;
    SensorlessInterface.position_error = 0.0f;
    SensorlessInterface.speed_error = 0.0f;
    SensorlessInterface.estimation_confidence = 0.0f;
    
    /* 重新初始化算法 */
    Sensorless_Init();
    if (SensorlessInterface.enabled) {
        Sensorless_SetAlgorithm(SensorlessInterface.algorithm);
        Sensorless.enabled = 1;
    }
}

/*======================*/
/*   Configuration      */
/*======================*/

/**
 * @brief 配置高频注入参数
 * @param freq 注入频率 (Hz)
 * @param voltage 注入电压 (V)
 */
void SensorlessInterface_ConfigHFI(float freq, float voltage)
{
    Sensorless.hfi.freq = freq;
    Sensorless.hfi.voltage = voltage;
    
    /* 重新初始化HFI滤波器 */
    LowPassFilter_Init(&Sensorless.hfi.lpf_d, freq * 0.2f, T_2K_HZ);
    LowPassFilter_Init(&Sensorless.hfi.lpf_q, freq * 0.2f, T_2K_HZ);
    BandPassFilter_Init(&Sensorless.hfi.bpf_d, freq, freq * 0.2f, T_2K_HZ);
    BandPassFilter_Init(&Sensorless.hfi.bpf_q, freq, freq * 0.2f, T_2K_HZ);
}

/**
 * @brief 配置滑膜观测器参数
 * @param gain 滑膜增益
 * @param cutoff_freq 滤波器截止频率 (Hz)
 */
void SensorlessInterface_ConfigSMO(float gain, float cutoff_freq)
{
    Sensorless.smo.gain_k = gain;
    
    /* 重新初始化SMO滤波器 */
    LowPassFilter_Init(&Sensorless.smo.lpf_ealpha, cutoff_freq, T_2K_HZ);
    LowPassFilter_Init(&Sensorless.smo.lpf_ebeta, cutoff_freq, T_2K_HZ);
}

/**
 * @brief 配置PLL参数
 * @param kp 比例增益
 * @param ki 积分增益
 */
void SensorlessInterface_ConfigPLL(float kp, float ki)
{
    Sensorless.pll.kp = kp;
    Sensorless.pll.ki = ki;
}

/**
 * @brief 配置算法切换阈值
 * @param low_to_high 从低速算法切换到高速算法的阈值 (rpm)
 * @param high_to_low 从高速算法切换到低速算法的阈值 (rpm)
 */
void SensorlessInterface_ConfigSwitchThreshold(float low_to_high, float high_to_low)
{
    Sensorless.speed_threshold_up = low_to_high;
    Sensorless.speed_threshold_down = high_to_low;
}

/*======================*/
/*   Status Functions   */
/*======================*/

/**
 * @brief 获取估计的转子位置
 * @return 转子位置 (rad)
 */
float SensorlessInterface_GetTheta(void)
{
    return SensorlessInterface.theta_est;
}

/**
 * @brief 获取估计的转子速度
 * @return 转子速度 (rpm)
 */
float SensorlessInterface_GetSpeed(void)
{
    return SensorlessInterface.speed_est;
}

/**
 * @brief 获取电机状态
 * @return 电机状态
 */
MotorState_t SensorlessInterface_GetMotorState(void)
{
    return SensorlessInterface.motor_state;
}

/**
 * @brief 获取当前激活的算法
 * @return 算法类型
 */
SensorlessAlgorithm_t SensorlessInterface_GetActiveAlgorithm(void)
{
    return Sensorless.algorithm;
}

/**
 * @brief 获取故障标志
 * @return 故障标志
 */
uint8_t SensorlessInterface_GetFaultFlags(void)
{
    return SensorlessInterface.fault_flags;
}

/*======================*/
/*   Diagnostic Functions */
/*======================*/

/**
 * @brief 获取位置估计误差
 * @return 位置误差 (rad)
 */
float SensorlessInterface_GetPositionError(void)
{
    return SensorlessInterface.position_error;
}

/**
 * @brief 获取速度估计误差
 * @return 速度误差 (rpm)
 */
float SensorlessInterface_GetSpeedError(void)
{
    return SensorlessInterface.speed_error;
}

/**
 * @brief 获取估计置信度
 * @return 置信度 (0.0-1.0)
 */
float SensorlessInterface_GetEstimationConfidence(void)
{
    return SensorlessInterface.estimation_confidence;
}

/**
 * @brief 清除故障标志
 */
void SensorlessInterface_ClearFaults(void)
{
    SensorlessInterface.fault_flags = SENSORLESS_FAULT_NONE;
    SensorlessInterface.fault_counter = 0.0f;
}

/*======================*/
/*   Debug Functions    */
/*======================*/

/**
 * @brief 获取调试信息
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 */
void SensorlessInterface_GetDebugInfo(char *buffer, uint16_t buffer_size)
{
    snprintf(buffer, buffer_size,
        "Sensorless Debug Info:\n"
        "Algorithm: %d\n"
        "Theta: %.3f rad\n"
        "Speed: %.1f rpm\n"
        "Motor State: %d\n"
        "HFI Active: %d\n"
        "SMO Active: %d\n"
        "Confidence: %.2f\n"
        "Faults: 0x%02X\n",
        SensorlessInterface.algorithm,
        SensorlessInterface.theta_est,
        SensorlessInterface.speed_est,
        SensorlessInterface.motor_state,
        SensorlessInterface.hfi_active,
        SensorlessInterface.smo_active,
        SensorlessInterface.estimation_confidence,
        SensorlessInterface.fault_flags
    );
}

/**
 * @brief 强制使用指定算法（调试用）
 * @param algorithm 算法类型
 */
void SensorlessInterface_ForceAlgorithm(SensorlessAlgorithm_t algorithm)
{
    SensorlessInterface.auto_switch = 0;
    SensorlessInterface_SetAlgorithm(algorithm);
}

/**
 * @brief 测试模式
 * @param enable 1-启用测试模式，0-禁用
 */
void SensorlessInterface_TestMode(uint8_t enable)
{
    /* 测试模式下可以放宽一些限制条件 */
    static float original_thresholds[2];
    
    if (enable) {
        /* 保存原始阈值 */
        original_thresholds[0] = Sensorless.speed_threshold_up;
        original_thresholds[1] = Sensorless.speed_threshold_down;
        
        /* 设置测试阈值 */
        Sensorless.speed_threshold_up = 100.0f;
        Sensorless.speed_threshold_down = 80.0f;
    } else {
        /* 恢复原始阈值 */
        Sensorless.speed_threshold_up = original_thresholds[0];
        Sensorless.speed_threshold_down = original_thresholds[1];
    }
}

/*======================*/
/*   Callback Functions */
/*======================*/

/**
 * @brief 设置位置回调函数
 * @param callback 回调函数指针
 */
void SensorlessInterface_SetPositionCallback(SensorlessCallback_t callback)
{
    position_callback = callback;
}

/**
 * @brief 设置速度回调函数
 * @param callback 回调函数指针
 */
void SensorlessInterface_SetSpeedCallback(SensorlessCallback_t callback)
{
    speed_callback = callback;
}

/**
 * @brief 设置故障回调函数
 * @param callback 回调函数指针
 */
void SensorlessInterface_SetFaultCallback(SensorlessCallback_t callback)
{
    fault_callback = callback;
}

/*======================*/
/*   Private Functions  */
/*======================*/

/**
 * @brief 更新状态信息
 */
static void SensorlessInterface_UpdateStatus(void)
{
    /* 更新基本状态 */
    SensorlessInterface.theta_est = Sensorless_GetTheta();
    SensorlessInterface.speed_est = Sensorless_GetSpeed();
    SensorlessInterface.motor_state = Sensorless.motor_state;
    
    /* 更新算法激活状态 */
    SensorlessInterface.hfi_active = Sensorless.hfi.enabled;
    SensorlessInterface.smo_active = Sensorless.smo.enabled;
    SensorlessInterface.pll_active = Sensorless.pll.enabled;
    SensorlessInterface.twelve_pulse_completed = TwelvePulse_IsCompleted(&Sensorless.twelve_pulse);
    
    /* 计算置信度 */
    SensorlessInterface.estimation_confidence = SensorlessInterface_CalculateConfidence();
}

/**
 * @brief 故障检测
 */
static void SensorlessInterface_FaultDetection(void)
{
    static uint16_t fault_counter = 0;
    
    /* 检测位置丢失 */
    if (SensorlessInterface.estimation_confidence < 0.3f) {
        fault_counter++;
        if (fault_counter > 100) {  /* 50ms */
            SensorlessInterface.fault_flags |= SENSORLESS_FAULT_POSITION_LOST;
        }
    } else {
        fault_counter = 0;
    }
    
    /* 检测速度不稳定 */
    static float last_speed = 0.0f;
    float speed_change = fabsf(SensorlessInterface.speed_est - last_speed);
    if (speed_change > 500.0f) {  /* 速度变化过大 */
        SensorlessInterface.fault_flags |= SENSORLESS_FAULT_SPEED_UNSTABLE;
    }
    last_speed = SensorlessInterface.speed_est;
    
    /* 检测启动失败 */
    if (SensorlessInterface.motor_state == MOTOR_STARTUP) {
        static uint16_t startup_counter = 0;
        startup_counter++;
        if (startup_counter > 2000) {  /* 启动超时 1s */
            SensorlessInterface.fault_flags |= SENSORLESS_FAULT_STARTUP_FAIL;
            startup_counter = 0;
        }
    }
}

/**
 * @brief 性能监控
 */
static void SensorlessInterface_PerformanceMonitor(void)
{
    /* 这里可以添加性能监控代码 */
    /* 例如：算法执行时间、内存使用情况等 */
    
    /* 更新故障计数器 */
    if (SensorlessInterface.fault_flags != SENSORLESS_FAULT_NONE) {
        SensorlessInterface.fault_counter += 1.0f;
    }
}

/**
 * @brief 计算估计置信度
 * @return 置信度 (0.0-1.0)
 */
static float SensorlessInterface_CalculateConfidence(void)
{
    float confidence = 1.0f;
    
    /* 根据当前算法调整置信度 */
    switch (Sensorless.algorithm) {
        case SENSORLESS_HFI:
            /* HFI在低速时置信度较高 */
            if (fabsf(SensorlessInterface.speed_est) < 200.0f) {
                confidence = 0.9f;
            } else {
                confidence = 0.5f;
            }
            break;
            
        case SENSORLESS_SMO:
            /* SMO在高速时置信度较高 */
            if (fabsf(SensorlessInterface.speed_est) > 150.0f) {
                confidence = 0.9f;
            } else {
                confidence = 0.3f;
            }
            break;
            
        case SENSORLESS_HYBRID:
            /* 混合算法置信度较高 */
            confidence = 0.8f;
            break;
            
        default:
            confidence = 0.0f;
            break;
    }
    
    /* 根据故障情况调整置信度 */
    if (SensorlessInterface.fault_flags != SENSORLESS_FAULT_NONE) {
        confidence *= 0.5f;
    }
    
    return confidence;
}
