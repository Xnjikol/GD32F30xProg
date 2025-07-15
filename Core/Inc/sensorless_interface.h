#ifndef _SENSORLESS_INTERFACE_H_
#define _SENSORLESS_INTERFACE_H_

#include "sensorless.h"

/*======================*/
/*   Interface Config   */
/*======================*/

/* 无位置传感器配置宏 */
#define SENSORLESS_ENABLED              1       /* 1-启用无位置传感器，0-禁用 */
#define SENSORLESS_DEFAULT_ALGORITHM    SENSORLESS_HYBRID  /* 默认算法 */

/* 启动配置 */
#define SENSORLESS_STARTUP_USE_TWELVE_PULSE  1  /* 启动时使用十二脉冲 */
#define SENSORLESS_STARTUP_CURRENT_LIMIT     2.0f  /* 启动电流限制 (A) */
#define SENSORLESS_STARTUP_VOLTAGE_LIMIT     10.0f /* 启动电压限制 (V) */

/* 算法切换配置 */
#define SENSORLESS_AUTO_SWITCH_ENABLED   1      /* 自动切换算法 */
#define SENSORLESS_SWITCH_HYSTERESIS     20.0f  /* 切换迟滞 (rpm) */

/*======================*/
/*   User Interface     */
/*======================*/

/* 无位置传感器用户接口结构 */
typedef struct {
    /* 配置参数 */
    SensorlessAlgorithm_t algorithm;        /* 当前算法 */
    uint8_t enabled;                        /* 使能标志 */
    uint8_t auto_switch;                    /* 自动切换使能 */
    
    /* 状态反馈 */
    float theta_est;                        /* 估计位置 (rad) */
    float speed_est;                        /* 估计速度 (rpm) */
    MotorState_t motor_state;               /* 电机状态 */
    
    /* 算法状态 */
    uint8_t hfi_active;                     /* HFI激活状态 */
    uint8_t smo_active;                     /* SMO激活状态 */
    uint8_t pll_active;                     /* PLL激活状态 */
    uint8_t twelve_pulse_completed;         /* 十二脉冲完成状态 */
    
    /* 性能指标 */
    float position_error;                   /* 位置估计误差 */
    float speed_error;                      /* 速度估计误差 */
    float estimation_confidence;            /* 估计置信度 */
    
    /* 故障信息 */
    uint8_t fault_flags;                    /* 故障标志 */
    float fault_counter;                    /* 故障计数 */
    
} SensorlessInterface_t;

/* 故障标志定义 */
#define SENSORLESS_FAULT_NONE           0x00    /* 无故障 */
#define SENSORLESS_FAULT_POSITION_LOST  0x01    /* 位置丢失 */
#define SENSORLESS_FAULT_SPEED_UNSTABLE 0x02    /* 速度不稳定 */
#define SENSORLESS_FAULT_ALGORITHM_FAIL 0x04    /* 算法失效 */
#define SENSORLESS_FAULT_STARTUP_FAIL   0x08    /* 启动失败 */

/*======================*/
/*   Global Variables   */
/*======================*/
extern SensorlessInterface_t SensorlessInterface;

/*======================*/
/*   Function Protos    */
/*======================*/

/* 用户接口函数 */
void SensorlessInterface_Init(void);
void SensorlessInterface_Update(void);
void SensorlessInterface_Enable(uint8_t enable);
void SensorlessInterface_SetAlgorithm(SensorlessAlgorithm_t algorithm);
void SensorlessInterface_StartupSequence(void);
void SensorlessInterface_Reset(void);

/* 配置函数 */
void SensorlessInterface_ConfigHFI(float freq, float voltage);
void SensorlessInterface_ConfigSMO(float gain, float cutoff_freq);
void SensorlessInterface_ConfigPLL(float kp, float ki);
void SensorlessInterface_ConfigSwitchThreshold(float low_to_high, float high_to_low);

/* 获取状态函数 */
float SensorlessInterface_GetTheta(void);
float SensorlessInterface_GetSpeed(void);
MotorState_t SensorlessInterface_GetMotorState(void);
SensorlessAlgorithm_t SensorlessInterface_GetActiveAlgorithm(void);
uint8_t SensorlessInterface_GetFaultFlags(void);

/* 诊断函数 */
float SensorlessInterface_GetPositionError(void);
float SensorlessInterface_GetSpeedError(void);
float SensorlessInterface_GetEstimationConfidence(void);
void SensorlessInterface_ClearFaults(void);

/* 调试函数 */
void SensorlessInterface_GetDebugInfo(char *buffer, uint16_t buffer_size);
void SensorlessInterface_ForceAlgorithm(SensorlessAlgorithm_t algorithm);
void SensorlessInterface_TestMode(uint8_t enable);

/* 回调函数类型 */
typedef void (*SensorlessCallback_t)(SensorlessInterface_t *interface);

/* 回调函数设置 */
void SensorlessInterface_SetPositionCallback(SensorlessCallback_t callback);
void SensorlessInterface_SetSpeedCallback(SensorlessCallback_t callback);
void SensorlessInterface_SetFaultCallback(SensorlessCallback_t callback);

#endif /* _SENSORLESS_INTERFACE_H_ */
