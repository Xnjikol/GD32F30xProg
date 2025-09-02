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

#include <stdbool.h>
#include "reciprocal.h"
#include "theta_calc.h"
#include "transformation.h"

/**
 * @brief 无传感器控制方法枚举
 */
typedef enum {
    SENSORLESS_METHOD_NONE         = 0, /*!< 无无传感器控制 */
    SENSORLESS_METHOD_SM_OBSERVER  = 1, /*!< 磁链观测器 */
    SENSORLESS_METHOD_HF_INJECTION = 2, /*!< 高频注入 */
    SENSORLESS_METHOD_PSI_OBSERVER = 4  /*!< 磁链观测器 */
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

typedef struct {
    float hysteresis;    // 滞环宽度
    float switch_speed;  // 切换速度
} Sensorless_Param_t;

bool Sensorless_Initialization(const Sensorless_Param_t* param);

bool Sensorless_Set_Voltage(Clark_t voltage);

bool Sensorless_Set_Current(Clark_t current);

void Sensorless_Set_SpeedFdbk(float fdbk);

void Sensorless_Set_SpeedRef(float ref);

void Sensorless_Set_Angle(float angle);

bool Sensorless_Update_Err(AngleResult_t result);

AngleResult_t Sensorless_UpdatePosition(void);

bool Sensorless_Calculate(void);

Clark_t Sensorless_FilterCurrent(Clark_t current);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORLESS_INTERFACE_H__ */