
/**********************************************************************
 * @file    protect.h
 * @brief   电机保护相关头文件
 * @version 1.0
 * @date    2025-08-12
 * @author  FRECON 项目组
 *
 * @details 定义电机运行保护相关的数据结构、枚举和接口函数声明。
 *          包括过流、过压、欠压、硬件故障、过热等保护机制。
 *
 * @note    本文件仅声明接口，具体实现请参考 protect.c。
 **********************************************************************/

#ifndef PROTECT_H
#define PROTECT_H

#include <stdbool.h>
#include "transformation.h"

/*-------------------- 保护标志位定义 --------------------*/
/**
 * @brief  电机保护标志位枚举
 * @note   使用位域方式定义，便于进行位操作和状态合并
 */
typedef enum {
    No_Protect       = 0,      /* 无保护标志 */
    Over_Avg_Current = 1 << 0, /* 过流保护 */
    Over_Max_Current = 1 << 1, /* 超过最大允许电流 */
    Over_Voltage     = 1 << 2, /* 过压保护 */
    Low_Voltage      = 1 << 3, /* 欠压保护 */
    Hardware_Fault   = 1 << 4, /* 硬件故障 */
    Over_Heat        = 1 << 5  /* 过热保护 */
} Protect_Flag_t;

/*-------------------- 保护参数结构体定义 --------------------*/
/**
 * @brief  电机保护参数结构体
 * @note   用于配置保护阈值和标志位
 */
typedef struct {
    float          Udc_rate;        /* 额定母线电压值(V) */
    float          Udc_fluctuation; /* 允许的电压波动范围(V) */
    float          I_Max;           /* 最大允许电流值(A) */
    float          Temperature;     /* 温度阈值(℃) */
    Protect_Flag_t Flag;            /* 保护标志位 */
} Protect_Parameter_t;

/*-------------------- 保护功能函数声明 --------------------*/
/**
 * @brief  保护参数初始化函数
 * @param  param  保护参数结构体指针
 * @retval true   初始化成功
 * @retval false  初始化失败
 */
bool Protect_Initialization(const Protect_Parameter_t* param);

/**
 * @brief  电流保护处理函数
 * @param  current 三相电流值结构体（Phase_t 类型，包含各相电流）
 * @retval true   检测到过流
 * @retval false  电流正常
 */
bool Protect_PhaseCurrent(Phase_t current);

/**
 * @brief  母线电压保护处理函数
 * @param  bus_voltage 实际母线电压值(V)
 * @retval true   电压异常（过压或欠压）
 * @retval false  电压正常
 */
bool Protect_BusVoltage(float bus_voltage);
/**
 * @brief  检测是否过温
 * @param  temperature 当前温度(℃)
 * @retval true  过温
 * @retval false 正常
 */
bool Protect_Temperature(float temperature);

/**
 * @brief  根据温度决定是否要开启散热风扇
 * @param  temperature 当前温度(℃)
 * @retval true  风扇开启
 * @retval false 风扇关闭
 */
bool Protect_Get_FanState(float temperature);

/**
 * @brief  硬件故障保护处理
 * @param  status 硬件故障状态
 */
void Protect_HardWareFault(bool status);

/**
 * @brief  温度保护处理
 * @param  temperature 当前温度(℃)
 * @retval true  过温
 * @retval false 正常
 */
bool Protect_Temperature(float temperature);

/**
 * @brief  获取风扇状态
 * @param  temperature 当前温度(℃)
 * @retval true  风扇开启
 * @retval false 风扇关闭
 */
bool Protect_Get_FanState(float temperature);

/**
 * @brief  检查保护状态
 * @retval true   有保护标志被置位
 * @retval false  无保护标志
 */
bool Protect_Validate_Flag(void);

/**
 * @brief  清除所有保护标志
 */
void Protect_Reset_Flag(void);

#endif /* PROTECT_H */
