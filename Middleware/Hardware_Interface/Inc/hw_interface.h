#ifndef _HW_INTERFACE_H_
#define _HW_INTERFACE_H_

#include "stdbool.h"
#include "stdint.h"
#include "foc_types.h"
#include "com_frame.h"

/* ================ 保护相关定义 ================ */
#define TEMPERATURE_THRESHOLD_DEFAULT 80.0F
#define CURRENT_THRESHOLD_DEFAULT 10.0F
#define VOLTAGE_RATE_DEFAULT 220.0F
#define VOLTAGE_FLUCTUATION_DEFAULT 40.0F

/* ================ 控制相关定义 ================ */
#define SPEED_LOOP_PRESCALER 10.0F  /* Speed loop frequency division factor */

typedef enum
{
  HW_PROTECT_NONE = 0,
  HW_PROTECT_OVER_CURRENT = 1 << 0,          
  HW_PROTECT_OVER_MAXIMUM_CURRENT = 1 << 1,  
  HW_PROTECT_OVER_VOLTAGE = 1 << 2,          
  HW_PROTECT_LOW_VOLTAGE = 1 << 3,           
  HW_PROTECT_HARDWARE_FAULT = 1 << 4,        
  HW_PROTECT_OVER_HEAT = 1 << 5              
} HW_ProtectFlags_t;

typedef struct
{
  float voltage_rate;         
  float voltage_fluctuation;  
  float current_max;          
  float temperature_max;      
  HW_ProtectFlags_t flags;    
} HW_ProtectParams_t;

/* ================ Hardware Interface 主要接口 ================ */

/**
 * @brief 硬件接口初始化
 * @param foc_param FOC参数结构体指针
 * @param motor_param 电机参数结构体指针
 * @param device_state 设备状态结构体指针
 * @return bool 初始化是否成功
 */
bool HW_Interface_Init(FOC_Parameter_t* foc_param, 
                      Motor_Parameter_t* motor_param,
                      DeviceState_t* device_state);

/**
 * @brief 硬件接口主处理函数（在主中断中调用）
 * 读取所有外设数据并更新FOC参数
 */
void HW_Interface_Process(void);

/**
 * @brief 获取PWM输出值（在主中断中调用）
 * @param tcm_phase PWM输出相位结构体指针
 * @return bool 获取是否成功
 */
bool HW_Interface_GetPWMOutput(Phase_t* tcm_phase);

/* ================ 状态管理接口 ================ */

/**
 * @brief 设置设备停止状态
 * @param stop 停止标志
 */
void HW_Interface_SetDeviceStop(bool stop);

/**
 * @brief 获取设备停止状态
 * @return bool 设备停止标志
 */
bool HW_Interface_GetDeviceStop(void);

/**
 * @brief 启用硬件保护
 */
void HW_Interface_EnableHardwareProtect(void);

/**
 * @brief 禁用硬件保护
 */
void HW_Interface_DisableHardwareProtect(void);

/**
 * @brief 获取保护标志
 * @return HW_ProtectFlags_t 当前保护标志
 */
HW_ProtectFlags_t HW_Interface_GetProtectFlags(void);

/**
 * @brief 清除指定的保护标志
 * @param flags_to_clear 要清除的保护标志
 */
void HW_Interface_ClearProtectFlags(HW_ProtectFlags_t flags_to_clear);

/* ================ 通信接口 ================ */

/**
 * @brief CAN发送接口
 * @param frame CAN帧结构体指针
 * @return bool 发送是否成功
 */
bool HW_Interface_CANSend(const can_frame_t* frame);

/**
 * @brief CAN接收接口
 * @param frame CAN帧结构体指针
 * @return bool 接收是否成功
 */
bool HW_Interface_CANReceive(can_frame_t* frame);

/**
 * @brief SCI发送接口
 * @param data 浮点数据指针
 * @param count 数据个数
 */
void HW_Interface_SCISend(float* data, uint8_t count);

/* ================ 校准和配置接口 ================ */

/**
 * @brief ADC校准
 */
void HW_Interface_CalibrateADC(void);

/**
 * @brief 获取系统频率参数
 */
void HW_Interface_GetSystemFrequency(void);

/**
 * @brief 初始化保护参数
 */
void HW_Interface_InitProtectParams(void);

/* ================ 设备状态控制接口 ================ */

/**
 * @brief 获取软件刹车状态
 * @return bool 软件刹车状态
 */
bool HW_Interface_GetSoftwareBRK(void);

/**
 * @brief 设置硬件故障标志
 */
void HW_Interface_SetHardwareFault(void);

/**
 * @brief SCI发送回调函数
 */
void HW_Interface_SCISendCallback(void);

/**
 * @brief 温度保护检查
 */
void HW_Interface_TemperatureProtect(void);

/**
 * @brief 栅极状态检查
 */
void HW_Interface_GateState(void);

#endif /* _HW_INTERFACE_H_ */
