#ifndef __MAIN_INT_SIMPLIFIED_H__
#define __MAIN_INT_SIMPLIFIED_H__

#include "hw_interface.h"
#include "foc_types.h"
#include "gd32f30x.h"

/**
 * @brief Motor参数初始化
 * 初始化Motor结构体的各个参数
 * @param motor Motor参数结构体指针
 * @param device 设备状态结构体指针
 */
void Motor_Init(Motor_Parameter_t* motor, DeviceState_t* device);

/**
 * @brief 主中断初始化
 * 初始化硬件接口层和相关参数
 */
void Main_Int_Init(void);

/**
 * @brief 主中断处理函数
 * 在ADC中断中调用，处理FOC控制逻辑
 */
void Main_Int_Handler(void);

/* ================ 设备状态管理函数 ================ */

/**
 * @brief 触发完整初始化
 * 将设备从WAITING状态切换到SETUP状态
 */
void Main_Int_TriggerFullInit(void);

/**
 * @brief 检查基础就绪状态
 * @return bool 是否基础就绪
 */
bool Main_Int_IsBasicReady(void);

/**
 * @brief 检查完全就绪状态
 * @return bool 是否完全就绪
 */
bool Main_Int_IsFullyReady(void);

/**
 * @brief 设置设备停止状态
 * @param stop 停止标志
 */
void Main_Int_SetDeviceStop(bool stop);

/**
 * @brief 获取设备停止状态
 * @return bool 设备停止标志
 */
bool Main_Int_GetDeviceStop(void);

/**
 * @brief 启用/禁用硬件保护
 * @param enable 启用标志
 */
void Main_Int_EnableHardwareProtect(bool enable);

/**
 * @brief 清除保护标志
 * @param flags 要清除的保护标志
 */
void Main_Int_ClearProtectFlags(HW_ProtectFlags_t flags);

/**
 * @brief 获取保护标志
 * @return HW_ProtectFlags_t 当前保护标志
 */
HW_ProtectFlags_t Main_Int_GetProtectFlags(void);

/* ================ 通信接口 ================ */

/**
 * @brief CAN发送接口
 * @param frame CAN帧结构体指针
 * @return bool 发送是否成功
 */
bool Main_Int_CANSend(const can_frame_t* frame);

/**
 * @brief CAN接收接口
 * @param frame CAN帧结构体指针
 * @return bool 接收是否成功
 */
bool Main_Int_CANReceive(can_frame_t* frame);

/**
 * @brief SCI发送接口
 * @param data 浮点数据指针
 * @param count 数据个数
 */
void Main_Int_SCISend(float* data, uint8_t count);

/* ================ 内联便利函数 ================ */

/**
 * @brief 更新FOC电流反馈（已封装在HW_Interface_Process中）
 */
static inline void FOC_UpdateCurrent(float Ia, float Ib, float Ic)
{
  /* 该函数已不需要，电流更新由HW_Interface_Process处理 */
  (void)Ia; (void)Ib; (void)Ic; /* 避免编译警告 */
}

/**
 * @brief 更新FOC电压反馈（已封装在HW_Interface_Process中）
 */
static inline void FOC_UpdateVoltage(float Udc, float inv_Udc)
{
  /* 该函数已不需要，电压更新由HW_Interface_Process处理 */
  (void)Udc; (void)inv_Udc; /* 避免编译警告 */
}

/**
 * @brief 更新FOC角度（已封装在HW_Interface_Process中）
 */
static inline void FOC_UpdateTheta(float Theta)
{
  /* 该函数已不需要，角度更新由HW_Interface_Process处理 */
  (void)Theta; /* 避免编译警告 */
}

/**
 * @brief 获取PWM输出（已封装在HW_Interface_GetPWMOutput中）
 */
static inline void FOC_OutputCompare(float* Tcm1, float* Tcm2, float* Tcm3)
{
  /* 该函数已不需要，PWM输出由HW_Interface_GetPWMOutput处理 */
  if (Tcm1) *Tcm1 = 0.0f;
  if (Tcm2) *Tcm2 = 0.0f;
  if (Tcm3) *Tcm3 = 0.0f;
}

/**
 * @brief 更新最大电流（已封装在硬件接口层中）
 */
static inline void FOC_UpdateMaxCurrent(float I_Max)
{
  /* 该函数已不需要，保护参数由硬件接口层管理 */
  (void)I_Max; /* 避免编译警告 */
}

#endif /* __MAIN_INT_SIMPLIFIED_H__ */
