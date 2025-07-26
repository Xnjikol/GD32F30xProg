#ifndef _FOC_H_
#define _FOC_H_

#include "filter.h"
#include "foc_types.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

/* ================ 主要FOC控制函数 ================ */

/**
 * @brief FOC系统初始化
 * @param foc FOC参数结构体指针
 * @return bool 初始化是否成功
 */
bool FOC_Init(FOC_Parameter_t* foc);

/**
 * @brief FOC主控制函数
 * @param foc FOC参数结构体指针
 */
void FOC_Main(FOC_Parameter_t* foc);

/* ================ FOC参数更新函数 ================ */

/**
 * @brief 更新FOC参数中的角度和速度反馈
 * @param foc FOC参数结构体指针
 * @param motor 电机参数结构体指针
 */
void FOC_UpdateThetaAndSpeed(FOC_Parameter_t* foc, const Motor_Parameter_t* motor);

/**
 * @brief 更新FOC参数中的电流反馈
 * @param foc FOC参数结构体指针
 * @param current_phase 三相电流数据
 */
void FOC_UpdateCurrentFeedback(FOC_Parameter_t* foc, const Phase_t* current_phase);

/**
 * @brief 更新FOC参数中的电压反馈
 * @param foc FOC参数结构体指针
 * @param udc 直流母线电压
 * @param inv_udc 直流母线电压倒数
 */
void FOC_UpdateVoltageFeedback(FOC_Parameter_t* foc, float udc, float inv_udc);

/**
 * @brief 获取FOC计算的PWM输出
 * @param foc FOC参数结构体指针
 * @param tcm_output PWM输出数据
 * @return bool 是否成功获取输出
 */
bool FOC_GetPWMOutput(const FOC_Parameter_t* foc, Phase_t* tcm_output);

/**
 * @brief 更新FOC系统频率参数
 * @param foc FOC参数结构体指针
 * @param freq 主频率
 * @param ts 主采样周期
 * @param pwm_arr PWM周期
 */
void FOC_UpdateFrequencyParams(FOC_Parameter_t* foc, float freq, float ts, float pwm_arr);

/**
 * @brief 设置FOC控制模式
 * @param foc FOC参数结构体指针
 * @param mode 控制模式
 */
void FOC_SetMode(FOC_Parameter_t* foc, FOC_Mode_t mode);

/**
 * @brief 设置FOC停止状态
 * @param foc FOC参数结构体指针
 * @param stop 停止标志
 */
void FOC_SetStopFlag(FOC_Parameter_t* foc, bool stop);

/**
 * @brief 设置FOC速度参考值
 * @param foc FOC参数结构体指针
 * @param speed_ref 速度参考值
 */
void FOC_SetSpeedReference(FOC_Parameter_t* foc, float speed_ref);

#endif /* _FOC_H_ */
