#ifndef _SMO_H
#define _SMO_H

#include "pid.h"
#include "reciprocal.h"
#include "theta_calc.h"
#include "transformation.h"

/**
 * @brief 无传感器控制状态机
 */
typedef struct {
    float leso_beta1; /*!< LESO增益 */
    float leso_beta2; /*!< LESO增益 */
    float Rs;         /*!< 定子电阻 */
    float Ld;         /*!< 定子电感 */
    float Lq;         /*!< 定子电感 */
} LESO_Param_t;

/**
 * @brief SMO参数设置与获取接口
 */
bool Leso_Set_SampleTime(const SystemTimeConfig_t* config);

bool Leso_Initialization(const LESO_Param_t* param);

void Leso_Set_InvPn(float inv_Pn);

void Leso_Set_SpeedFilter(float cutoff_freq, float sample_freq);

void Leso_Set_Pid_Handler(PID_Handler_t config);

void Leso_Set_Voltage(Clark_t voltage);

void Leso_Set_Current(Clark_t current);

void Leso_Set_Theta(float theta);

void Leso_Set_Theta_Err(float ref);

void Leso_Set_Speed_Err(float ref);

void Leso_Set_Enabled(bool enabled);

bool Leso_Get_Enabled(void);

AngleResult_t Leso_Get_Result(void);

Clark_t Leso_Get_EmfEst(void);

/**
 * @brief SMO内部状态更新接口
 */
void Leso_Update_EmfEstA(void);
void Leso_Update_EmfEstB(void);
void Leso_Update_Angle(void);

#endif /* _SMO_H */