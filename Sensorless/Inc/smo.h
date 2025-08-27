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
    float smo_gain; /*!< SMO增益 */
    float Rs;       /*!< 定子电阻 */
    float Ld;       /*!< 定子电感 */
    float Lq;       /*!< 定子电感 */
} SMO_Param_t;

/**
 * @brief SMO参数设置与获取接口
 */
bool          Smo_Set_SampleTime(const SystemTimeConfig_t* config);
bool          Smo_Initialization(const SMO_Param_t* param);
void          Smo_Set_InvPn(float inv_Pn);
void          Smo_Set_EmfFilter(float cutoff_freq, float sample_time);
void          Smo_Set_SpeedFilter(float cutoff_freq, float sample_freq);
void          Smo_Set_Pid_Handler(PID_Handler_t config);
void          Smo_Set_Voltage(Clark_t voltage);
void          Smo_Set_Current(Clark_t current);
void          Smo_Set_Theta(float theta);
void          Smo_Set_Enabled(bool enabled);
bool          Smo_Get_Enabled(void);
AngleResult_t Smo_Get_Result(void);

/**
 * @brief SMO内部状态更新接口
 */
void Smo_Update_EmfEst(void);
void Smo_Update_Angle(void);

#endif /* _SMO_H */