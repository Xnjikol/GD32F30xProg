#ifndef _FOC_H_
#define _FOC_H_

#include <stdbool.h>
#include "pid.h"
#include "reciprocal.h"
#include "signal.h"
#include "transformation.h"

extern Clark_t Foc_Iclark_Fdbk;

typedef enum
{
    IDLE,
    VF_MODE,
    IF_MODE,
    SPEED,
    STARTUP,
    IDENTIFY
} FocMode_t;

typedef struct
{
    Park_t vol_ref;  // 电压参考
    float  freq;
    float  offset;
} VF_Parameter_t;

typedef struct
{
    Park_t cur_ref;  // DQ轴电流参考
    float  freq;
    float  offset;
    bool   use_sensor;  // 是否使用传感器
} IF_Parameter_t;

extern Phase_t Foc_IPhase;
extern Clark_t Foc_Iclark_Fdbk;
extern Park_t  Foc_Idq_Ref;
extern Park_t  Foc_Idq_Fdbk;
extern Clark_t Foc_Uclark_Ref;
extern Park_t  Foc_Udq_Ref;

extern float Foc_Speed_Ref;
extern float Foc_Speed_Fdbk;
extern float Foc_Speed_Ramp;
extern float Foc_Theta;

extern float Foc_BusVoltage;
extern float Foc_BusVoltage_Inv;

void      Foc_Set_SampleTime(const SystemTimeConfig_t* config);
void      Foc_Set_Mode(FocMode_t mode);
FocMode_t Foc_Get_Mode(void);
void      Foc_Set_ResetFlag(bool reset);
bool      Foc_Get_ResetFlag(void);
void      Foc_Set_BusVoltage(float voltage);
float     Foc_Get_BusVoltage(void);
Park_t    Foc_Get_Inductor(void);
void      Foc_Set_BusVoltageInv(float voltage);
void      Foc_Set_Angle(float angle);
void      Foc_Set_Speed(float speed);
float     Foc_Get_SpeedRamp(void);
void      Foc_Set_Speed_and_Angle(MotorState_t* angle_speed);
void      Foc_Set_Iclark_Fdbk(Clark_t current);
Clark_t   Foc_Get_Iclark_Fdbk(void);
void      Foc_Set_Idq_Ref(Park_t idq_ref);
Park_t    Foc_Get_Idq_Ref(void);
void      Foc_Set_Idq_Fdbk(Park_t idq_fdbk);
Park_t    Foc_Get_Idq_Fdbk(void);
void      Foc_Set_Udq_Ref(Park_t udq_ref);
Park_t    Foc_Get_Udq_Ref(void);
void      Foc_Set_Uclark_Ref(Clark_t uclark_ref);
Clark_t   Foc_Get_Uclark_Ref(void);
Phase_t   Foc_Get_Tcm(void);
void      Foc_Set_Vf_Param(VF_Parameter_t* vf_param);
void      Foc_Set_If_Param(IF_Parameter_t* if_param);
void      Foc_Set_Pid_Speed_Handler(PID_Handler_t* handler);
void      Foc_Set_Pid_CurD_Handler(PID_Handler_t* handler);
void      Foc_Set_Pid_CurQ_Handler(PID_Handler_t* handler);
void      Foc_Set_Ramp_Speed_Handler(RampGenerator_t* handler);

Park_t Foc_Update_Main(void);

#endif /* _FOC_H_ */
