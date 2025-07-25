#ifndef _FOC_TYPES_H_
#define _FOC_TYPES_H_

#include "pid.h"
#include "signal.h"
#include "stdint.h"
#include "transformation.h"


typedef enum
{
  Disable = 0,
  Enable = !Disable
} EnableStatus,
    CommandStatus;

/*    Type Definitions   */
typedef enum
{
  IDLE,
  VF_MODE,
  IF_MODE,
  Speed,
  EXIT,
  Identify,
  space
} FOC_Mode_t;

typedef enum
{
  BASIC_INIT,      // 基础初始化：仅获取系统参数
  BASIC_READY,     // 基础就绪：等待完整初始化触发
  FULL_INIT,       // 完整初始化：用户触发的完整参数初始化
  READY,           // 就绪：完整初始化完成，准备运行
  RUNNING          // 运行：正常工作状态
} DeviceStateEnum_t;

typedef struct
{
  DeviceStateEnum_t Mode;
  bool basic_init_done;     // 基础初始化完成标志
  bool full_init_done;      // 完整初始化完成标志
  bool system_params_valid; // 系统参数有效标志
} DeviceState_t;

typedef struct
{
  float Rs;
  float Ld;
  float Lq;
  float Flux;
  float Pn;
  float Resolver_Pn;
  float inv_MotorPn;
  float Position;  // position sensor usually up to 16bits resolution
  float Position_Scale;
  float Position_Offset;  // Zero Position
  float Mech_Theta;       // Mechanical angle (rad)
  float Elec_Theta;       // Electrical angle (rad)
  float Speed;            // Speed (rpm)
} Motor_Parameter_t;

typedef struct
{
  float Vref_Ud;
  float Vref_Uq;
  float Freq;
  float Theta;
} VF_Parameter_t;

typedef struct
{
  float Id_ref;
  float Iq_ref;
  float IF_Freq;
  float Theta;
  EnableStatus Sensor_State;
} IF_Parameter_t;

typedef struct
{
  float ref;                  // 目标速度（参考值）
  float fdbk;                 // 实际速度反馈
  bool reset;                 // 停止标志
  uint16_t prescaler;         // 分频数（等于SPEED_LOOP_PRESCALER）
  uint16_t counter;           // 分频计数器
  RampGenerator_t* hnd_ramp;  // 斜坡输出（用于平滑目标速度变化）
  PID_Handler_t* hnd_speed;   // PID控制器句柄
} SpdLoop_t;

typedef struct
{
  Park_t* ref;               // DQ轴电流参考
  Park_t* fdbk;              // DQ轴电流反馈
  bool reset;                // 停止标志
  PID_Handler_t* handler_d;  // D轴PID控制器句柄
  PID_Handler_t* handler_q;  // Q轴PID控制器句柄
} CurLoop_t;

typedef struct
{
  bool initialized;
  float I_Max;
  float Udc;
  float inv_Udc;
  float Theta;   /* Electrical angle (rad) */
  float PWM_ARR; /* PWM period */
  Phase_t* Tcm;
  float Ts;
  float freq;
  bool Stop;               // Stop flag
  SpdLoop_t* Hnd_spdloop;  // 转速环相关变量
  CurLoop_t* Hnd_curloop;  // 电流环相关变量
  Phase_t* Iabc_fdbk;      // ABC相电流反馈
  Clark_t* Iclark_fdbk;    // αβ轴电流反馈
  Park_t* Idq_ref;         // DQ轴电流参考
  Park_t* Idq_fdbk;        // DQ轴电流反馈
  Park_t* Udq_ref;         // DQ轴电压参考
  Clark_t* Uclark_ref;     // αβ轴电压指令
  FOC_Mode_t Mode;         // 当前控制模式
} FOC_Parameter_t;

typedef struct
{
  float Tcm1;
  float Tcm2;
  float Tcm3;
} SVPWM_t;

#endif /* _FOC_TYPES_H_ */
