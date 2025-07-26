#ifndef __MAIN_INT_H__
#define __MAIN_INT_H__

#include "can.h"
#include "foc.h"
#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "main.h"
#include "pid.h"
#include "sensorless_interface.h"
#include "systick.h"

/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#ifndef GATE_POLARITY_LOW_ACTIVE
#define GATE_POLARITY_LOW_ACTIVE /* Define here */
#endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) && !defined(GATE_POLARITY_LOW_ACTIVE)
#error "Please define GATE_POLARITY_HIGH_ACTIVE or GATE_POLARITY_LOW_ACTIVE in foc.h"
#endif

#ifndef Resolver_Position
#ifndef Encoder_Position
#define Encoder_Position /* Define here */
#endif
#endif

/* FOC parameters */
#define SPEED_LOOP_PRESCALER 10.0F /* Speed loop frequency division factor */

// Since CCP demanded struct FOC is Global Variable, make it visible to main ISR //
extern FOC_Parameter_t FOC;
extern Motor_Parameter_t Motor;
extern DeviceState_t Device;

static inline void FOC_UpdateCurrent(float Ia, float Ib, float Ic)
{
  FOC.Iabc_fdbk->a = Ia;
  FOC.Iabc_fdbk->b = Ib;
  FOC.Iabc_fdbk->c = Ic;
}

static inline void FOC_UpdateVoltage(float Udc, float inv_Udc)
{
  FOC.Udc = Udc;
  FOC.inv_Udc = inv_Udc;
}

static inline void FOC_UpdateTheta(float Theta)
{
  FOC.Theta = Theta;
}

static inline void FOC_OutputCompare(float* Tcm1, float* Tcm2, float* Tcm3)
{
  *Tcm1 = FOC.Tcm->a;
  *Tcm2 = FOC.Tcm->b;
  *Tcm3 = FOC.Tcm->c;
}
static inline void FOC_UpdateMaxCurrent(float I_Max)
{
  FOC.I_Max = I_Max;
}

void FOC_UpdateMainFrequency(float freq, float Ts, float PWM_ARR);

void Main_Int_Handler(void);

// 设备状态管理函数
static inline void Main_Int_TriggerFullInit(void)
{
  extern DeviceState_t Device;
  if (Device.Mode == WAITING)
  {
    Device.Mode = SETUP;
  }
}

// 获取设备状态的便利函数
static inline bool Main_Int_IsBasicReady(void)
{
  extern DeviceState_t Device;
  return Device.Mode == WAITING && Device.basic_init_done;
}

static inline bool Main_Int_IsFullyReady(void)
{
  extern DeviceState_t Device;
  return Device.Mode >= READY && Device.full_init_done;
}

#endif /* __MAIN_INT_H__ */
