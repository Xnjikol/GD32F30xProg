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
#include "systick.h"

Motor_Parameter_t Motor;
Sensor_Parameter_t Sensor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
PID_Controller_t Speed_PID;
RampGenerator_t Speed_Ramp;
Clarke_t Inv_Park;
Clarke_t Clarke;
Phase_Data_t Phase_Current;
Park_Data_t DQ_Current;
Park_Data_t DQ_Current_ref;
Park_Data_t DQ_Voltage_ref;

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
#define SPEED_LOOP_PRESCALER 10 /* Speed loop frequency division factor */

extern Sensor_Parameter_t Sensor;

static inline void Sensor_UpdatePosition(uint16_t Position)
{
  Sensor.Position_Scale = Position;   // Update position scale
  Sensor.Position_Offset = Position;  // Update zero position
}

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
  *Tcm1 = FOC.Tcm1;
  *Tcm2 = FOC.Tcm2;
  *Tcm3 = FOC.Tcm3;
}
static inline void FOC_UpdateMaxCurrent(float I_Max)
{
  FOC.I_Max = I_Max;
}

void FOC_UpdateMainFrequency(float freq, float Ts, float PWM_ARR);

void Main_Int_Handler(void);

#endif /* __MAIN_INT_H__ */
