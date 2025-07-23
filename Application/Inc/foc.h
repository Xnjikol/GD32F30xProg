#ifndef _FOC_H_
#define _FOC_H_

#include "filter.h"
#include "foc_types.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"


/*       Constants      */

#define T_2kHz 0.0005F  /* T 2kHz */
#define f_2kHz 2000.0F  /* f 2kHz */
#define T_1kHz 0.0001F  /* T 1kHz */
#define T_200Hz 0.005F  /* T 200Hz */
#define T_10kHz 0.0001F /* 10kHz sampling time */

#define TIME_2KHZ 0.0005F  /* T 2kHz */
#define FREQ_2KHZ 2000.0F  /* f 2kHz */
#define TIME_1KHZ 0.0001F  /* T 1kHz */
#define TIME_200HZ 0.005F  /* T 200Hz */
#define TIME_10KHZ 0.0001F /* 10kHz sampling time */


// Since CCP demanded struct FOC is Global Variable, make it visible to main ISR //
extern FOC_Parameter_t FOC;

void FOC_Main(FOC_Parameter_t* foc, Motor_Parameter_t* motor, VF_Parameter_t* vf,
              IF_Parameter_t* if_p, PID_Controller_t* id_pid, PID_Controller_t* iq_pid,
              PID_Controller_t* speed_pid, RampGenerator_t* speed_ramp, Clarke_t* inv_park,
              Clarke_t* I_clarke, float* speed_ref);

static inline void FOC_UpdateCurrent(float Ia, float Ib, float Ic)
{
  FOC.Iphase->a = Ia;
  FOC.Iphase->b = Ib;
  FOC.Iphase->c = Ic;
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

#endif /* _FOC_H_ */
