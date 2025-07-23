#ifndef _FOC_TYPES_H_
#define _FOC_TYPES_H_

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
  INIT,
  IDLE,
  VF_MODE,
  IF_MODE,
  Speed,
  EXIT,
  Identify,
  space
} FOC_Mode_t;

typedef struct
{
  float Rs;
  float Ld;
  float Lq;
  float Flux;
  float Pn;
  uint16_t Position_Scale;
  float Resolver_Pn;
  float inv_MotorPn;
  float Position_Offset;  // Zero Position
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
  float value;  // output value
  float slope;  // Δvalue/s
  float limit_min;
  float limit_max;
  float target;
  float Ts;
} RampGenerator_t;

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
  uint16_t Position;         // position sensor usually up to 16bits resolution
  uint16_t Position_Scale;   // Scale of position sensor
  uint16_t Position_Offset;  // Zero Position
  uint16_t Pn;               // Motor Poles
  float Mech_Theta;          // Mechanical angle (rad)
  float Elec_Theta;          // Electrical angle (rad)
  float Speed;               // Speed (rpm)
} Sensor_Parameter_t;

typedef struct
{
  PhaseABC_t* Iphase;  // ABC Phase current
  Park_t* Ipark;       // DQ Axis current
  float I_Max;
  float Udc;
  float inv_Udc;
  float Theta;       /* Electrical angle (rad) */
  float Speed;       /* Speed (rpm) */
  Park_t* Upark_ref;  // DQ Axis voltage reference
  Park_t* Ipark_ref;  // DQ Axis current reference
  float PWM_ARR;     /* PWM period */
  float Tcm1;
  float Tcm2;
  float Tcm3;
  float Ts;
  float f;
  FOC_Mode_t Mode;  // 当前控制模式
} FOC_Parameter_t;
typedef struct
{
  float Tcm1;
  float Tcm2;
  float Tcm3;
} SVPWM_t;

#endif /* _FOC_TYPES_H_ */
