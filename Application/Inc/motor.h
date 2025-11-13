#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdbool.h>
#include <stdint.h>
#include "reciprocal.h"

extern float Motor_Rs;
extern float Motor_Ld;
extern float Motor_Lq;
extern float Motor_InvLd;
extern float Motor_InvLq;
extern float Motor_Flux;
extern float Motor_Pn;
extern float Resolver_Pn;
extern float Motor_InvPn;
extern float Motor_Position_Scale;
extern float Motor_Position_Offset;
extern float Motor_Theta_Factor;
extern float Speed_Time;
extern float Speed_Freq;
extern float SampleTime;
extern float SampleFreq;
extern float Motor_Position;

extern volatile float Motor_ThetaElec;
extern volatile float Motor_ThetaMech;
extern volatile float Motor_Speed;

extern uint16_t Speed_Prescaler;

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float Pn;
    float Resolver_Pn;
    float inv_MotorPn;
    float Position_Scale;
    float Position_Offset;  // Zero Position
    float theta_factor;
} MotorParam_t;

void Motor_Update(void);

bool Motor_Set_Filter(float sample_freq, float cutoff_freq);

#endif  // MOTOR_H_
