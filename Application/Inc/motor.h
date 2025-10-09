#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdbool.h>
#include <stdint.h>
#include "reciprocal.h"

typedef struct {
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

bool  Motor_Set_SampleTime(const SystemTimeConfig_t* time_config);
bool  Motor_Initialization(const MotorParam_t* motor_params);
bool  Motor_Set_SpeedPrescaler(uint16_t prescaler);
bool  Motor_Set_Filter(float sample_freq, float cutoff_freq);
void  Motor_Set_Position(uint16_t position);
void  Motor_Set_Theta_Elec(float theta);
float Motor_Get_ThetaElec(void);
void  Motor_Set_Theta_Mech(float theta);
float Motor_Get_Theta_Mech(void);
void  Motor_Set_Speed(float speed);
float Motor_Get_Speed(void);
#endif  // MOTOR_H_
