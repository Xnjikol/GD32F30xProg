#include "motor.h"
#include <stdbool.h>
#include "Buffer.h"
#include "filter.h"
#include "parameters.h"
#include "reciprocal.h"
#include "theta_calc.h"

#define MOTOR_DEFAULT_PRESCALER 10U

float Motor_Rs    = MOTOR_RS;
float Motor_Ld    = MOTOR_LD;
float Motor_Lq    = MOTOR_LQ;
float Motor_InvLd = 1 / MOTOR_LD;
float Motor_InvLq = 1 / MOTOR_LQ;
float Motor_Flux  = MOTOR_FLUX;

float Motor_Pn          = MOTOR_PN;
float Resolver_Pn       = MOTOR_RESOLVER_PN;
float Motor_MotorPn_inv = 1 / MOTOR_RESOLVER_PN;

float Speed_Time = SPEED_LOOP_TIME;
float Speed_Freq = SPEED_LOOP_FREQ;
float SampleTime = MAIN_LOOP_TIME;
float SampleFreq = MAIN_LOOP_FREQ;

float Motor_Position_Scale  = MOTOR_POSITION_SCALE;
float Motor_Position_Offset = MOTOR_POSITION_OFFSET;
float Motor_Position        = 0.0F;
float Motor_Theta_Factor    = MOTOR_THETA_FACTOR;
float Motor_ThetaElec       = 0.0F;
float Motor_ThetaMech       = 0.0F;
float Motor_Speed           = 0.0F;

uint16_t Speed_Prescaler = SPEED_LOOP_PRESCALER;

static IIR1stFilter_t Motor_Speed_Filter = {0};

static inline void Motor_Update_Theta(void)
{
    // 位置传感器数据处理
    float delta = Motor_Position - Motor_Position_Offset;
    if (delta < 0)
    {
        delta += (Motor_Position_Scale + 1.0F);
    }

    Motor_ThetaMech = delta * Motor_Theta_Factor;

    Motor_ThetaElec = Motor_ThetaMech * Motor_Pn;
    Motor_ThetaElec = wrap_theta_2pi(Motor_ThetaElec);

    // Buffer_Put(Motor_Position, 5);
    // Buffer_Put(Motor_ThetaMech, 6);
    Buffer_Put(Motor_ThetaElec, 5);
}

static inline void calculate_speed(void)
{
    static uint16_t cnt_speed  = 0x0000;
    static float    last_theta = 0.0F;
    cnt_speed++;
    if (cnt_speed < Speed_Prescaler)
    {
        return;
    }
    cnt_speed = 0x0000;

    Motor_Speed = calc_speed(Motor_ThetaMech, last_theta, SampleFreq);
    Motor_Speed = IIR1stFilter_Update(&Motor_Speed_Filter, Motor_Speed);
    last_theta  = Motor_ThetaMech;

    Buffer_Put(Motor_Speed, 4);

    return;
}

bool Motor_Set_SampleTime(const SystemTimeConfig_t* time_config)
{
    if (time_config == NULL)
    {
        return false;
    }
    SampleFreq = time_config->speed.freq;
    return true;
}

bool Motor_Set_Filter(float cutoff_freq, float sample_freq)
{
    IIR1stFilter_Init(&Motor_Speed_Filter, cutoff_freq, sample_freq);
    return true;
}

void Motor_Set_Position(uint16_t position)
{
    Motor_Position = position;
    Motor_Update_Theta();
}

void Motor_Set_Theta_Elec(float theta)
{
    Motor_ThetaElec = theta;
}

float Motor_Get_ThetaElec(void)
{
    return Motor_ThetaElec;
}

void Motor_Set_Theta_Mech(float theta)
{
    Motor_ThetaMech = theta;
}

float Motor_Get_Theta_Mech(void)
{
    return Motor_ThetaMech;
}

void Motor_Set_Speed(float speed)
{
    Motor_Speed = speed;
}

float Motor_Get_Speed(void)
{
    calculate_speed();
    return Motor_Speed;
}
