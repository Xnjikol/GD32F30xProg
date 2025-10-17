#include "motor.h"
#include <stdbool.h>
#include "Buffer.h"
#include "filter.h"
#include "reciprocal.h"
#include "theta_calc.h"

#define MOTOR_DEFAULT_PRESCALER 10U

static bool  Motor_Initialized     = false;
static float Motor_Rs              = 0.0F;
static float Motor_Ld              = 0.0F;
static float Motor_Lq              = 0.0F;
static float Motor_Flux            = 0.0F;
static float Motor_Pn              = 0.0F;
static float Resolver_Pn           = 0.0F;
static float Motor_MotorPn_inv     = 0.0F;
static float Motor_Position_Scale  = 0.0F;
static float Motor_Position_Offset = 0.0F;
static float Motor_Theta_Factor    = 0.0F;
static float Motor_SampleFreq      = 0.0F;
static float Motor_Position        = 0.0F;
static float Motor_Theta_Elec      = 0.0F;
static float Motor_Theta_Mech      = 0.0F;
static float Motor_Speed           = 0.0F;

static uint16_t Motor_Speed_Prescaler = 0;

static IIR1stFilter_t Motor_Speed_Filter = {0};

static inline void Motor_Update_Theta(void)
{
    // 位置传感器数据处理
    float delta = Motor_Position - Motor_Position_Offset;
    if (delta < 0)
    {
        delta += (Motor_Position_Scale + 1.0F);
    }

    Motor_Theta_Mech = delta * Motor_Theta_Factor;

    Motor_Theta_Elec = Motor_Theta_Mech * Motor_Pn;
    Motor_Theta_Elec = wrap_theta_2pi(Motor_Theta_Elec);

    Buffer_Put(Motor_Position, 5);
    Buffer_Put(Motor_Theta_Mech, 6);
    Buffer_Put(Motor_Theta_Elec, 7);
}

static inline void calculate_speed(void)
{
    static uint16_t cnt_speed  = 0x0000;
    static float    last_theta = 0.0F;
    cnt_speed++;
    if (cnt_speed < Motor_Speed_Prescaler)
    {
        return;
    }
    cnt_speed = 0x0000;

    Motor_Speed
        = calc_speed(Motor_Theta_Mech, last_theta, Motor_SampleFreq);
    Motor_Speed = IIR1stFilter_Update(&Motor_Speed_Filter, Motor_Speed);
    last_theta  = Motor_Theta_Mech;
}

bool Motor_Set_SampleTime(const SystemTimeConfig_t* time_config)
{
    if (time_config == NULL)
    {
        return false;
    }
    Motor_SampleFreq = time_config->speed.inv;
    return true;
}

bool Motor_Initialization(const MotorParam_t* motor_params)
{
    Motor_Rs              = motor_params->Rs;
    Motor_Ld              = motor_params->Ld;
    Motor_Lq              = motor_params->Lq;
    Motor_Flux            = motor_params->Flux;
    Motor_Pn              = motor_params->Pn;
    Resolver_Pn           = motor_params->Resolver_Pn;
    Motor_MotorPn_inv     = motor_params->inv_MotorPn;
    Motor_Position_Scale  = motor_params->Position_Scale;
    Motor_Position_Offset = motor_params->Position_Offset;
    Motor_Theta_Factor    = motor_params->theta_factor;

    Motor_Initialized = true;
    return true;
}

bool Motor_Set_SpeedPrescaler(uint16_t prescaler)
{
    if (prescaler == 0)
    {
        Motor_Speed_Prescaler = MOTOR_DEFAULT_PRESCALER;
        return false;  // 分频数不能为0
    }
    Motor_Speed_Prescaler = prescaler;
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
    Motor_Theta_Elec = theta;
}

float Motor_Get_ThetaElec(void)
{
    return Motor_Theta_Elec;
}

void Motor_Set_Theta_Mech(float theta)
{
    Motor_Theta_Mech = theta;
}

float Motor_Get_Theta_Mech(void)
{
    return Motor_Theta_Mech;
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
