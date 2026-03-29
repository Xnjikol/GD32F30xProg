/**
 * @file sensorless_interface.c
 * @brief 无传感器控制接口层实现
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 */

#include "sensorless_interface.h"
#include <stdbool.h>
#include "Buffer.h"
#include "FlyingStart.h"
#include "filter.h"
#include "flying.h"
#include "foc.h"
#include "hardware_interface.h"
#include "hf_injection.h"
#include "leso.h"
#include "motor.h"
#include "reciprocal.h"
#include "transformation.h"
#include <math.h>

bool Sensorless_Enabled = {0};

bool  Sensorless_Reset          = {0};
bool  Sensorless_Reset_Prev     = {0};
float Sensorless_Threshold_High = {0};
float Sensorless_Threshold_Low  = {0};
float Sensorless_Switch_Speed   = {0};
float Sensorless_SpeedRef       = {0};
float Sensorless_SpeedFdbk      = {0};
float Sensorless_SpeedEst       = {0};
float Sensorless_ThetaEst       = {0};
float Sensorless_ThetaErr       = {0};
float Sensorless_SpeedErr       = {0};

volatile float Sensorless_ThetAdj = {0};

PID_Handler_t  Sensorless_Theta_PID    = {0};
IIR1stFilter_t Sensorless_SpeedFilter1 = {0};
IIR2ndFilter_t Sensorless_SpeedFilter2 = {0};

sensorless_method_t Sensorless_Method = {0};

bool restore_states(void)
{
    Sensorless_Theta_PID.integral       = 0.0F;
    Sensorless_Theta_PID.previous_error = 0.0F;
    Sensorless_Theta_PID.output         = 0.0F;
    Sensorless_ThetaEst                 = 0.0F;
    Sensorless_SpeedEst                 = 0.0F;

    return true;
}

bool Sensorless_Initialization(const Sensorless_Param_t* param)
{
    if (param == NULL)
    {
        return false;
    }

    Sensorless_Threshold_High = param->switch_speed + param->hysteresis;
    Sensorless_Threshold_Low  = param->switch_speed - param->hysteresis;
    Sensorless_Switch_Speed   = param->switch_speed;

    return true;
}

bool Sensorless_Set_SpeedFilter(float cutoff_freq, float sample_freq)
{
    if (cutoff_freq <= 0.0F || sample_freq <= 0.0F)
    {
        return false;
    }
    IIR2ndFilter_Init(&Sensorless_SpeedFilter2, cutoff_freq, sample_freq);
    IIR1stFilter_Init(&Sensorless_SpeedFilter1, cutoff_freq, sample_freq);
    return true;
}

bool Sensorless_Set_PidParams(const PID_Handler_t* pid_handler)
{
    if (pid_handler == NULL)
    {
        return false;
    }
    Sensorless_Theta_PID = *pid_handler;
    return true;
}

bool Sensorless_Set_Voltage(Clark_t voltage)
{
    if (Hfi_Enabled)
    {
    }

    if (Leso_Enabled)
    {
        Leso_Set_Voltage(voltage);
    }

    return true;
}

bool Sensorless_Set_Current(Clark_t current)
{
    // if (Hfi_Get_Enabled()) {
    //     Hfi_Set_Current(current);
    // }

    Leso_Set_Current(current);

    return true;
}

bool Sensorless_Calculate_Err(MotorState_t result)
{
    if (!Sensorless_Enabled)
    {
        return false;
    }

    float theta = result.theta;
    float speed = result.speed;
    float error = 0.0F;

    error               = wrap_theta_2pi(theta - Sensorless_ThetaEst + PI) - PI;
    Sensorless_ThetaErr = rad2deg(error);
    Sensorless_SpeedErr = speed - Sensorless_SpeedEst;

    Buffer_Put(Sensorless_ThetaErr, 8);
    Buffer_Put(Sensorless_SpeedErr, 9);

    Hfi_Calc_ThetaErr(theta);
    Hfi_Calc_SpeedErr(speed);

    Leso_Calc_ThetaErr(theta);
    Leso_Calc_SpeedErr(speed);

    return true;
}

static inline float pll_update(float error, bool reset)
{
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Sensorless_Theta_PID);

    if (reset)
    {
        Sensorless_ThetaEst = 0.0F;
    }

    if (isnanf(omega))
    {
        omega = 0.0F;
    }

    if (isinff(omega))
    {
        omega = 0.0F;
    }

    Sensorless_ThetaEst += omega * SampleTime;
    if (Sensorless_ThetaEst > M_2PI)
    {
        Sensorless_ThetaEst -= M_2PI;
    }
    if (Sensorless_ThetaEst < 0.0F)
    {
        Sensorless_ThetaEst += M_2PI;
    }
    Sensorless_ThetaEst = wrap_theta_2pi(Sensorless_ThetaEst);

    return omega;
}

static inline float calculate_speed(float omega)
{
    static uint16_t speed_cnt = 0x0000U;
    static float    speed_int = 0.0F;
    float           speed     = 0.0F;
    speed_int += radps2rpm(omega) * Motor_InvPn * 0.1F;
    speed_cnt++;
    if (speed_cnt < Speed_Prescaler)
    {
        return Sensorless_SpeedEst;
    }
    speed_cnt = 0x0000U;
    speed     = IIR2ndFilter_Update(&Sensorless_SpeedFilter2, speed_int);
    speed_int = 0.0F;
    Sensorless_SpeedEst = speed;

    return speed;
}

MotorState_t Sensorless_Update_Position(void)
{
    MotorState_t default_result = {0};
    if (!Sensorless_Enabled)
    {
        return default_result;
    }

    return (MotorState_t){.speed = Sensorless_SpeedEst,
                          .theta = Sensorless_ThetaEst + Sensorless_ThetAdj};
}

bool Sensorless_Calculate(void)
{
    float error = 0.0F;
    float omega = 0.0F;
    float speed = 0.0F;

    switch (Sensorless_Method)
    {
    case SENSORLESS_FLYINGSTART:
        error        = 0.0F;
        Hfi_Enabled  = false;
        Leso_Enabled = false;
        if (!FlyingStartEnabled)
        {
            if (Sensorless_SpeedEst > Sensorless_Switch_Speed)
            {
                Sensorless_Method = SENSORLESS_HIGH_LESO;
            }
            else
            {
                Sensorless_Method = SENSORLESS_LOW;
            }
        }
        break;

    case SENSORLESS_START:
        static uint16_t start_cnt = 0x0000U;
        start_cnt++;

        error = Hfi_Error;

        if (start_cnt >= 10U)
        {
            Sensorless_Method = SENSORLESS_LOW;
            start_cnt         = 0x0000U;
        }
        break;

    case SENSORLESS_LOW:
        error = Hfi_Error;

        Hfi_Enabled  = true;
        Leso_Enabled = false;

        // Leso_Enabled = fabsf(Foc_Speed_Ramp) >= Sensorless_Threshold_Low;
        if (fabsf(Foc_Speed_Fdbk) >= Sensorless_Switch_Speed)
        {
            Sensorless_Method = SENSORLESS_LOW2HIGH;
        }
        break;

    case SENSORLESS_LOW2HIGH:
        error = Hfi_Error;

        Hfi_Enabled  = true;
        Leso_Enabled = true;

        if (fabsf(Foc_Speed_Fdbk) > Sensorless_Threshold_High)
        {
            Sensorless_Method = SENSORLESS_HIGH_LESO;
        }
        if (fabsf(Foc_Speed_Fdbk) < Sensorless_Threshold_Low)
        {
            Sensorless_Method = SENSORLESS_LOW;
        }
        break;

    case SENSORLESS_HIGH_LESO:
        error = Leso_Error;

        Hfi_Enabled  = false;
        Leso_Enabled = true;

        // Hfi_Enabled = fabsf(Foc_Speed_Ramp) <= Sensorless_Threshold_High;
        if (fabsf(Foc_Speed_Ramp) <= Sensorless_Switch_Speed)
        {
            Sensorless_Method = SENSORLESS_HIGH2LOW;
        }
        break;

    case SENSORLESS_HIGH2LOW:
        error = Leso_Error;

        Hfi_Enabled  = true;
        Leso_Enabled = true;

        if (fabsf(Foc_Speed_Fdbk) > Sensorless_Threshold_High)
        {
            Sensorless_Method = SENSORLESS_HIGH_LESO;
        }
        if (fabsf(Foc_Speed_Fdbk) < Sensorless_Threshold_Low)
        {
            Sensorless_Method = SENSORLESS_LOW;
        }
        break;

    default:
        Sensorless_Method = SENSORLESS_HIGH_LESO;
        error             = Leso_Error;
        break;
    }
    omega = pll_update(error, Sensorless_Reset);
    speed = calculate_speed(omega);

    Leso_Theta = Sensorless_ThetaEst;
    Leso_Speed = speed;
    Leso_We    = rpm2radps(speed) * Motor_Pn;

    Hfi_Theta = Sensorless_ThetaEst;

    Hfi_Update();
    Leso_Update();

    Sensorless_Reset_Prev = Sensorless_Reset;

    return true;
}

Park_t Sensorless_Inject_Voltage(Park_t voltage)
{
    if (!Sensorless_Enabled)
    {
        return voltage;
    }

    if (Sensorless_Reset)
    {
        return voltage;
    }

    if (Hfi_Get_Enabled())
    {
        Park_t inj = Hfi_Get_Inject_Voltage();
        voltage.d += inj.d;
        voltage.q += inj.q;
        return voltage;
    }

    return voltage;
}

Clark_t Sensorless_FilterCurrent(Clark_t current)
{
    if (!Sensorless_Enabled)
    {
        return current;
    }

    if (Sensorless_Reset)
    {
        return current;
    }

    if (!Hfi_Get_Enabled())
    {
        return current;
    }

    return Hfi_Process_Current(current);
}
