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
#include "filter.h"
#include "flying.h"
#include "foc.h"
#include "hf_injection.h"
#include "leso.h"
#include "motor.h"
#include "reciprocal.h"
#include "smo.h"
#include "transformation.h"
#include <math.h>

volatile bool Sensorless_Enabled = {0};

bool  Sensorless_Reset          = {0};
bool  Sensorless_Reset_Prev     = {0};
float Sensorless_Threshold_Hfi  = {0};
float Sensorless_Threshold_Leso = {0};
float Sensorless_Switch_Speed   = {0};
float Sensorless_SpeedRef       = {0};
float Sensorless_SpeedFdbk      = {0};
float Sensorless_SpeedEst       = {0};
float Sensorless_ThetaEst       = {0};
float Sensorless_InvPn          = {0};
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

bool Sensorless_Set_SampleTime(const SystemTimeConfig_t* config)
{
    if (config == NULL)
    {
        return false;
    }

    return true;
}

bool Sensorless_Initialization(const Sensorless_Param_t* param)
{
    if (param == NULL)
    {
        return false;
    }

    Sensorless_Threshold_Hfi  = param->switch_speed + param->hysteresis;
    Sensorless_Threshold_Leso = param->switch_speed - param->hysteresis;
    Sensorless_Switch_Speed   = param->switch_speed;

    return true;
}

bool Sensorless_Set_SpeedFilter(float cutoff_freq, float sample_freq)
{
    if (cutoff_freq <= 0.0F || sample_freq <= 0.0F)
    {
        return false;
    }
    IIR2ndFilter_Init(
        &Sensorless_SpeedFilter2, cutoff_freq, sample_freq);
    IIR1stFilter_Init(
        &Sensorless_SpeedFilter1, cutoff_freq, sample_freq);
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

bool Sensorless_Set_MotorParams(const MotorParam_t* motor_param)
{
    if (motor_param == NULL || motor_param->inv_MotorPn <= 0)
    {
        return false;
    }

    Sensorless_InvPn = motor_param->inv_MotorPn;
    return true;
}

sensorless_method_t Sensorless_Get_Method(void)
{
    return Sensorless_Method;
}

Clark_t Sensorless_Get_SmoEmf(void)
{
    return Leso_Get_EmfEst();
}

bool Sensorless_Set_Voltage(Clark_t voltage)
{
    if (Hfi_Get_Enabled())
    {
    }

    if (Leso_Get_Enabled())
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

void Sensorless_Set_SpeedFdbk(float fdbk)
{
    Sensorless_SpeedFdbk = fdbk;
}

void Sensorless_Set_SpeedRef(float ref)
{
    Sensorless_SpeedRef = ref;
}

MotorState_t Sensorless_Get_Error(void)
{
    return (MotorState_t){.theta = Sensorless_ThetaErr,
                          .speed = Sensorless_SpeedErr};
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

    error = wrap_theta_2pi(theta - Sensorless_ThetaEst + PI) - PI;
    Sensorless_ThetaErr = rad2deg(error);
    if (Sensorless_ThetaErr > 90.0F)
    {
        Sensorless_ThetaErr -= 180.0F;
    }
    else if (Sensorless_ThetaErr < -90.0F)
    {
        Sensorless_ThetaErr += 180.0F;
    }
    Sensorless_SpeedErr = speed - Sensorless_SpeedEst;

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
        return omega;
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

    Buffer_Put(Sensorless_ThetaEst, 9);

    return omega;
}

static inline float calculate_speed(float omega)
{
    static uint16_t speed_cnt = 0x0000U;
    static float    speed_int = 0.0F;
    // float           speed1    = 0.0F;
    // float           speed2    = 0.0F;
    float speed = 0.0F;
    speed_int += radps2rpm(omega) * Sensorless_InvPn * 0.1F;
    speed_cnt++;
    if (speed_cnt < 0x000AU)
    {
        return Sensorless_SpeedEst;
    }
    speed_cnt = 0x0000U;
    // speed1 = IIR1stFilter_Update(&Sensorless_SpeedFilter1, speed_int);
    // speed2 = IIR2ndFilter_Update(&Sensorless_SpeedFilter2, speed_int);
    speed = IIR2ndFilter_Update(&Sensorless_SpeedFilter2, speed_int);
    // if (Sensorless_Method == SENSORLESS_LOW2HIGH) {
    //     speed = speed2;
    // } else {
    //     speed = speed2;
    // }
    speed_int           = 0.0F;
    Sensorless_SpeedEst = speed;

    Buffer_Put(Sensorless_SpeedEst, 8);

    return speed;
}

MotorState_t Sensorless_Update_Position(void)
{
    MotorState_t default_result
        = {.speed = Sensorless_SpeedEst,
           .theta = Sensorless_ThetaEst + Sensorless_ThetAdj};
    if (!Sensorless_Enabled)
    {
        return default_result;
    }
    float error = 0.0F;
    float omega = 0.0F;
    float speed = 0.0F;
    switch (Sensorless_Method)
    {
    case SENSORLESS_START:
        Sensorless_Method = SENSORLESS_HIGH_SMO;

        error = SmoHandle.state.pll_err;
        return default_result;
        break;

    case SENSORLESS_LOW:
        return default_result;
        break;

    case SENSORLESS_HIGH_LESO:
        error = Leso_Get_PllErr();
        break;

    case SENSORLESS_HIGH_SMO:
        error = SmoHandle.state.pll_err;
        break;

    default:
        Sensorless_Method = SENSORLESS_HIGH_SMO;

        error = SmoHandle.state.pll_err;
        break;
    }
    MotorState_t leso_result = {0};
    MotorState_t hfi_result  = {0};

    leso_result = Leso_Get_Result();
    hfi_result  = Hfi_Get_Result();
    omega       = pll_update(error, Sensorless_Reset);
    speed       = calculate_speed(omega);

    Leso_Set_Theta(Sensorless_ThetaEst);
    Leso_Set_Speed(speed);
    Hfi_Set_Theta(Sensorless_ThetaEst);
    SmoHandle.state.theta = Sensorless_ThetaEst;
    SmoHandle.state.speed = speed;

    return (MotorState_t){
        .speed = Sensorless_SpeedEst,
        .theta = Sensorless_ThetaEst + Sensorless_ThetAdj};
}

static inline void enable_leso(bool enable)
{
    if (enable)
    {
        if (!Leso_Get_Enabled())
        {
            Leso_Set_Enabled(true);
        }
    }
    else
    {
        if (Leso_Get_Enabled())
        {
            Leso_Set_Enabled(false);
        }
    }
}

static inline void enable_hfi(bool enable)
{
    if (enable)
    {
        if (!Hfi_Get_Enabled())
        {
            Hfi_Set_Enabled(true);
            float estimate = 0.0F, target = 0.0F, error = 0.0F;
            estimate = Hfi_Get_Result().theta;
            target   = Sensorless_ThetaEst;
            error    = wrap_theta_2pi(target - estimate + PI) - PI;
            if (error >= M_PI_4 || error <= -M_PI_4)
            {
                Hfi_Set_InitialPosition(target);
            }
        }
    }
    else
    {
        if (Hfi_Get_Enabled())
        {
            Hfi_Set_Enabled(false);
        }
    }
}

bool Sensorless_Calculate(void)
{
    enable_leso(fabsf(Foc_Speed_Ramp) >= Sensorless_Threshold_Leso);
    enable_hfi(fabsf(Foc_Speed_Ramp) <= Sensorless_Threshold_Hfi);

    SmoHandle.enabled = Leso_Enabled;

    Hfi_Update();

    Leso_Update_Beta();
    Leso_Update_EmfEstA();
    Leso_Update_EmfEstB();
    Leso_Update();

    SMO_Update(&SmoHandle, Foc_Uclark_Ref, Foc_Iclark_Fdbk);

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
