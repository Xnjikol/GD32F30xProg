#include "leso.h"
#include <stdbool.h>
#include "Buffer.h"
#include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export
#include "filter.h"
#include "motor.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

#define SQRT(x, y) arm_sqrt_f32(x, y)

bool  Leso_Enabled = {0};
float Leso_Beta1   = {0};
float Leso_Beta2   = {0};
float Leso_Rs      = {0};
// float Motor_Ld      = {0};
// float Motor_Lq      = {0};
// float Motor_InvLd   = {0};
// float Motor_InvLq   = {0};
// float Leso_Ld               = {0};
// float Leso_Lq               = {0};
// float Leso_InvLd            = {0};
// float Leso_InvLq            = {0};
float Leso_Gain   = {0};
float Leso_Factor = {0};
float Leso_Wc     = {0};
float Leso_Wc_Min = {0};
float Leso_Wc_Max = {0};
float Leso_We     = {0};
float Leso_Theta  = {0};
float Leso_Speed  = {0};
float Leso_Error  = {0};

volatile float Leso_Theta_Err = {0};
volatile float Leso_Speed_Err = {0};
volatile float Leso_Int_limit = {0};

Clark_t Leso_Voltage = {0};
Clark_t Leso_Current = {0};
Clark_t Leso_CurEst  = {0};
Clark_t Leso_EmfEst  = {0};

IIR2ndFilter_t Leso_Speed_Filter = {0};
PID_Handler_t  Leso_Theta_PID    = {0};

/**
 * @brief 设置SMO采样时间相关参数
 * @note 该函数应在 Leso_Initialization 之前调用，因为部分参数依赖采样时间进行计算。
 * @param config 指向 SystemTimeConfig_t 结构体的指针，包含采样时间、速度频率和预分频参数
 */
bool Leso_Set_SampleTime(const SystemTimeConfig_t* config)
{
    if (config == NULL)
    {
        return false;
    }

    return true;
}

/**
 * @brief 设置SMO参数
 * @note 需要先调用 Leso_Set_SampleTime 函数，再调用本函数进行参数设置。
 * @param param 指向 LESO_Param_t 结构体的指针，包含所需参数
 */
bool Leso_Initialization(const LESO_Param_t* param)
{
    Leso_Gain   = param->wc_gain;
    Leso_Wc_Max = param->wc_max;
    Leso_Wc_Min = param->wc_min;
    Leso_Rs     = param->Rs;
    // Leso_Ld     = param->Ld;
    // Leso_Lq     = param->Lq;
    // Leso_InvLd  = 1.0F / Leso_Ld;
    // Leso_InvLq  = 1.0F / Leso_Lq;

    Leso_Int_limit = 1E35F;  // 积分限幅值

    return true;
}

void Leso_Set_Inductor(Park_t inductance)
{
    float leso_Ld = inductance.d <= 0.001 ? 0.001 : inductance.d;
    float leso_Lq = inductance.q <= 0.001 ? 0.001 : inductance.q;
    // float leso_InvLd = 1.0F / leso_Ld;
    // float leso_InvLq = 1.0F / leso_Lq;
    Park_t Idq  = ParkeTransform(Leso_Current, Leso_Theta);
    float  temp = (leso_Ld - leso_Lq) / leso_Ld * Idq.q;
    if (temp < 0.0f)
        temp = 0.0f;
    SQRT(temp, &Leso_Factor);
}

void Leso_Set_SpeedFilter(float cutoff_freq, float sample_freq)
{
    IIR2ndFilter_Init(&Leso_Speed_Filter, cutoff_freq, sample_freq);
}

void Leso_Set_Pid_Handler(PID_Handler_t config)
{
    Leso_Theta_PID = config;
}

void Leso_Set_Voltage(Clark_t voltage)
{
    Leso_Voltage = voltage;
}

void Leso_Set_Current(Clark_t current)
{
    Leso_Current = current;
}

void Leso_Set_Theta(float theta)
{
    Leso_Theta = theta;
}

void Leso_Set_Speed(float speed)
{
    Leso_Speed = speed;
    Leso_We    = rpm2radps(speed) * Motor_Pn;
}

void Leso_Calc_ThetaErr(float ref)
{
    float err      = wrap_theta_2pi(ref - Leso_Theta + PI) - PI;
    Leso_Theta_Err = rad2deg(err);
}

void Leso_Calc_SpeedErr(float ref)
{
    Leso_Speed_Err = ref - Leso_Speed;
}

void Leso_Set_Enabled(bool enabled)
{
    Leso_Enabled = enabled;
}

bool Leso_Get_Enabled(void)
{
    return Leso_Enabled;
}

MotorState_t Leso_Get_Result(void)
{
    MotorState_t result = {0};
    result.theta        = Leso_Theta;
    result.speed        = Leso_Speed;
    return result;
}

float Leso_Get_PllErr(void)
{
    return Leso_Error;
}

Clark_t Leso_Get_EmfEst(void)
{
    return Leso_EmfEst;
}

static inline float clamp_f32(float val, float min, float max)
{
    if (val > max)
    {
        return max;
    }
    if (val < min)
    {
        return min;
    }
    return val;
}

void Leso_Update_Beta(void)
{
    Leso_Wc = Leso_Gain * Leso_Factor * Leso_We;
    if (Leso_Wc > Leso_Wc_Max)
    {
        Leso_Wc = Leso_Wc_Max;
    }
    if (Leso_Wc < Leso_Wc_Min)
    {
        Leso_Wc = Leso_Wc_Min;
    }

    // 根据带宽计算观测器增益
    Leso_Beta1 = 2.0F * Leso_Wc;
    Leso_Beta2 = Leso_Wc * Leso_Wc;
}

void Leso_Update_EmfEstA(void)
{
    // 更新电动势估计值
    static float leso_f1a  = 0.0F;
    float        leso_f0   = 0.0F;
    float        leso_b0u  = 0.0F;
    float        leso_err  = 0.0F;
    float        leso_dcur = 0.0F;

    Buffer_Put(Leso_CurEst.a, 0);
    Buffer_Put(Leso_Current.a, 1);

    if (!Leso_Enabled)
    {
        leso_f1a      = 0.0F;
        Leso_CurEst.a = 0.0F;
        Leso_EmfEst.a = -Motor_Lq * leso_f1a;
        return;
    }

    leso_err = Leso_CurEst.a - Leso_Current.a;

    leso_f0  = -Leso_Current.a * Leso_Rs * Motor_InvLq;
    leso_b0u = Leso_Voltage.a * Motor_InvLq;
    leso_f1a -= Leso_Beta2 * SampleTime * leso_err;

    leso_f1a = clamp_f32(leso_f1a, -Leso_Int_limit, Leso_Int_limit);

    leso_dcur = leso_f0 + leso_b0u + leso_f1a - Leso_Beta1 * leso_err;
    Leso_CurEst.a += leso_dcur * SampleTime;

    Leso_CurEst.a = clamp_f32(Leso_CurEst.a, -Leso_Int_limit, Leso_Int_limit);
    Leso_EmfEst.a = -Motor_Lq * leso_f1a;

    return;
}

void Leso_Update_EmfEstB(void)
{
    // 更新电动势估计值
    static float leso_f1b  = 0.0F;
    float        leso_f0   = 0.0F;
    float        leso_b0u  = 0.0F;
    float        leso_err  = 0.0F;
    float        leso_dcur = 0.0F;

    Buffer_Put(Leso_CurEst.b, 2);
    Buffer_Put(Leso_Current.b, 3);

    if (!Leso_Enabled)
    {
        leso_f1b      = 0.0F;
        Leso_CurEst.b = 0.0F;
        Leso_EmfEst.b = -Motor_Lq * leso_f1b;
        return;
    }

    leso_err = Leso_CurEst.b - Leso_Current.b;

    leso_f0  = -Leso_Current.b * Leso_Rs * Motor_InvLq;
    leso_b0u = Leso_Voltage.b * Motor_InvLq;
    leso_f1b -= Leso_Beta2 * SampleTime * leso_err;

    leso_f1b = clamp_f32(leso_f1b, -Leso_Int_limit, Leso_Int_limit);

    leso_dcur = leso_f0 + leso_b0u + leso_f1b - Leso_Beta1 * leso_err;
    Leso_CurEst.b += leso_dcur * SampleTime;

    Leso_CurEst.b = clamp_f32(Leso_CurEst.b, -Leso_Int_limit, Leso_Int_limit);
    Leso_EmfEst.b = -Motor_Lq * leso_f1b;

    return;
}

static inline float pll_update(float error, bool reset)
{
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Leso_Theta_PID);
    Leso_Theta += omega * SampleTime;
    if (Leso_Theta > M_2PI)
    {
        Leso_Theta -= M_2PI;
    }
    if (Leso_Theta < 0.0F)
    {
        Leso_Theta += M_2PI;
    }
    Leso_Theta = wrap_theta_2pi(Leso_Theta);
    return omega;
}

static inline float calculate_error(Clark_t emf, float angle)
{
    float angleErr   = 0.0F;
    float errorAlpha = 0.0F;
    float errorBeta  = 0.0F;

    if (!Leso_Enabled)
    {
        return angleErr;
    }

    float sin_leso = SIN(angle);
    float cos_leso = COS(angle);

    errorAlpha = -emf.a * cos_leso;
    errorBeta  = emf.b * sin_leso;
    angleErr   = errorAlpha - errorBeta;

    float norm = 0.0F;
    SQRT(emf.a * emf.a + emf.b * emf.b, &norm);
    if (norm > 0.0001F)
    {
        angleErr /= norm;
    }
    else
    {
        angleErr = 0.0F;
    }

    return angleErr;
}

static inline float calculate_speed(float omega)
{
    static uint16_t leso_count = 0x0000U;
    static float    leso_integ = 0.0F;
    float           speed      = 0.0F;

    if (!Leso_Enabled)
    {
        leso_integ = 0.0F;
        leso_count = 0x0000U;
        Leso_Speed = 0.0F;
        return Leso_Speed;
    }

    leso_integ += radps2rpm(omega) * Motor_InvPn * 0.1F;
    leso_count++;
    if (leso_count < 0x000AU)
    {
        return Leso_Speed;
    }
    leso_count = 0x0000U;
    speed      = IIR2ndFilter_Update(&Leso_Speed_Filter, leso_integ);
    leso_integ = 0.0F;
    return speed;
}

void Leso_Update(void)
{
    Leso_Error = calculate_error(Leso_EmfEst, Leso_Theta);
}
