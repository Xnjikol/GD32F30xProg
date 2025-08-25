#include "smo.h"
#include <stdbool.h>
#include "filter.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"
#include <math.h>

// Smo_A 和 Smo_B 是为了减少运行时计算时间而预先计算好的系数，用于后续算法中直接使用。

static float           Smo_Gain             = {0};
static float           Smo_Rs               = {0};
static float           Smo_Ld               = {0};
static float           Smo_Lq               = {0};
static float           Smo_InvLd            = {0};
static float           Smo_InvLq            = {0};
static float           Smo_A                = {0};
static float           Smo_B                = {0};
static float           Smo_SampleTime       = {0};
static float           Smo_SampleFreq_Speed = {0};
static float           Smo_Prescaler        = {0};
static float           Smo_InvPn            = {0};
static float           Smo_Theta            = {0};
static float           Smo_Speed            = {0};
static Clark_t         Smo_Voltage          = {0};
static Clark_t         Smo_Current          = {0};
static Clark_t         Smo_CurEst           = {0};
static Clark_t         Smo_EmfEst           = {0};
static LowPassFilter_t Smo_Emf_A_Filter     = {0};
static LowPassFilter_t Smo_Emf_B_Filter     = {0};
static LowPassFilter_t Smo_Speed_Filter     = {0};
static PID_Handler_t   Smo_Theta_PID        = {0};
static bool            Smo_Enabled          = false;

/**
 * @brief 设置SMO采样时间相关参数
 * @note 该函数应在 Smo_Set_Parameters 之前调用，因为部分参数依赖采样时间进行计算。
 * @param config 指向 SystemTimeConfig_t 结构体的指针，包含采样时间、速度频率和预分频参数
 */
void Smo_Set_SampleTime(const SystemTimeConfig_t* config) {
    if (config != NULL) {
        Smo_SampleTime       = config->current.val;  // 采样时间
        Smo_SampleFreq_Speed = config->speed.inv;    // 速度频率倒数
        Smo_Prescaler        = config->prescaler;    // 预分频系数
    }
}

/**
 * @brief 设置SMO参数
 * @note 需要先调用 Smo_Set_SampleTime 函数，再调用本函数进行参数设置。
 * @param param 指向 SMO_Param_t 结构体的指针，包含所需参数
 */
void Smo_Set_Parameters(const SMO_Param_t* param) {
    if (param != NULL) {
        Smo_Gain  = param->smo_gain;
        Smo_Rs    = param->Rs;
        Smo_Ld    = param->Ld;
        Smo_Lq    = param->Lq;
        Smo_InvLd = 1.0F / Smo_Ld;
        Smo_InvLq = 1.0F / Smo_Lq;
        Smo_A     = 1 - (Smo_Rs * Smo_InvLd * Smo_SampleTime);
        Smo_B     = Smo_SampleTime * Smo_InvLd;
    }
}

void Smo_Set_InvPn(float inv_Pn) {
    Smo_InvPn = inv_Pn;
}

void Smo_Set_EmfFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Smo_Emf_A_Filter, cutoff_freq, sample_freq);
    LowPassFilter_Init(&Smo_Emf_B_Filter, cutoff_freq, sample_freq);
}
void Smo_Set_SpeedFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Smo_Speed_Filter, cutoff_freq, sample_freq);
}

void Smo_Set_Pid(PID_Handler_t config) {
    Smo_Theta_PID = config;
}

void Smo_Set_Voltage(Clark_t voltage) {
    Smo_Voltage = voltage;
}

void Smo_Set_Current(Clark_t current) {
    Smo_Current = current;
}

void Smo_Set_Theta(float theta) {
    Smo_Theta = theta;
}

void Smo_Set_Enabled(bool enabled) {
    Smo_Enabled = enabled;
}

bool Smo_Get_Enabled(void) {
    return Smo_Enabled;
}

AngleResult_t Smo_Get_Result(void) {
    AngleResult_t result = {0};
    result.theta         = Smo_Theta;
    result.speed         = Smo_Speed;
    return result;
}

static inline float sign(float x) {
    return (x > 0) - (x < 0);
}

void Smo_Update_EmfEst(void) {
    // 更新电动势估计值
    Smo_CurEst.a
        = Smo_A * Smo_CurEst.a + Smo_B * (Smo_Voltage.a - Smo_EmfEst.a);
    Smo_CurEst.b
        = Smo_A * Smo_CurEst.b + Smo_B * (Smo_Voltage.b - Smo_EmfEst.b);

    float est_delta = 0.0F;

    est_delta    = Smo_CurEst.a - Smo_Current.a;
    Smo_EmfEst.a = Smo_Gain * sign(est_delta);

    est_delta    = Smo_CurEst.b - Smo_Current.b;
    Smo_EmfEst.b = Smo_Gain * sign(est_delta);
}

static inline float Pll_Update(float error, bool reset) {
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Smo_Theta_PID);
    Smo_Theta += omega * Smo_SampleTime;
    return omega;
}

static inline float Calc_Error(Clark_t emf, float limit) {
    double norm = sqrt(emf.a * emf.a + emf.b * emf.b);
    double diff = emf.a * cos(Smo_Theta) + emf.b * sin(Smo_Theta);
    return (norm > limit) ? (diff / norm) : (diff / limit);
}

void Smo_Update_Angle(void) {
    Clark_t emf_filtered = {0};
    emf_filtered.a
        = LowPassFilter_Update(&Smo_Emf_A_Filter, Smo_EmfEst.a);
    emf_filtered.b
        = LowPassFilter_Update(&Smo_Emf_B_Filter, Smo_EmfEst.b);

    float error = Calc_Error(emf_filtered, 1e-4F);

    // 计算电动势的相位角
    float omega = Pll_Update(error, Smo_Enabled);
    float n     = radps2rpm(omega) * Smo_InvPn;
    Smo_Speed   = LowPassFilter_Update(&Smo_Speed_Filter, n);
}
