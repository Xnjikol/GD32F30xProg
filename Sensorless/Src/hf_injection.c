/**
 * @file hf_injection.c
 * @brief 脉振高频注入无传感器控制实现
 * @author ZFY
 * @date 2025年7月25日
 * @version 1.0
 *
 * 该文件实现了脉振高频注入无传感器控制算法，
 * 用于低速和零速下的位置和速度估计
 */

#include "hf_injection.h"
#include <stdbool.h>
#include <stddef.h>
#include "filter.h"
#include "pll.h"
#include "reciprocal.h"
#include "signal.h"
#include "transformation.h"

static bool  Hfi_Enabled    = {0};
static bool  Hfi_ParamError = {0};
static float Hfi_InjFreq    = {0};
static float Hfi_InjVolt    = {0};
static float Hfi_Ld         = {0};
static float Hfi_Lq         = {0};
static float Hfi_Delta_L    = {0};
static float Hfi_SampleTime = {0};
static float Hfi_SampleFreq = {0};

static float   Hfi_Theta      = {0};
static float   Hfi_Omega      = {0};
static float   Hfi_Speed      = {0};
static float   Hfi_Phase      = {0};
static float   Hfi_Error      = {0};
static Clark_t Hfi_IClarkFdbk = {0};
static Park_t  Hfi_IParkFdbk  = {0};
static Park_t  Hfi_IParkResp  = {0};  // 提取的高频响应电流
static Park_t  Hfi_IParkFilt  = {0};  // 高频滤波后的电流
static Clark_t Hfi_IClarkFilt = {0};
static Park_t  Hfi_VoltageInj = {0};  //将要注入的高频电压

static BandPassFilter_t Hfi_Current_Filter = {0};
static LowPassFilter_t  Hfi_Error_Filter   = {0};
static Pll_Handler_t    Hfi_PLL            = {0};
static SawtoothWave_t   Hfi_Phase_Gen      = {0};

bool Hfi_Set_SampleTime(const SystemTimeConfig_t* time_config) {
    if (time_config == NULL) {
        Hfi_ParamError = true;
        return false;
    }

    /* 设置采样时间 */
    Hfi_SampleTime = time_config->current.val;
    Hfi_SampleFreq = time_config->current.inv;
    return true;
}

bool Hfi_Initialization(const hf_injection_params_t* params) {
    if (params == NULL) {
        Hfi_ParamError = true;
        return false;
    }

    Hfi_InjFreq = params->injection_freq;
    Hfi_InjVolt = params->injection_voltage;
    Hfi_Ld      = params->Ld;
    Hfi_Lq      = params->Lq;
    Hfi_Delta_L = params->delta_L;

    SawtoothWave_Init(
        &Hfi_Phase_Gen, 1, Hfi_InjFreq, 0.0F, Hfi_SampleTime);

    return true;
}

void Hfi_Set_BandPassFilter(float center_freq,
                            float band_width,
                            float sample_freq) {
    BandPassFilter_Init(
        &Hfi_Current_Filter, sample_freq, center_freq, band_width);
}

void Hfi_Set_LowPassFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Hfi_Error_Filter, cutoff_freq, sample_freq);
}

void Hfi_Set_PLL(const pll_params_t* pll_params) {
    PLL_Init(&Hfi_PLL, pll_params);
}

void Hfi_Set_Current(Clark_t current) {
    Hfi_IClarkFdbk = current;
    ParkTransform(&current, Hfi_Theta, &Hfi_IParkFdbk);
}

void Hfi_Set_Enabled(bool enabled) {
    Hfi_Enabled = enabled;
}

bool Hfi_Get_Enabled(void) {
    return Hfi_Enabled;
}

Park_t Hfi_Get_FilteredCurrent(void) {
    return Hfi_IParkFilt;
}

/**
 * @brief 生成高频注入信号
 */
static inline void generate_signal() {
    /* 更新高频相位 */
    Hfi_Phase = SawtoothWaveGenerator(&Hfi_Phase_Gen, false) * M_2PI;

    /* 生成脉振高频注入信号 (d轴注入) */
    float cos_hf     = COS(Hfi_Phase);
    Hfi_VoltageInj.d = Hfi_InjVolt * cos_hf;
    Hfi_VoltageInj.q = 0.0f;
}

Park_t Hfi_Get_Injection(void) {
    return Hfi_VoltageInj;
}

Clark_t Hfi_Apply_Injection(Park_t vol) {
    Clark_t out;
    vol.d += Hfi_VoltageInj.d;
    vol.q += Hfi_VoltageInj.q;
    InvParkTransform(&vol, Hfi_Theta, &out);
    return out;
}

static inline Park_t extract_high_frequency_response(void) {
    if (!Hfi_Enabled) {
        Hfi_IParkResp = (Park_t){0};
        return (Park_t){0};
    }

    Hfi_IParkResp.d
        = BandPassFilter_Update(&Hfi_Current_Filter, Hfi_IParkFdbk.d);
    Hfi_IParkResp.q
        = BandPassFilter_Update(&Hfi_Current_Filter, Hfi_IParkFdbk.q);

    Hfi_IParkFilt.d = Hfi_IParkFdbk.d - Hfi_IParkResp.d;
    Hfi_IParkFilt.q = Hfi_IParkFdbk.q - Hfi_IParkResp.q;

    /* 反Park变换得到αβ轴电流 */
    InvParkTransform(&Hfi_IParkFilt, Hfi_Theta, &Hfi_IClarkFilt);

    return Hfi_IParkResp;
}

static inline float calculate_position_error(void) {
    float position_error = 0.0F;

    if (!Hfi_Enabled) {
        Hfi_Error = 0.0F;
        return Hfi_Error;
    }

    float sin_hf = SIN(Hfi_Phase);

    /* 计算位置误差 */
    position_error = Hfi_IParkResp.d * 2 * sin_hf;
    position_error
        = LowPassFilter_Update(&Hfi_Error_Filter, position_error);

    Hfi_Error = position_error;
    return position_error;
}

static inline float calculate_speed(void) {
    Hfi_Theta = PLL_Update(&Hfi_PLL, Hfi_Error);
    Hfi_Omega = PLL_GetSpeed(&Hfi_PLL);
    Hfi_Speed = radps2rpm(Hfi_Omega);

    return Hfi_Speed;
}

void Hfi_Update(void) {
    extract_high_frequency_response();
    calculate_position_error();
    calculate_speed();
}

void Hfi_Set_InitialPosition(float theta) {
    Hfi_Theta = theta;
    PLL_SetInitialPosition(&Hfi_PLL, theta);
}

AngleResult_t Hfi_Get_Result(void) {
    AngleResult_t angle_result;
    angle_result.theta = Hfi_Theta;
    angle_result.speed = Hfi_Speed;
    return angle_result;
}
