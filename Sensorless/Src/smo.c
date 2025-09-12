#include "smo.h"
#include <stdbool.h>
#include "arm_math.h"
#include "filter.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

#define SQRT(x, y) arm_sqrt_f32(x, y)

// Smo_A 和 Smo_B 是为了减少运行时计算时间而预先计算好的系数，用于后续算法中直接使用。

static bool  Smo_Enabled          = {0};
static float Smo_Gain             = {0};
static float Smo_Rs               = {0};
static float Smo_Ld               = {0};
static float Smo_Lq               = {0};
static float Smo_InvLd            = {0};
static float Smo_InvLq            = {0};
static float Smo_A                = {0};
static float Smo_B                = {0};
static float Smo_SampleTime       = {0};
static float Smo_SampleFreq_Speed = {0};
static float Smo_Prescaler        = {0};
static float Smo_InvPn            = {0};
static float Smo_Wc               = {0};
static float Smo_Theta_Temp       = {0};
static float Smo_Theta            = {0};
static float Smo_Speed            = {0};

static volatile float Smo_Theta_Err = {0};
static volatile float Smo_Speed_Err = {0};

static Clark_t Smo_Voltage = {0};
static Clark_t Smo_Current = {0};
static Clark_t Smo_CurEst  = {0};
static Clark_t Smo_EmfEst  = {0};

static LowPassFilter_t Smo_EmfA_Filter  = {0};
static LowPassFilter_t Smo_EmfB_Filter  = {0};
static LowPassFilter_t Smo_Speed_Filter = {0};
static PID_Handler_t   Smo_Theta_PID    = {0};

/**
 * @brief 设置SMO采样时间相关参数
 * @note 该函数应在 Smo_Initialization 之前调用，因为部分参数依赖采样时间进行计算。
 * @param config 指向 SystemTimeConfig_t 结构体的指针，包含采样时间、速度频率和预分频参数
 */
bool Smo_Set_SampleTime(const SystemTimeConfig_t* config) {
    if (config == NULL) {
        return false;
    }

    Smo_SampleTime       = config->current.val;  // 采样时间
    Smo_SampleFreq_Speed = config->speed.inv;    // 速度频率倒数
    Smo_Prescaler        = config->prescaler;    // 预分频系数

    return true;
}

/**
 * @brief 设置SMO参数
 * @note 需要先调用 Smo_Set_SampleTime 函数，再调用本函数进行参数设置。
 * @param param 指向 SMO_Param_t 结构体的指针，包含所需参数
 */
bool Smo_Initialization(const SMO_Param_t* param) {
    if (param == NULL) {
        return false;
    }

    Smo_Gain  = param->smo_gain;
    Smo_Rs    = param->Rs;
    Smo_Ld    = param->Ld;
    Smo_Lq    = param->Lq;
    Smo_InvLd = 1.0F / Smo_Ld;
    Smo_InvLq = 1.0F / Smo_Lq;
    Smo_A     = 1 - (Smo_Rs * Smo_InvLd * Smo_SampleTime);
    Smo_B     = Smo_SampleTime * Smo_InvLd;

    return true;
}

void Smo_Set_InvPn(float inv_Pn) {
    Smo_InvPn = inv_Pn;
}

void Smo_Set_EmfFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Smo_EmfA_Filter, cutoff_freq, sample_freq);
    LowPassFilter_Init(&Smo_EmfB_Filter, cutoff_freq, sample_freq);
    Smo_Wc = M_2PI * cutoff_freq;
}

void Smo_Set_SpeedFilter(float cutoff_freq, float sample_freq) {
    Smo_Wc = cutoff_freq;
    LowPassFilter_Init(&Smo_Speed_Filter, cutoff_freq, sample_freq);
}

void Smo_Set_Pid_Handler(PID_Handler_t config) {
    Smo_Theta_PID = config;
}

void Smo_Set_Voltage(Clark_t voltage) {
    Smo_Voltage = voltage;
}

void Smo_Set_Current(Clark_t current) {
    Smo_Current = current;
}

void Smo_Set_Theta(float theta) {
    Smo_Theta_Temp = theta;
}

void Smo_Set_Theta_Err(float ref) {
    float err     = wrap_theta_2pi(ref - Smo_Theta + PI) - PI;
    Smo_Theta_Err = rad2deg(err);
}

void Smo_Set_Speed_Err(float ref) {
    Smo_Speed_Err = ref - Smo_Speed;
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

Clark_t Smo_Get_EmfEst(void) {
    return Smo_EmfEst;
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

static inline float compensate_theta(float theta, float omega) {
    // 角度补偿
    float comp = 0.0F;
    ATAN2(omega, Smo_Wc, &comp);
    // ATAN2(Smo_atan1, Smo_atan2, &Smo_atan_out);
    return theta + comp;
}

static inline float pll_update(float error, bool reset) {
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Smo_Theta_PID);
    Smo_Theta_Temp += omega * Smo_SampleTime;
    if (Smo_Theta_Temp > M_2PI) {
        Smo_Theta_Temp -= M_2PI;
    }
    if (Smo_Theta_Temp < 0.0F) {
        Smo_Theta_Temp += M_2PI;
    }
    Smo_Theta = compensate_theta(Smo_Theta_Temp, omega);
    Smo_Theta = wrap_theta_2pi(Smo_Theta);
    return omega;
}

static inline float calculate_error(Clark_t emf, float angle) {
    float angleErr   = 0.0F;
    float errorAlpha = 0.0F;
    float errorBeta  = 0.0F;

    if (!Smo_Enabled) {
        return angleErr;
    }

    float sin_smo = SIN(angle);
    float cos_smo = COS(angle);

    errorAlpha = -emf.a * cos_smo;
    errorBeta  = emf.b * sin_smo;
    angleErr   = errorAlpha - errorBeta;

    float norm = 0.0F;
    SQRT(emf.a * emf.a + emf.b * emf.b, &norm);
    if (norm > 0.0001F) {
        angleErr /= norm;
    } else {
        angleErr = 0.0F;
    }

    return angleErr;
}

static inline float calculate_speed(float omega) {
    static uint16_t smo_count = 0x0000U;
    static float    smo_integ = 0.0F;
    float           speed     = 0.0F;
    smo_integ += radps2rpm(omega) * Smo_InvPn * 0.1F;
    smo_count++;
    if (smo_count < 0x000AU) {
        return Smo_Speed;
    }
    smo_count = 0x0000U;
    speed     = LowPassFilter_Update(&Smo_Speed_Filter, smo_integ);
    smo_integ = 0.0F;
    return speed;
}

void Smo_Update_Angle(void) {
    Smo_EmfEst.a = LowPassFilter_Update(&Smo_EmfA_Filter, Smo_EmfEst.a);
    Smo_EmfEst.b = LowPassFilter_Update(&Smo_EmfB_Filter, Smo_EmfEst.b);

    float error = calculate_error(Smo_EmfEst, Smo_Theta_Temp);

    // 计算电动势的相位角
    float omega = pll_update(error, !Smo_Enabled);
    Smo_Speed   = calculate_speed(omega);
}
