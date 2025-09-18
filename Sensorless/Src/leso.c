#include "leso.h"
#include <stdbool.h>
#include "arm_math.h"
#include "filter.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

#define SQRT(x, y) arm_sqrt_f32(x, y)

// Leso_A 和 Leso_B 是为了减少运行时计算时间而预先计算好的系数，用于后续算法中直接使用。

static bool  Leso_Enabled          = {0};
static float Leso_Gain             = {0};
static float Leso_Rs               = {0};
static float Leso_Ld               = {0};
static float Leso_Lq               = {0};
static float Leso_InvLd            = {0};
static float Leso_InvLq            = {0};
static float Leso_A                = {0};
static float Leso_B                = {0};
static float Leso_SampleTime       = {0};
static float Leso_SampleFreq_Speed = {0};
static float Leso_Prescaler        = {0};
static float Leso_InvPn            = {0};
static float Leso_Wc               = {0};
static float Leso_Theta_Temp       = {0};
static float Leso_Theta            = {0};
static float Leso_Speed            = {0};

static volatile float Leso_Theta_Err = {0};
static volatile float Leso_Speed_Err = {0};

static Clark_t Leso_Voltage = {0};
static Clark_t Leso_Current = {0};
static Clark_t Leso_CurEst  = {0};
static Clark_t Leso_EmfEst  = {0};

static LowPassFilter_t Leso_EmfA_Filter  = {0};
static LowPassFilter_t Leso_EmfB_Filter  = {0};
static LowPassFilter_t Leso_Speed_Filter = {0};
static PID_Handler_t   Leso_Theta_PID    = {0};

/**
 * @brief 设置SMO采样时间相关参数
 * @note 该函数应在 Leso_Initialization 之前调用，因为部分参数依赖采样时间进行计算。
 * @param config 指向 SystemTimeConfig_t 结构体的指针，包含采样时间、速度频率和预分频参数
 */
bool Leso_Set_SampleTime(const SystemTimeConfig_t* config) {
    if (config == NULL) {
        return false;
    }

    Leso_SampleTime       = config->current.val;  // 采样时间
    Leso_SampleFreq_Speed = config->speed.inv;    // 速度频率倒数
    Leso_Prescaler        = config->prescaler;    // 预分频系数

    return true;
}

/**
 * @brief 设置SMO参数
 * @note 需要先调用 Leso_Set_SampleTime 函数，再调用本函数进行参数设置。
 * @param param 指向 LESO_Param_t 结构体的指针，包含所需参数
 */
bool Leso_Initialization(const LESO_Param_t* param) {
    if (param == NULL) {
        return false;
    }

    Leso_Gain  = param->leso_gain;
    Leso_Rs    = param->Rs;
    Leso_Ld    = param->Ld;
    Leso_Lq    = param->Lq;
    Leso_InvLd = 1.0F / Leso_Ld;
    Leso_InvLq = 1.0F / Leso_Lq;
    Leso_A     = 1 - (Leso_Rs * Leso_InvLd * Leso_SampleTime);
    Leso_B     = Leso_SampleTime * Leso_InvLd;

    return true;
}

void Leso_Set_InvPn(float inv_Pn) {
    Leso_InvPn = inv_Pn;
}

void Leso_Set_EmfFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Leso_EmfA_Filter, cutoff_freq, sample_freq);
    LowPassFilter_Init(&Leso_EmfB_Filter, cutoff_freq, sample_freq);
    Leso_Wc = M_2PI * cutoff_freq;
}

void Leso_Set_SpeedFilter(float cutoff_freq, float sample_freq) {
    LowPassFilter_Init(&Leso_Speed_Filter, cutoff_freq, sample_freq);
}

void Leso_Set_Pid_Handler(PID_Handler_t config) {
    Leso_Theta_PID = config;
}

void Leso_Set_Voltage(Clark_t voltage) {
    Leso_Voltage = voltage;
}

void Leso_Set_Current(Clark_t current) {
    Leso_Current = current;
}

void Leso_Set_Theta(float theta) {
    Leso_Theta_Temp = theta;
}

void Leso_Set_Theta_Err(float ref) {
    float err      = wrap_theta_2pi(ref - Leso_Theta + PI) - PI;
    Leso_Theta_Err = rad2deg(err);
}

void Leso_Set_Speed_Err(float ref) {
    Leso_Speed_Err = ref - Leso_Speed;
}

void Leso_Set_Enabled(bool enabled) {
    Leso_Enabled = enabled;
}

bool Leso_Get_Enabled(void) {
    return Leso_Enabled;
}

AngleResult_t Leso_Get_Result(void) {
    AngleResult_t result = {0};
    result.theta         = Leso_Theta;
    result.speed         = Leso_Speed;
    return result;
}

Clark_t Leso_Get_EmfEst(void) {
    return Leso_EmfEst;
}

static inline float sign(float x) {
    return (x > 0) - (x < 0);
}

void Leso_Update_EmfEst(void) {
    // 更新电动势估计值
    Leso_CurEst.a = Leso_A * Leso_CurEst.a
                    + Leso_B * (Leso_Voltage.a - Leso_EmfEst.a);
    Leso_CurEst.b = Leso_A * Leso_CurEst.b
                    + Leso_B * (Leso_Voltage.b - Leso_EmfEst.b);

    float est_delta = 0.0F;

    est_delta     = Leso_CurEst.a - Leso_Current.a;
    Leso_EmfEst.a = Leso_Gain * sign(est_delta);

    est_delta     = Leso_CurEst.b - Leso_Current.b;
    Leso_EmfEst.b = Leso_Gain * sign(est_delta);
}

static inline float compensate_theta(float theta, float omega) {
    // 角度补偿
    float comp = 0.0F;
    ATAN2(omega, Leso_Wc, &comp);
    // ATAN2(Leso_atan1, Leso_atan2, &Leso_atan_out);
    return theta + comp;
}

static inline float pll_update(float error, bool reset) {
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Leso_Theta_PID);
    Leso_Theta_Temp += omega * Leso_SampleTime;
    if (Leso_Theta_Temp > M_2PI) {
        Leso_Theta_Temp -= M_2PI;
    }
    if (Leso_Theta_Temp < 0.0F) {
        Leso_Theta_Temp += M_2PI;
    }
    Leso_Theta = compensate_theta(Leso_Theta_Temp, omega);
    Leso_Theta = wrap_theta_2pi(Leso_Theta);
    return omega;
}

static inline float calculate_error(Clark_t emf, float angle) {
    float angleErr   = 0.0F;
    float errorAlpha = 0.0F;
    float errorBeta  = 0.0F;

    if (!Leso_Enabled) {
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
    static uint16_t leso_count = 0x0000U;
    static float    leso_integ = 0.0F;
    float           speed      = 0.0F;
    leso_integ += radps2rpm(omega) * Leso_InvPn * 0.1F;
    leso_count++;
    if (leso_count < 0x000AU) {
        return Leso_Speed;
    }
    leso_count = 0x0000U;
    speed      = LowPassFilter_Update(&Leso_Speed_Filter, leso_integ);
    leso_integ = 0.0F;
    return speed;
}

void Leso_Update_Angle(void) {
    Leso_EmfEst.a
        = LowPassFilter_Update(&Leso_EmfA_Filter, Leso_EmfEst.a);
    Leso_EmfEst.b
        = LowPassFilter_Update(&Leso_EmfB_Filter, Leso_EmfEst.b);

    float error = calculate_error(Leso_EmfEst, Leso_Theta_Temp);

    // 计算电动势的相位角
    float omega = pll_update(error, !Leso_Enabled);
    Leso_Speed  = calculate_speed(omega);
}
