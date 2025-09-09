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
#include "theta_calc.h"
#include "transformation.h"

static bool  Hfi_Enabled    = {0};
static bool  Hfi_ParamError = {0};
static bool  Hfi_InjectSign = {0};
static float Hfi_InjFreq    = {0};
static float Hfi_InjVolt    = {0};
static float Hfi_Ld         = {0};
static float Hfi_Lq         = {0};
static float Hfi_Delta_L    = {0};
static float Hfi_InvPn      = {0};
static float Hfi_SampleTime = {0};
static float Hfi_SampleFreq = {0};

static volatile float Hfi_Theta_Err = {0};
static volatile float Hfi_Speed_Err = {0};

static float   Hfi_Theta      = {0};
static float   Hfi_Omega      = {0};
static float   Hfi_Speed      = {0};
static float   Hfi_Error      = {0};
static Clark_t Hfi_IClarkFdbk = {0};
static Park_t  Hfi_IParkFdbk  = {0};
static Clark_t Hfi_IClarkResp = {0};  // 提取的高频响应电流
static Clark_t Hfi_IClarkFilt = {0};  // 滤波后的静止坐标系电流
static Park_t  Hfi_VoltageInj = {0};  // 将要注入的高频电压

static LowPassFilter_t Hfi_Error_Filter = {0};
static LowPassFilter_t Hfi_Speed_Filter = {0};
static PID_Handler_t   Hfi_Theta_Pid    = {0};

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
    Hfi_InvPn   = params->inv_Pn;

    LowPassFilter_Init(
        &Hfi_Speed_Filter, 10.0F, Hfi_SampleFreq / 10.0F);
    LowPassFilter_Init(&Hfi_Error_Filter, 500.0F, Hfi_SampleFreq);

    return true;
}

void Hfi_Set_PidParams(const PID_Handler_t* pid_handler) {
    if (pid_handler == NULL) {
        Hfi_ParamError = true;
        return;
    }
    Hfi_Theta_Pid = *pid_handler;
}

void Hfi_Set_Current(Clark_t current) {
    Hfi_IClarkFdbk = current;
    Hfi_IParkFdbk  = ParkTransform(current, Hfi_Theta);
}

void Hfi_Set_Enabled(bool enabled) {
    Hfi_Enabled = enabled;
}

bool Hfi_Get_Enabled(void) {
    return Hfi_Enabled;
}

void Hfi_Set_Theta_Err(float ref) {
    float err     = wrap_theta_2pi(ref - Hfi_Theta + PI) - PI;
    Hfi_Theta_Err = rad2deg(err);
}

void Hfi_Set_Speed_Err(float ref) {
    Hfi_Speed_Err = ref - Hfi_Speed;
}

Clark_t Hfi_Process_Current(Clark_t current) {
    static Clark_t current_last = {0};
    static Clark_t cur_high_old = {0};
    Clark_t        cur_high_new = {0};
    Clark_t        cur_base     = {0};
    Clark_t        cur_resp     = {0};

    cur_base.a     = (current.a + current_last.a) * 0.5F;
    cur_base.b     = (current.b + current_last.b) * 0.5F;
    current_last   = current;
    Hfi_IClarkFilt = cur_base;

    cur_high_new.a = current.a - cur_base.a;
    cur_high_new.b = current.b - cur_base.b;

    cur_resp.a = cur_high_new.a - cur_high_old.a;
    cur_resp.a *= Hfi_InjectSign ? -1.0F : 1.0F;
    cur_resp.b = cur_high_new.b - cur_high_old.b;
    cur_resp.b *= Hfi_InjectSign ? -1.0F : 1.0F;

    // cur_resp.a = LowPassFilter_Update(&Hfi_Error_Filter, cur_resp.a);
    // cur_resp.b = LowPassFilter_Update(&Hfi_Error_Filter, cur_resp.b);

    cur_high_old   = cur_high_new;
    Hfi_IClarkResp = cur_resp;

    return cur_base;
}

static inline void generate_signal() {
    Park_t inj_dq = {0};
    /* 更新高频注入信号 */
    inj_dq.d = Hfi_InjVolt * (Hfi_InjectSign ? 1.0F : -1.0F);
    inj_dq.q = 0.0F;

    Hfi_VoltageInj = inj_dq;
}

static inline void update_signal() {
    Hfi_InjectSign = !Hfi_InjectSign;
}

Park_t Hfi_Get_Inject_Voltage(void) {
    update_signal();
    generate_signal();
    return Hfi_VoltageInj;
}

static inline float pll_update(float error, bool reset) {
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Hfi_Theta_Pid);
    Hfi_Theta += omega * Hfi_SampleTime;
    Hfi_Theta = wrap_theta_2pi(Hfi_Theta);

    Hfi_Omega = omega;
    return omega;
}

// static inline float calculate_error(float response) {
//     float position_error = 0.0F;

//     if (!Hfi_Enabled) {
//         return position_error;
//     }

//     float sin_hf = SIN(Hfi_Phase);

//     /* 计算位置误差 */
//     position_error = -response * 2 * sin_hf;
//     position_error
//         = LowPassFilter_Update(&Hfi_Error_Filter, position_error);

//     return position_error;
// }

static inline float calculate_error(Clark_t response, float angle) {
    float angleErr   = 0.0F;
    float errorAlpha = 0.0F;
    float errorBeta  = 0.0F;

    if (!Hfi_Enabled) {
        return angleErr;
    }

    float sin_hf = SIN(angle);
    float cos_hf = COS(angle);

    errorAlpha = -response.a * sin_hf;
    errorBeta  = response.b * cos_hf;
    angleErr   = errorAlpha + errorBeta;

    return angleErr;
}

static inline float calculate_omega(float error) {
    float omega = pll_update(error, !Hfi_Enabled);

    return omega;
}

static inline float calculate_speed(float omega) {
    static uint16_t hfi_count = 0x0000U;
    static float    hfi_integ = 0.0F;
    float           speed     = 0.0F;
    hfi_integ += radps2rpm(omega) * Hfi_InvPn * 0.1F;
    hfi_count++;
    if (hfi_count < 0x000AU) {
        return Hfi_Speed;
    }
    hfi_count = 0x0000U;
    speed     = LowPassFilter_Update(&Hfi_Speed_Filter, hfi_integ);
    hfi_integ = 0.0F;
    return speed;
}

void Hfi_Update(void) {
    float omega = 0;
    Hfi_Error   = calculate_error(Hfi_IClarkResp, Hfi_Theta);
    omega       = calculate_omega(Hfi_Error);
    Hfi_Speed   = calculate_speed(omega);
}

void Hfi_Set_InitialPosition(float theta) {
    Hfi_Theta = theta;
    PID_SetIntegral(&Hfi_Theta_Pid, !Hfi_Enabled, theta);
}

AngleResult_t Hfi_Get_Result(void) {
    AngleResult_t angle_result;
    angle_result.theta = Hfi_Theta;
    angle_result.speed = Hfi_Speed;
    return angle_result;
}
