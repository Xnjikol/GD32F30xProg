#include "sensorless.h"
#include "math_utils.h"
#include <string.h>
#include <math.h>

/*======================*/
/*    Global Variables  */
/*======================*/

Sensorless_t Sensorless;

/*======================*/
/*    Private Functions */
/*======================*/

static void Sensorless_AlgorithmSwitch(void);
static void Sensorless_InitAllAlgorithms(void);

/*======================*/
/*    Main Functions    */
/*======================*/

/**
 * @brief 无位置传感器算法初始化
 */
void Sensorless_Init(void)
{
    memset(&Sensorless, 0, sizeof(Sensorless_t));

    /* 设置默认参数 */
    Sensorless.algorithm = SENSORLESS_NONE;
    Sensorless.motor_state = MOTOR_STOP;
    Sensorless.speed_threshold_up = SPEED_THRESHOLD_HFI_TO_SMO;
    Sensorless.speed_threshold_down = SPEED_THRESHOLD_SMO_TO_HFI;
    Sensorless.switch_counter = 0;
    Sensorless.enabled = 0;

    /* 初始化所有算法 */
    Sensorless_InitAllAlgorithms();
}

/**
 * @brief 无位置传感器算法更新
 * @param ia A相电流
 * @param ib B相电流
 * @param ic C相电流
 * @param ua A相电压
 * @param ub B相电压
 * @param uc C相电压
 */
void Sensorless_Update(float ia, float ib, float ic, float ua, float ub, float uc)
{
    if (!Sensorless.enabled)
    {
        return;
    }

    /* Clarke变换 */
    float ialpha = ia;
    float ibeta = 0.57735026919f * (ia + 2.0f * ib);
    float ualpha = ua;
    float ubeta = 0.57735026919f * (ua + 2.0f * ub);

    /* 根据当前算法更新对应的观测器 */
    switch (Sensorless.algorithm)
    {
    case SENSORLESS_HFI:
        /* 高频注入需要dq轴电流和电压 */
        if (Sensorless.hfi.enabled)
        {
            float cos_theta = COS(Sensorless.theta_est);
            float sin_theta = SIN(Sensorless.theta_est);

            /* Park变换 */
            float id = ialpha * cos_theta + ibeta * sin_theta;
            float iq = -ialpha * sin_theta + ibeta * cos_theta;
            float ud = ualpha * cos_theta + ubeta * sin_theta;
            float uq = -ualpha * sin_theta + ubeta * cos_theta;

            HFI_Update(&Sensorless.hfi, id, iq, ud, uq);
            Sensorless.theta_est = HFI_GetTheta(&Sensorless.hfi);
        }
        break;

    case SENSORLESS_SMO:
        if (Sensorless.smo.enabled)
        {
            SMO_Update(&Sensorless.smo, ialpha, ibeta, ualpha, ubeta);
            Sensorless.theta_est = SMO_GetTheta(&Sensorless.smo);
            Sensorless.speed_est = SMO_GetSpeed(&Sensorless.smo);
        }
        break;

    case SENSORLESS_HYBRID:
        /* 混合算法：根据速度自动切换 */
        Sensorless_AlgorithmSwitch();
        /* 更新当前激活的算法 */
        Sensorless_Update(ia, ib, ic, ua, ub, uc);
        break;

    case SENSORLESS_FLUX_OBSERVER:
        if (Sensorless.flux_observer.enabled)
        {
            FluxObserver_Update(&Sensorless.flux_observer, ialpha, ibeta, ualpha, ubeta);
            Sensorless.theta_est = FluxObserver_GetTheta(&Sensorless.flux_observer);
            Sensorless.speed_est = FluxObserver_GetSpeed(&Sensorless.flux_observer);
        }
        break;

    default:
        /* 无算法或未知算法 */
        break;
    }

    /* 角度归一化 */
    Sensorless.theta_est = MathUtils_WrapAngle2Pi(Sensorless.theta_est);
}

/**
 * @brief 设置无位置传感器算法
 * @param algorithm 算法类型
 */
void Sensorless_SetAlgorithm(SensorlessAlgorithm_t algorithm)
{
    /* 禁用当前算法 */
    switch (Sensorless.algorithm)
    {
    case SENSORLESS_HFI:
        Sensorless.hfi.enabled = 0;
        break;
    case SENSORLESS_SMO:
        Sensorless.smo.enabled = 0;
        break;
    case SENSORLESS_FLUX_OBSERVER:
        Sensorless.flux_observer.enabled = 0;
        break;
    default:
        break;
    }

    /* 设置新算法 */
    Sensorless.algorithm = algorithm;

    /* 使能新算法 */
    switch (algorithm)
    {
    case SENSORLESS_HFI:
        Sensorless.hfi.enabled = 1;
        break;
    case SENSORLESS_SMO:
        Sensorless.smo.enabled = 1;
        break;
    case SENSORLESS_HYBRID:
        /* 混合算法默认从HFI开始 */
        Sensorless.hfi.enabled = 1;
        Sensorless.smo.enabled = 1;
        break;
    case SENSORLESS_FLUX_OBSERVER:
        Sensorless.flux_observer.enabled = 1;
        break;
    default:
        break;
    }
}

/**
 * @brief 获取估计的转子位置
 * @return 转子位置 (rad)
 */
float Sensorless_GetTheta(void)
{
    return Sensorless.theta_est;
}

/**
 * @brief 获取估计的转子速度
 * @return 转子速度 (rpm)
 */
float Sensorless_GetSpeed(void)
{
    return Sensorless.speed_est;
}

/*======================*/
/*    HFI Functions     */
/*======================*/

/**
 * @brief 高频注入算法初始化
 * @param hfi 高频注入结构体指针
 */
void HFI_Init(HFI_t *hfi)
{
    memset(hfi, 0, sizeof(HFI_t));

    hfi->freq = HFI_INJECTION_FREQ;
    hfi->voltage = HFI_INJECTION_VOLTAGE;
    hfi->theta = 0.0f;

    /* 初始化滤波器 */
    LowPassFilter_Init(&hfi->lpf_d, HFI_FILTER_CUTOFF, T_2K_HZ);
    LowPassFilter_Init(&hfi->lpf_q, HFI_FILTER_CUTOFF, T_2K_HZ);
    BandPassFilter_Init(&hfi->bpf_d, HFI_INJECTION_FREQ, 200.0f, T_2K_HZ);
    BandPassFilter_Init(&hfi->bpf_q, HFI_INJECTION_FREQ, 200.0f, T_2K_HZ);

    hfi->enabled = 0;
}

/**
 * @brief 高频注入算法更新
 * @param hfi 高频注入结构体指针
 * @param id d轴电流
 * @param iq q轴电流
 * @param ud d轴电压
 * @param uq q轴电压
 */
void HFI_Update(HFI_t *hfi, float id, float iq, float ud, float uq)
{
    /* 标记未使用的参数以避免编译警告 */
    (void)ud;
    (void)uq;

    if (!hfi->enabled)
    {
        return;
    }

    /* 更新注入信号相位 */
    hfi->theta += 2.0f * M_PI * hfi->freq * T_2K_HZ;
    if (hfi->theta >= 2.0f * M_PI)
    {
        hfi->theta -= 2.0f * M_PI;
    }

    hfi->cos_theta = COS(hfi->theta);
    hfi->sin_theta = SIN(hfi->theta);

    /* 生成注入信号 */
    hfi->ud_inj = hfi->voltage * hfi->cos_theta;
    hfi->uq_inj = 0.0f; /* 只在d轴注入 */

    /* 提取高频响应电流 */
    float id_hf = BandPassFilter_Update(&hfi->bpf_d, id);
    float iq_hf = BandPassFilter_Update(&hfi->bpf_q, iq);

    /* 解调 */
    hfi->demod_d = id_hf * hfi->cos_theta;
    hfi->demod_q = iq_hf * hfi->cos_theta;

    /* 低通滤波 */
    hfi->demod_d = LowPassFilter_Update(&hfi->lpf_d, hfi->demod_d);
    hfi->demod_q = LowPassFilter_Update(&hfi->lpf_q, hfi->demod_q);

    /* 位置估计 */
    hfi->error = hfi->demod_q;

    /* 简单的PI控制器用于位置估计 */
    static float integral = 0.0f;
    static float kp = 100.0f;
    static float ki = 10000.0f;

    integral += hfi->error * T_2K_HZ;

    /* 积分限幅 */
    if (integral > 100.0f)
        integral = 100.0f;
    if (integral < -100.0f)
        integral = -100.0f;

    float correction = kp * hfi->error + ki * integral;
    hfi->theta_est += correction * T_2K_HZ;

    /* 角度归一化 */
    hfi->theta_est = MathUtils_WrapAngle2Pi(hfi->theta_est);
}

/**
 * @brief 获取高频注入电压
 * @param hfi 高频注入结构体指针
 * @param ud_inj d轴注入电压输出
 * @param uq_inj q轴注入电压输出
 */
void HFI_GetInjectionVoltage(HFI_t *hfi, float *ud_inj, float *uq_inj)
{
    if (hfi->enabled)
    {
        *ud_inj = hfi->ud_inj;
        *uq_inj = hfi->uq_inj;
    }
    else
    {
        *ud_inj = 0.0f;
        *uq_inj = 0.0f;
    }
}

/**
 * @brief 获取高频注入估计的转子位置
 * @param hfi 高频注入结构体指针
 * @return 转子位置 (rad)
 */
float HFI_GetTheta(HFI_t *hfi)
{
    return hfi->theta_est;
}

/*======================*/
/*    SMO Functions     */
/*======================*/

/**
 * @brief 滑膜观测器初始化
 * @param smo 滑膜观测器结构体指针
 */
void SMO_Init(SMO_t *smo)
{
    memset(smo, 0, sizeof(SMO_t));

    smo->gain_k = SMO_GAIN_K;
    smo->hysteresis = SMO_HYSTERESIS;

    /* 初始化滤波器 */
    LowPassFilter_Init(&smo->lpf_ealpha, SMO_FILTER_CUTOFF, T_2K_HZ);
    LowPassFilter_Init(&smo->lpf_ebeta, SMO_FILTER_CUTOFF, T_2K_HZ);

    smo->enabled = 0;
}

/**
 * @brief 滑膜观测器更新
 * @param smo 滑膜观测器结构体指针
 * @param ialpha alpha轴电流
 * @param ibeta beta轴电流
 * @param ualpha alpha轴电压
 * @param ubeta beta轴电压
 */
void SMO_Update(SMO_t *smo, float ialpha, float ibeta, float ualpha, float ubeta)
{
    if (!smo->enabled)
    {
        return;
    }

    /* 电流估计误差 */
    float error_alpha = ialpha - smo->ialpha_est;
    float error_beta = ibeta - smo->ibeta_est;

    /* 滑膜控制律 */
    float sign_alpha = (error_alpha > smo->hysteresis) ? 1.0f : (error_alpha < -smo->hysteresis) ? -1.0f
                                                                                                 : error_alpha / smo->hysteresis;
    float sign_beta = (error_beta > smo->hysteresis) ? 1.0f : (error_beta < -smo->hysteresis) ? -1.0f
                                                                                              : error_beta / smo->hysteresis;

    /* 滑膜项 */
    float sliding_alpha = smo->gain_k * sign_alpha;
    float sliding_beta = smo->gain_k * sign_beta;

    /* 电流估计更新 */
    extern Motor_Parameter_t Motor;
    float Rs = Motor.Rs;
    float Ls = (Motor.Ld + Motor.Lq) * 0.5f; /* 平均电感 */

    smo->ialpha_est += (ualpha - Rs * smo->ialpha_est - sliding_alpha) * T_2K_HZ / Ls;
    smo->ibeta_est += (ubeta - Rs * smo->ibeta_est - sliding_beta) * T_2K_HZ / Ls;

    /* 反EMF提取 */
    smo->ealpha = sliding_alpha;
    smo->ebeta = sliding_beta;

    /* 反EMF滤波 */
    smo->ealpha_filt = LowPassFilter_Update(&smo->lpf_ealpha, smo->ealpha);
    smo->ebeta_filt = LowPassFilter_Update(&smo->lpf_ebeta, smo->ebeta);

    /* 位置估计 */
    smo->theta_est = atan2f(smo->ebeta_filt, smo->ealpha_filt) - M_PI_2;
    smo->theta_est = MathUtils_WrapAngle2Pi(smo->theta_est);

    /* 速度估计 */
    static float last_theta = 0.0f;
    float delta_theta = smo->theta_est - last_theta;

    /* 处理角度跳变 */
    if (delta_theta > M_PI)
    {
        delta_theta -= 2.0f * M_PI;
    }
    else if (delta_theta < -M_PI)
    {
        delta_theta += 2.0f * M_PI;
    }

    smo->speed_est = delta_theta * F_2K_HZ * 60.0f / (2.0f * M_PI);
    last_theta = smo->theta_est;
}

/**
 * @brief 获取滑膜观测器估计的转子位置
 * @param smo 滑膜观测器结构体指针
 * @return 转子位置 (rad)
 */
float SMO_GetTheta(SMO_t *smo)
{
    return smo->theta_est;
}

/**
 * @brief 获取滑膜观测器估计的转子速度
 * @param smo 滑膜观测器结构体指针
 * @return 转子速度 (rpm)
 */
float SMO_GetSpeed(SMO_t *smo)
{
    return smo->speed_est;
}

/*======================*/
/*    PLL Functions     */
/*======================*/

/**
 * @brief PLL锁相环初始化
 * @param pll PLL结构体指针
 */
void PLL_Init(PLL_t *pll)
{
    memset(pll, 0, sizeof(PLL_t));

    pll->kp = PLL_KP;
    pll->ki = PLL_KI;
    pll->max_freq = PLL_MAX_FREQ;
    pll->enabled = 0;
}

/**
 * @brief PLL锁相环更新
 * @param pll PLL结构体指针
 * @param sin_theta sin(实际位置)
 * @param cos_theta cos(实际位置)
 */
void PLL_Update(PLL_t *pll, float sin_theta, float cos_theta)
{
    if (!pll->enabled)
    {
        return;
    }

    /* 计算相位误差 */
    float sin_est = SIN(pll->theta_est);
    float cos_est = COS(pll->theta_est);

    pll->error = sin_theta * cos_est - cos_theta * sin_est;

    /* PI控制器 */
    pll->integral += pll->error * T_2K_HZ;

    /* 积分限幅 */
    float integral_limit = pll->max_freq * 2.0f * M_PI / pll->ki;
    if (pll->integral > integral_limit)
    {
        pll->integral = integral_limit;
    }
    else if (pll->integral < -integral_limit)
    {
        pll->integral = -integral_limit;
    }

    /* 计算频率 */
    float freq = pll->kp * pll->error + pll->ki * pll->integral;

    /* 频率限幅 */
    if (freq > pll->max_freq * 2.0f * M_PI)
    {
        freq = pll->max_freq * 2.0f * M_PI;
    }
    else if (freq < -pll->max_freq * 2.0f * M_PI)
    {
        freq = -pll->max_freq * 2.0f * M_PI;
    }

    /* 积分得到位置 */
    pll->theta_est += freq * T_2K_HZ;
    pll->theta_est = MathUtils_WrapAngle2Pi(pll->theta_est);

    /* 更新速度估计 */
    pll->speed_est = freq * 60.0f / (2.0f * M_PI);
}

/**
 * @brief 获取PLL估计的转子位置
 * @param pll PLL结构体指针
 * @return 转子位置 (rad)
 */
float PLL_GetTheta(PLL_t *pll)
{
    return pll->theta_est;
}

/**
 * @brief 获取PLL估计的转子速度
 * @param pll PLL结构体指针
 * @return 转子速度 (rpm)
 */
float PLL_GetSpeed(PLL_t *pll)
{
    return pll->speed_est;
}

/*======================*/
/*  Twelve Pulse Functions */
/*======================*/

/**
 * @brief 十二脉冲算法初始化
 * @param pulse 十二脉冲结构体指针
 */
void TwelvePulse_Init(TwelvePulse_t *pulse)
{
    memset(pulse, 0, sizeof(TwelvePulse_t));

    pulse->test_voltage = 8.0f; /* 测试电压 8V */
    pulse->test_time = 0.001f;   /* 每个脉冲持续时间 10ms */
    pulse->settle_time = 0.02f; /* 稳定时间 20ms */
    pulse->completed = 0;
    pulse->enabled = 0;
    pulse->state = TWELVE_PULSE_IDLE;
    pulse->current_pulse = 0;
    pulse->pulse_timer = 0.0f;
    pulse->measurement_samples = 0;
    pulse->theta_est = 0.0f;
    pulse->confidence = 0.0f;

    /* 初始化滤波器 */
    for (uint8_t i = 0; i < 12; i++)
    {
        pulse->id_response[i] = 0.0f;
        pulse->iq_response[i] = 0.0f;
        pulse->current_magnitude[i] = 0.0f;
        pulse->response_quality[i] = 0.0f;
    }
}

/**
 * @brief 启动十二脉冲测试
 * @param pulse 十二脉冲结构体指针
 */
void TwelvePulse_Start(TwelvePulse_t *pulse)
{
    pulse->current_pulse = 0;
    pulse->completed = 0;
    pulse->enabled = 1;
    pulse->state = TWELVE_PULSE_APPLYING;
    pulse->pulse_timer = 0.0f;
    pulse->measurement_samples = 0;
    pulse->theta_est = 0.0f;
    pulse->confidence = 0.0f;

    /* 清空响应数据 */
    memset(pulse->id_response, 0, sizeof(pulse->id_response));
    memset(pulse->iq_response, 0, sizeof(pulse->iq_response));
    memset(pulse->current_magnitude, 0, sizeof(pulse->current_magnitude));
    memset(pulse->response_quality, 0, sizeof(pulse->response_quality));
}

/**
 * @brief 十二脉冲算法更新
 * @param pulse 十二脉冲结构体指针
 * @param id d轴电流
 * @param iq q轴电流
 */
void TwelvePulse_Update(TwelvePulse_t *pulse, float id, float iq)
{
    if (!pulse->enabled || pulse->completed)
    {
        return;
    }

    /* 更新定时器 */
    pulse->pulse_timer += T_2K_HZ;

    switch (pulse->state)
    {
    case TWELVE_PULSE_IDLE:
        /* 空闲状态，等待启动 */
        break;

    case TWELVE_PULSE_APPLYING:
        /* 电压注入阶段 */
        if (pulse->pulse_timer >= pulse->test_time)
        {
            pulse->state = TWELVE_PULSE_MEASURING;
            pulse->pulse_timer = 0.0f;
            pulse->measurement_samples = 0;
            pulse->id_response[pulse->current_pulse] = 0.0f;
            pulse->iq_response[pulse->current_pulse] = 0.0f;
        }
        break;

    case TWELVE_PULSE_MEASURING:
        /* 电流响应测量阶段 */
        if (pulse->pulse_timer >= pulse->settle_time)
        {
            /* 累积电流响应 */
            pulse->id_response[pulse->current_pulse] += id;
            pulse->iq_response[pulse->current_pulse] += iq;
            pulse->measurement_samples++;

            /* 测量完成 */
            if (pulse->measurement_samples >= 10)
            { /* 采样10次取平均 */
                /* 计算平均值 */
                pulse->id_response[pulse->current_pulse] /= pulse->measurement_samples;
                pulse->iq_response[pulse->current_pulse] /= pulse->measurement_samples;

                /* 计算电流幅值 */
                pulse->current_magnitude[pulse->current_pulse] =
                    sqrtf(pulse->id_response[pulse->current_pulse] * pulse->id_response[pulse->current_pulse] +
                          pulse->iq_response[pulse->current_pulse] * pulse->iq_response[pulse->current_pulse]);

                /* 计算响应质量 */
                pulse->response_quality[pulse->current_pulse] =
                    fabsf(pulse->id_response[pulse->current_pulse]) +
                    fabsf(pulse->iq_response[pulse->current_pulse]);

                /* 下一个脉冲 */
                pulse->current_pulse++;

                if (pulse->current_pulse >= 12)
                {
                    /* 完成所有脉冲，进入计算阶段 */
                    pulse->state = TWELVE_PULSE_CALCULATING;
                }
                else
                {
                    /* 准备下一个脉冲 */
                    pulse->state = TWELVE_PULSE_APPLYING;
                    pulse->pulse_timer = 0.0f;
                }
            }
        }
        break;

    case TWELVE_PULSE_CALCULATING:
        /* 计算初始位置 */
        TwelvePulse_CalculateInitialPosition(pulse);
        pulse->state = TWELVE_PULSE_COMPLETED;
        pulse->completed = 1;
        pulse->enabled = 0;
        break;

    case TWELVE_PULSE_COMPLETED:
        /* 测试完成 */
        break;
    }
}

/**
 * @brief 计算十二脉冲的初始位置
 * @param pulse 十二脉冲结构体指针
 */
void TwelvePulse_CalculateInitialPosition(TwelvePulse_t *pulse)
{
    float response_sum = 0.0f;
    float weighted_angle = 0.0f;
    float max_response = 0.0f;
    uint8_t max_index = 0;

    /* 方法1：寻找最大响应角度 */
    for (uint8_t i = 0; i < 12; i++)
    {
        if (pulse->current_magnitude[i] > max_response)
        {
            max_response = pulse->current_magnitude[i];
            max_index = i;
        }
    }

    /* 方法2：加权平均算法 */
    for (uint8_t i = 0; i < 12; i++)
    {
        float angle = i * M_PI / 6.0f; /* 30度间隔 */
        float weight = pulse->current_magnitude[i];

        /* 使用复数形式计算加权平均角度 */
        weighted_angle += angle * weight;
        response_sum += weight;
    }

    if (response_sum > 0.0f)
    {
        pulse->theta_raw = weighted_angle / response_sum;
    }
    else
    {
        pulse->theta_raw = max_index * M_PI / 6.0f;
    }

    /* 方法3：谐波分析算法 */
    float cos_sum = 0.0f, sin_sum = 0.0f;
    for (uint8_t i = 0; i < 12; i++)
    {
        float angle = i * M_PI / 6.0f;
        float weight = pulse->current_magnitude[i];

        cos_sum += COS(angle) * weight;
        sin_sum += SIN(angle) * weight;
    }

    if (cos_sum != 0.0f || sin_sum != 0.0f)
    {
        pulse->theta_filtered = atan2f(sin_sum, cos_sum);
        if (pulse->theta_filtered < 0.0f)
        {
            pulse->theta_filtered += 2.0f * M_PI;
        }
    }
    else
    {
        pulse->theta_filtered = pulse->theta_raw;
    }

    /* 选择最终结果 */
    pulse->theta_est = pulse->theta_filtered;

    /* 计算置信度 */
    if (response_sum > 0.0f)
    {
        pulse->confidence = max_response / response_sum;
        /* 检查响应的一致性 */
        float variance = 0.0f;
        for (uint8_t i = 0; i < 12; i++)
        {
            float diff = pulse->current_magnitude[i] - (response_sum / 12.0f);
            variance += diff * diff;
        }
        variance /= 12.0f;

        /* 方差越小，置信度越高 */
        if (variance < 0.1f)
        {
            pulse->confidence = fminf(pulse->confidence * 1.2f, 1.0f);
        }
    }
    else
    {
        pulse->confidence = 0.0f;
    }
}

/**
 * @brief 获取十二脉冲的电压参考值
 * @param pulse 十二脉冲结构体指针
 * @param ud_ref d轴电压参考输出
 * @param uq_ref q轴电压参考输出
 */
void TwelvePulse_GetVoltageReference(TwelvePulse_t *pulse, float *ud_ref, float *uq_ref)
{
    *ud_ref = 0.0f;
    *uq_ref = 0.0f;

    if (!pulse->enabled || pulse->completed)
    {
        return;
    }

    /* 只在电压注入阶段输出测试电压 */
    if (pulse->state == TWELVE_PULSE_APPLYING)
    {
        /* 计算当前脉冲的角度 */
        float angle = pulse->current_pulse * M_PI / 6.0f; /* 30度间隔 */

        /* 在该角度上注入电压 */
        *ud_ref = pulse->test_voltage * COS(angle);
        *uq_ref = pulse->test_voltage * SIN(angle);
    }
}

/**
 * @brief 获取十二脉冲的置信度
 * @param pulse 十二脉冲结构体指针
 * @return 置信度 (0.0-1.0)
 */
float TwelvePulse_GetConfidence(TwelvePulse_t *pulse)
{
    return pulse->confidence;
}

/**
 * @brief 获取十二脉冲估计的转子位置
 * @param pulse 十二脉冲结构体指针
 * @return 转子位置 (rad)
 */
float TwelvePulse_GetTheta(TwelvePulse_t *pulse)
{
    return pulse->theta_est;
}

/**
 * @brief 检查十二脉冲算法是否完成
 * @param pulse 十二脉冲结构体指针
 * @return 1-完成，0-未完成
 */
uint8_t TwelvePulse_IsCompleted(TwelvePulse_t *pulse)
{
    return pulse->completed;
}

/*======================*/
/* Flux Observer Functions */
/*======================*/

/**
 * @brief 磁链观测器初始化
 * @param observer 磁链观测器结构体指针
 */
void FluxObserver_Init(FluxObserver_t *observer)
{
    memset(observer, 0, sizeof(FluxObserver_t));

    extern Motor_Parameter_t Motor;
    observer->Rs = Motor.Rs;
    observer->Ls = (Motor.Ld + Motor.Lq) * 0.5f;

    /* 初始化高通滤波器 */
    HighPassFilter_Init(&observer->hpf_alpha, 5.0f, T_2K_HZ);
    HighPassFilter_Init(&observer->hpf_beta, 5.0f, T_2K_HZ);

    observer->enabled = 0;
}

/**
 * @brief 磁链观测器更新
 * @param observer 磁链观测器结构体指针
 * @param ialpha alpha轴电流
 * @param ibeta beta轴电流
 * @param ualpha alpha轴电压
 * @param ubeta beta轴电压
 */
void FluxObserver_Update(FluxObserver_t *observer, float ialpha, float ibeta, float ualpha, float ubeta)
{
    if (!observer->enabled)
    {
        return;
    }

    /* 磁链估计 */
    observer->psi_alpha += (ualpha - observer->Rs * ialpha) * T_2K_HZ;
    observer->psi_beta += (ubeta - observer->Rs * ibeta) * T_2K_HZ;

    /* 高通滤波去除积分漂移 */
    observer->psi_alpha_filt = HighPassFilter_Update(&observer->hpf_alpha, observer->psi_alpha);
    observer->psi_beta_filt = HighPassFilter_Update(&observer->hpf_beta, observer->psi_beta);

    /* 位置估计 */
    observer->theta_est = atan2f(observer->psi_beta_filt, observer->psi_alpha_filt);
    observer->theta_est = MathUtils_WrapAngle2Pi(observer->theta_est);

    /* 速度估计 */
    static float last_theta = 0.0f;
    float delta_theta = observer->theta_est - last_theta;

    /* 处理角度跳变 */
    if (delta_theta > M_PI)
    {
        delta_theta -= 2.0f * M_PI;
    }
    else if (delta_theta < -M_PI)
    {
        delta_theta += 2.0f * M_PI;
    }

    observer->speed_est = delta_theta * F_2K_HZ * 60.0f / (2.0f * M_PI);
    last_theta = observer->theta_est;
}

/**
 * @brief 获取磁链观测器估计的转子位置
 * @param observer 磁链观测器结构体指针
 * @return 转子位置 (rad)
 */
float FluxObserver_GetTheta(FluxObserver_t *observer)
{
    return observer->theta_est;
}

/**
 * @brief 获取磁链观测器估计的转子速度
 * @param observer 磁链观测器结构体指针
 * @return 转子速度 (rpm)
 */
float FluxObserver_GetSpeed(FluxObserver_t *observer)
{
    return observer->speed_est;
}

/*======================*/
/*  Utility Functions   */
/*======================*/

/**
 * @brief 状态转换函数
 * @param new_state 新状态
 */
void Sensorless_StateTransition(MotorState_t new_state)
{
    Sensorless.motor_state = new_state;

    switch (new_state)
    {
    case MOTOR_STOP:
        Sensorless.enabled = 0;
        break;
    case MOTOR_STARTUP:
        /* 启动时使用十二脉冲确定初始位置 */
        TwelvePulse_Start(&Sensorless.pulse);
        break;
    case MOTOR_LOW_SPEED:
        /* 低速使用HFI */
        Sensorless_SetAlgorithm(SENSORLESS_HFI);
        break;
    case MOTOR_HIGH_SPEED:
        /* 高速使用SMO */
        Sensorless_SetAlgorithm(SENSORLESS_SMO);
        break;
    }
}

/*======================*/
/*  Private Functions   */
/*======================*/

/**
 * @brief 算法切换逻辑
 */
static void Sensorless_AlgorithmSwitch(void)
{
    static SensorlessAlgorithm_t last_algorithm = SENSORLESS_NONE;

    /* 根据速度进行算法切换 */
    if (fabsf(Sensorless.speed_est) > Sensorless.speed_threshold_up)
    {
        /* 高速，切换到SMO */
        if (last_algorithm != SENSORLESS_SMO)
        {
            Sensorless.hfi.enabled = 0;
            Sensorless.smo.enabled = 1;
            last_algorithm = SENSORLESS_SMO;
        }
    }
    else if (fabsf(Sensorless.speed_est) < Sensorless.speed_threshold_down)
    {
        /* 低速，切换到HFI */
        if (last_algorithm != SENSORLESS_HFI)
        {
            Sensorless.smo.enabled = 0;
            Sensorless.hfi.enabled = 1;
            last_algorithm = SENSORLESS_HFI;
        }
    }
}

/**
 * @brief 初始化所有算法
 */
static void Sensorless_InitAllAlgorithms(void)
{
    HFI_Init(&Sensorless.hfi);
    SMO_Init(&Sensorless.smo);
    PLL_Init(&Sensorless.pll);
    TwelvePulse_Init(&Sensorless.pulse);
    FluxObserver_Init(&Sensorless.flux_observer);
}
