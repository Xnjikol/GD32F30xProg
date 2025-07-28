/**
 * @file hf_injection.c
 * @brief 脉振高频注入无传感器控制实现
 * @author FRECON
 * @date 2025年7月25日
 * @version 1.0
 *
 * 该文件实现了脉振高频注入无传感器控制算法，
 * 用于低速和零速下的位置和速度估计
 */

#include "hf_injection.h"

#include <string.h>


/* 私有宏定义 */
#ifndef SQRT3
#define SQRT3 1.73205080757F
#endif

#ifndef SQRT3_2
#define SQRT3_2 0.86602540378F /* √3/2 */
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846F /* π */
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692F /* 2π */
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923F /* π/2 */
#endif
#define POSITION_ERROR_MAX 0.1f    /* 最大位置误差 (rad) */
#define CONVERGENCE_TIME 1000      /* 收敛时间计数器 */
#define MIN_INJECTION_FREQ 500.0f  /* 最小注入频率 (Hz) */
#define MAX_INJECTION_FREQ 5000.0f /* 最大注入频率 (Hz) */

/* 私有函数声明 */
static void HF_Injection_InitFilters(hf_injection_t* hf_inj);
static void HF_Injection_UpdatePhase(hf_injection_t* hf_inj);
static void HF_Injection_ExtractHighFreqCurrent(hf_injection_t* hf_inj, const Clark_t* current_ab);
static void HF_Injection_CalculatePositionError(hf_injection_t* hf_inj);
static void HF_Injection_PositionTracking(hf_injection_t* hf_inj);

/**
 * @brief 初始化高频注入观测器
 */
int HF_Injection_Init(hf_injection_t* hf_inj, const hf_injection_params_t* params)
{
  if (hf_inj == NULL || params == NULL)
  {
    return -1;
  }

  /* 参数检查 */
  if (params->injection_freq < MIN_INJECTION_FREQ || params->injection_freq > MAX_INJECTION_FREQ ||
      params->injection_voltage <= 0.0f || params->Ts <= 0.0f || params->delta_L == 0.0f)
  {
    return -1;
  }

  /* 复制参数 */
  memcpy(&hf_inj->params, params, sizeof(hf_injection_params_t));

  /* 初始化状态 */
  memset(&hf_inj->state, 0, sizeof(hf_injection_state_t));

  /* 设置PI控制器参数 */
  hf_inj->state.kp_track = 100.0f;
  hf_inj->state.ki_track = 1000.0f;

  /* 初始化高频正弦波生成器 */
  SineWave_Init(&hf_inj->state.hf_sine_gen, 
                1.0f,                    // amplitude = 1.0，后续与电压幅值相乘
                params->injection_freq,  // 注入频率
                0.0f,                    // phase = 0
                params->Ts);             // 采样周期

  /* 初始化滤波器 */
  HF_Injection_InitFilters(hf_inj);

  /* 使能标志 */
  hf_inj->state.is_enabled = 0;
  hf_inj->state.is_converged = 0;

  return 0;
}

/**
 * @brief 反初始化高频注入观测器
 */
void HF_Injection_DeInit(hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return;
  }

  /* 释放滤波器内存 */
  if (hf_inj->state.lpf_epsilon != NULL)
  {
    LowPassFilter_Reset(hf_inj->state.lpf_epsilon);
    free(hf_inj->state.lpf_epsilon);
    hf_inj->state.lpf_epsilon = NULL;
  }

  if (hf_inj->state.bpf_current != NULL)
  {
    BandPassFilter_Reset(hf_inj->state.bpf_current);
    free(hf_inj->state.bpf_current);
    hf_inj->state.bpf_current = NULL;
  }

  if (hf_inj->state.hpf_current != NULL)
  {
    HighPassFilter_Reset(hf_inj->state.hpf_current);
    free(hf_inj->state.hpf_current);
    hf_inj->state.hpf_current = NULL;
  }

  /* 清零状态 */
  memset(&hf_inj->state, 0, sizeof(hf_injection_state_t));
}

/**
 * @brief 生成高频注入信号
 */
void HF_Injection_GenerateSignal(hf_injection_t* hf_inj, Clark_t* v_inj_ab)
{
  if (hf_inj == NULL || v_inj_ab == NULL || !hf_inj->state.is_enabled)
  {
    if (v_inj_ab != NULL)
    {
      v_inj_ab->a = 0.0f;
      v_inj_ab->b = 0.0f;
    }
    return;
  }

  /* 更新高频相位 */
  hf_inj->state.hf_phase_current = hf_inj->state.hf_sine_gen.theta;

  /* 生成脉振高频注入信号 (d轴注入) */
  float cos_hf = SineWaveGenerator(&hf_inj->state.hf_sine_gen, false);
  hf_inj->state.v_hf_dq.d = hf_inj->params.injection_voltage * cos_hf;
  hf_inj->state.v_hf_dq.q = 0.0f;

  /* 将dq轴注入电压转换到αβ轴 */
  Clark_t clarke_voltage;
  InvParkTransform(&hf_inj->state.v_hf_dq, hf_inj->state.theta_est, &clarke_voltage);

  v_inj_ab->a = clarke_voltage.a;
  v_inj_ab->b = clarke_voltage.b;

  /* 保存注入电压 */
  hf_inj->state.v_hf_ab.a = clarke_voltage.a;
  hf_inj->state.v_hf_ab.b = clarke_voltage.b;
}

/**
 * @brief 处理高频电流响应并估计位置
 */
void HF_Injection_ProcessResponse(hf_injection_t* hf_inj, const Clark_t* current_ab)
{
  if (hf_inj == NULL || current_ab == NULL || !hf_inj->state.is_enabled)
  {
    return;
  }

  /* 保存当前αβ轴电流 */
  hf_inj->state.current_ab = (Clark_t*) current_ab;

  /* 提取高频电流分量 */
  HF_Injection_ExtractHighFreqCurrent(hf_inj, current_ab);

  /* 计算位置误差信号 */
  HF_Injection_CalculatePositionError(hf_inj);

  /* 位置跟踪控制 */
  HF_Injection_PositionTracking(hf_inj);

  /* 计算速度 */
  float dt = hf_inj->params.Ts;
  float theta_diff = wrap_theta_pi(hf_inj->state.theta_hf - hf_inj->state.theta_prev);
  hf_inj->state.omega_hf = theta_diff / dt;

  /* 更新前一次位置 */
  hf_inj->state.theta_prev = hf_inj->state.theta_hf;

  /* 检查收敛性 */
  if (fabsf(hf_inj->state.epsilon_filtered) < POSITION_ERROR_MAX)
  {
    static uint16_t converge_counter = 0;
    converge_counter++;
    if (converge_counter > CONVERGENCE_TIME)
    {
      hf_inj->state.is_converged = 1;
      converge_counter = CONVERGENCE_TIME;
    }
  }
  else
  {
    hf_inj->state.is_converged = 0;
  }
}

/**
 * @brief 获取估计的转子位置
 */
float HF_Injection_GetPosition(const hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return 0.0f;
  }

  return hf_inj->state.theta_hf;
}

/**
 * @brief 获取估计的转子速度
 */
float HF_Injection_GetSpeed(const hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return 0.0f;
  }

  return hf_inj->state.omega_hf;
}

/**
 * @brief 使能/禁用高频注入
 */
void HF_Injection_Enable(hf_injection_t* hf_inj, uint8_t enable)
{
  if (hf_inj == NULL)
  {
    return;
  }

  hf_inj->state.is_enabled = enable;

  if (!enable)
  {
    /* 禁用时重置状态 */
    hf_inj->state.is_converged = 0;
    SineWaveGenerator(&hf_inj->state.hf_sine_gen, true);  // 重置正弦波生成器
    hf_inj->state.integral_track = 0.0f;
  }
}

/**
 * @brief 检查高频注入是否收敛
 */
uint8_t HF_Injection_IsConverged(const hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return 0;
  }

  return hf_inj->state.is_converged;
}

/**
 * @brief 重置高频注入观测器状态
 */
void HF_Injection_Reset(hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return;
  }

  /* 重置状态变量 */
  hf_inj->state.theta_hf = 0.0f;
  hf_inj->state.omega_hf = 0.0f;
  hf_inj->state.theta_integral = 0.0f;
  hf_inj->state.theta_prev = 0.0f;
  hf_inj->state.hf_phase_current = 0.0f;
  SineWaveGenerator(&hf_inj->state.hf_sine_gen, true);  // 重置正弦波生成器
  hf_inj->state.epsilon = 0.0f;
  hf_inj->state.epsilon_filtered = 0.0f;
  hf_inj->state.integral_track = 0.0f;
  hf_inj->state.is_converged = 0;

  /* 重置滤波器 */
  if (hf_inj->state.lpf_epsilon != NULL)
  {
    LowPassFilter_Reset(hf_inj->state.lpf_epsilon);
  }
  if (hf_inj->state.bpf_current != NULL)
  {
    BandPassFilter_Reset(hf_inj->state.bpf_current);
  }
  if (hf_inj->state.hpf_current != NULL)
  {
    HighPassFilter_Reset(hf_inj->state.hpf_current);
  }
}

/**
 * @brief 设置初始位置
 */
void HF_Injection_SetInitialPosition(hf_injection_t* hf_inj, float initial_theta)
{
  if (hf_inj == NULL)
  {
    return;
  }

  hf_inj->state.theta_hf = wrap_theta_pi(initial_theta);
  hf_inj->state.theta_est = hf_inj->state.theta_hf;
  hf_inj->state.theta_prev = hf_inj->state.theta_hf;
}

/**
 * @brief 更新控制器参数
 */
void HF_Injection_UpdateParams(hf_injection_t* hf_inj, const hf_injection_params_t* params)
{
  if (hf_inj == NULL || params == NULL)
  {
    return;
  }

  /* 更新参数 */
  memcpy(&hf_inj->params, params, sizeof(hf_injection_params_t));

  /* 重新初始化滤波器 */
  HF_Injection_InitFilters(hf_inj);
}

/* 私有函数实现 */

/**
 * @brief 初始化滤波器
 */
static void HF_Injection_InitFilters(hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return;
  }

  /* 释放已存在的滤波器 */
  if (hf_inj->state.lpf_epsilon != NULL)
  {
    LowPassFilter_Reset(hf_inj->state.lpf_epsilon);
    free(hf_inj->state.lpf_epsilon);
  }

  if (hf_inj->state.bpf_current != NULL)
  {
    BandPassFilter_Reset(hf_inj->state.bpf_current);
    free(hf_inj->state.bpf_current);
  }

  if (hf_inj->state.hpf_current != NULL)
  {
    HighPassFilter_Reset(hf_inj->state.hpf_current);
    free(hf_inj->state.hpf_current);
  }

  /* 创建新的滤波器 */
  hf_inj->state.lpf_epsilon = (LowPassFilter_t*) malloc(sizeof(LowPassFilter_t));
  hf_inj->state.bpf_current = (BandPassFilter_t*) malloc(sizeof(BandPassFilter_t));
  hf_inj->state.hpf_current = (HighPassFilter_t*) malloc(sizeof(HighPassFilter_t));

  if (hf_inj->state.lpf_epsilon != NULL)
  {
    float sample_freq = 1.0f / hf_inj->params.Ts;
    LowPassFilter_Init(hf_inj->state.lpf_epsilon, hf_inj->params.cutoff_freq_lf, sample_freq);
  }

  if (hf_inj->state.bpf_current != NULL)
  {
    float sample_freq = 1.0f / hf_inj->params.Ts;
    float center_freq = hf_inj->params.injection_freq;
    float bandwidth = hf_inj->params.injection_freq * 0.2f; /* 20% 带宽 */
    float low_cutoff = center_freq - bandwidth / 2.0f;
    float high_cutoff = center_freq + bandwidth / 2.0f;
    BandPassFilter_Init(hf_inj->state.bpf_current, low_cutoff, high_cutoff, sample_freq);
  }

  if (hf_inj->state.hpf_current != NULL)
  {
    float sample_freq = 1.0f / hf_inj->params.Ts;
    HighPassFilter_Init(hf_inj->state.hpf_current, hf_inj->params.cutoff_freq_hf, sample_freq);
  }
}

/**
 * @brief 更新高频相位
 */
static void HF_Injection_UpdatePhase(hf_injection_t* hf_inj)
{
  /* 相位更新现在由锯齿波生成器自动处理 */
  /* 此函数保留以保持接口兼容性 */
  (void)hf_inj; // 避免未使用参数警告
}

/**
 * @brief 提取高频电流分量
 */
static void HF_Injection_ExtractHighFreqCurrent(hf_injection_t* hf_inj, const Clark_t* current_ab)
{
  if (hf_inj == NULL || current_ab == NULL)
  {
    return;
  }

  /* 使用带通滤波器提取高频电流分量 */
  Clark_t i_hf_filtered;

  if (hf_inj->state.bpf_current != NULL)
  {
    i_hf_filtered.a = BandPassFilter_Update(hf_inj->state.bpf_current, current_ab->a);
    i_hf_filtered.b = BandPassFilter_Update(hf_inj->state.bpf_current, current_ab->b);
  }
  else
  {
    i_hf_filtered.a = current_ab->a;
    i_hf_filtered.b = current_ab->b;
  }

  /* 转换到dq轴 */
  ParkTransform(&i_hf_filtered, hf_inj->state.theta_est, &hf_inj->state.i_hf_dq);

  /* 保存αβ轴高频电流 */
  hf_inj->state.i_hf_ab.a = i_hf_filtered.a;
  hf_inj->state.i_hf_ab.b = i_hf_filtered.b;
}

/**
 * @brief 计算位置误差信号
 */
static void HF_Injection_CalculatePositionError(hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return;
  }

  /* 解调获取位置误差信号 */
  /* 使用 id_hf * sin(ωt) 来提取位置误差 */
  /* 当前高频相位已保存在 hf_phase_current 中 */
  float sin_hf = sinf(hf_inj->state.hf_phase_current);
  
  hf_inj->state.epsilon = hf_inj->state.i_hf_dq.d * sin_hf;

  /* 低通滤波去除高频成分 */
  if (hf_inj->state.lpf_epsilon != NULL)
  {
    hf_inj->state.epsilon_filtered =
        LowPassFilter_Update(hf_inj->state.lpf_epsilon, hf_inj->state.epsilon);
  }
  else
  {
    hf_inj->state.epsilon_filtered = hf_inj->state.epsilon;
  }
}

/**
 * @brief 位置跟踪控制
 */
static void HF_Injection_PositionTracking(hf_injection_t* hf_inj)
{
  if (hf_inj == NULL)
  {
    return;
  }

  /* PI控制器 */
  float error = -hf_inj->state.epsilon_filtered; /* 负反馈 */

  /* 比例项 */
  float proportional = hf_inj->state.kp_track * error;

  /* 积分项 */
  hf_inj->state.integral_track += hf_inj->state.ki_track * error * hf_inj->params.Ts;

  /* 积分限幅 */
  float integral_limit = 1000.0f; /* rad/s */
  if (hf_inj->state.integral_track > integral_limit)
  {
    hf_inj->state.integral_track = integral_limit;
  }
  else if (hf_inj->state.integral_track < -integral_limit)
  {
    hf_inj->state.integral_track = -integral_limit;
  }

  /* 计算估计速度 */
  float omega_est = proportional + hf_inj->state.integral_track;

  /* 位置积分 */
  hf_inj->state.theta_integral += omega_est * hf_inj->params.Ts;
  hf_inj->state.theta_hf = wrap_theta_pi(hf_inj->state.theta_integral);

  /* 更新估计位置用于下次计算 */
  hf_inj->state.theta_est = hf_inj->state.theta_hf;
}
