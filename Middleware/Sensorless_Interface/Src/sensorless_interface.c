/**
 * @file sensorless_interface.c
 * @brief 无传感器控制接口层实现
 * @author FRECON
 * @date 2025年7月25日
 * @version 1.0
 */

#include "sensorless_interface.h"

#include <math.h>
#include <string.h>

#include "flux_observer.h"
#include "hf_injection.h"
#include "transformation.h"

/* 私有变量 */
static flux_observer_t g_flux_observer;     /*!< 磁链观测器实例 */
static hf_injection_t g_hf_injection;       /*!< 高频注入观测器实例 */
static sensorless_config_t g_config;        /*!< 配置参数 */
static sensorless_output_t g_output;        /*!< 输出数据 */
static sensorless_state_t g_state;          /*!< 当前状态 */
static sensorless_method_t g_active_method; /*!< 当前激活的方法 */
static uint8_t g_initialized = 0;           /*!< 初始化标志 */
static uint8_t g_enabled = 0;               /*!< 使能标志 */
static uint8_t g_hf_enabled = 0;            /*!< 高频注入使能标志 */

/* 速度计算相关 */
static float g_prev_angle = 0.0f;          /*!< 上一次角度 */
static float g_speed_lpf = 0.0f;           /*!< 速度低通滤波值 */
static const float SPEED_LPF_COEFF = 0.1f; /*!< 速度滤波系数 */

/* 私有函数声明 */
static void sensorless_update_state(void);
static void sensorless_calculate_speed(void);
static void sensorless_validate_output(void);
static void sensorless_method_switching(void);
static int sensorless_init_hf_injection(void);
static void sensorless_execute_flux_observer(Clark_t* voltage, Clark_t* current);
static void sensorless_execute_hf_injection(Clark_t* voltage, Clark_t* current,
                                            Clark_t* injection_voltage);
static void sensorless_execute_hybrid(Clark_t* voltage, Clark_t* current,
                                      Clark_t* injection_voltage);

/**
 * @brief 无传感器控制系统初始化
 */
int sensorless_init(const sensorless_config_t* config)
{
  if (config == NULL)
  {
    return -1;
  }

  /* 保存配置 */
  memcpy(&g_config, config, sizeof(sensorless_config_t));

  /* 配置磁链观测器参数 */
  flux_observer_params_t flux_params = {.Rs = config->motor_rs,
                                        .Ls = config->motor_ls,
                                        .Ts = config->control_ts,
                                        .flux_rated = config->motor_flux_rated,
                                        .cutoff_freq = config->lpf_cutoff_freq};

  /* 初始化磁链观测器 */
  if (flux_observer_init(&g_flux_observer, &flux_params) != 0)
  {
    return -1;
  }

  /* 根据配置初始化高频注入 */
  if (config->method == SENSORLESS_METHOD_HF_INJECTION ||
      config->method == SENSORLESS_METHOD_HYBRID)
  {
    if (sensorless_init_hf_injection() != 0)
    {
      return -1;
    }
  }

  /* 初始化输出结构 */
  memset(&g_output, 0, sizeof(sensorless_output_t));

  /* 设置初始状态和方法 */
  g_state = SENSORLESS_STATE_STOPPED;
  g_active_method = config->method;
  g_output.state = g_state;
  g_output.active_method = g_active_method;

  /* 重置内部变量 */
  g_prev_angle = 0.0f;
  g_speed_lpf = 0.0f;
  g_enabled = 0;
  g_hf_enabled = 0;
  g_initialized = 1;

  return 0;
}

/**
 * @brief 无传感器控制系统复位
 */
void sensorless_reset(void)
{
  if (!g_initialized)
  {
    return;
  }

  /* 重置磁链观测器 */
  flux_observer_reset(&g_flux_observer);

  /* 重置高频注入观测器 */
  if (g_config.method == SENSORLESS_METHOD_HF_INJECTION ||
      g_config.method == SENSORLESS_METHOD_HYBRID)
  {
    HF_Injection_Reset(&g_hf_injection);
  }

  /* 重置输出 */
  memset(&g_output, 0, sizeof(sensorless_output_t));

  /* 重置状态 */
  g_state = SENSORLESS_STATE_STOPPED;
  g_output.state = g_state;
  g_output.active_method = g_active_method;

  /* 重置内部变量 */
  g_prev_angle = 0.0f;
  g_speed_lpf = 0.0f;
  g_enabled = 0;
  g_hf_enabled = 0;
}

/**
 * @brief 无传感器控制主执行函数
 */
void sensorless_execute(Clark_t* voltage, Clark_t* current, Clark_t* injection_voltage)
{
  if (!g_initialized || !g_enabled)
  {
    g_output.valid = 0;
    if (injection_voltage != NULL)
    {
      injection_voltage->a = 0.0f;
      injection_voltage->b = 0.0f;
    }
    return;
  }

  /* 方法切换逻辑 */
  sensorless_method_switching();

  /* 根据当前方法执行相应算法 */
  switch (g_active_method)
  {
    case SENSORLESS_METHOD_FLUX_OBSERVER:
      sensorless_execute_flux_observer(voltage, current);
      if (injection_voltage != NULL)
      {
        injection_voltage->a = 0.0f;
        injection_voltage->b = 0.0f;
      }
      break;

    case SENSORLESS_METHOD_HF_INJECTION:
      sensorless_execute_hf_injection(voltage, current, injection_voltage);
      break;

    case SENSORLESS_METHOD_HYBRID:
      sensorless_execute_hybrid(voltage, current, injection_voltage);
      break;

    default:
      sensorless_execute_flux_observer(voltage, current);
      if (injection_voltage != NULL)
      {
        injection_voltage->a = 0.0f;
        injection_voltage->b = 0.0f;
      }
      break;
  }

  /* 计算速度 */
  sensorless_calculate_speed();

  /* 更新状态 */
  sensorless_update_state();

  /* 验证输出有效性 */
  sensorless_validate_output();

  /* 更新输出中的激活方法 */
  g_output.active_method = g_active_method;
}

/**
 * @brief 获取无传感器控制输出
 */
void sensorless_get_output(sensorless_output_t* output)
{
  if (output == NULL)
  {
    return;
  }

  memcpy(output, &g_output, sizeof(sensorless_output_t));
}

/**
 * @brief 获取转子角度
 */
float sensorless_get_rotor_angle(void)
{
  return g_output.rotor_angle;
}

/**
 * @brief 获取转子速度
 */
float sensorless_get_rotor_speed(void)
{
  return g_output.rotor_speed;
}

/**
 * @brief 获取转子机械速度
 */
float sensorless_get_rotor_speed_mech(void)
{
  return g_output.rotor_speed_mech;
}

/**
 * @brief 获取磁链幅值
 */
float sensorless_get_flux_magnitude(void)
{
  return g_output.flux_magnitude;
}

/**
 * @brief 获取当前状态
 */
sensorless_state_t sensorless_get_state(void)
{
  return g_state;
}

/**
 * @brief 检查数据有效性
 */
uint8_t sensorless_is_valid(void)
{
  return g_output.valid;
}

/**
 * @brief 设置运行状态
 */
void sensorless_set_enable(uint8_t enable)
{
  g_enabled = enable;

  if (!enable)
  {
    g_state = SENSORLESS_STATE_STOPPED;
    g_output.state = g_state;
    g_output.valid = 0;
  }
}

/**
 * @brief 更新配置参数
 */
int sensorless_update_config(const sensorless_config_t* config)
{
  if (config == NULL || !g_initialized)
  {
    return -1;
  }

  /* 保存新配置 */
  memcpy(&g_config, config, sizeof(sensorless_config_t));

  /* 更新磁链观测器参数 */
  flux_observer_params_t flux_params = {.Rs = config->motor_rs,
                                        .Ls = config->motor_ls,
                                        .Ts = config->control_ts,
                                        .flux_rated = config->motor_flux_rated,
                                        .cutoff_freq = config->lpf_cutoff_freq};

  return flux_observer_set_params(&g_flux_observer, &flux_params);
}

/**
 * @brief 获取默认配置参数
 */
void sensorless_get_default_config(sensorless_config_t* config)
{
  if (config == NULL)
  {
    return;
  }

  /* 设置默认参数 */
  config->method = SENSORLESS_METHOD_HYBRID; /* 默认使用混合方法 */
  config->motor_rs = 0.185f;                 /* 定子电阻 */
  config->motor_ls = 0.00032f;               /* 定子电感 */
  config->motor_ld = 0.00030f;               /* d轴电感 */
  config->motor_lq = 0.00035f;               /* q轴电感 */
  config->motor_flux_rated = 0.00656f;       /* 额定磁链 */
  config->motor_pole_pairs = 4.0f;           /* 极对数 */
  config->control_ts = 0.0001f;              /* 控制周期 10kHz */
  config->lpf_cutoff_freq = 5.0f;            /* 低通滤波器截止频率 */
  config->hf_injection_freq = 1000.0f;       /* 高频注入频率 */
  config->hf_injection_voltage = 5.0f;       /* 高频注入电压 */
  config->hf_cutoff_freq_hf = 2000.0f;       /* 高频滤波器截止频率 */
  config->hf_cutoff_freq_lf = 10.0f;         /* 低频滤波器截止频率 */
  config->min_speed_threshold = 10.0f;       /* 最小速度阈值 */
  config->min_flux_threshold = 0.001f;       /* 最小磁链阈值 */
  config->hf_switch_speed = 50.0f;           /* 高频注入切换速度阈值 */
}

/**
 * @brief 设置无传感器控制方法
 */
int sensorless_set_method(sensorless_method_t method)
{
  if (!g_initialized)
  {
    return -1;
  }

  g_config.method = method;
  g_active_method = method;

  /* 如果切换到高频注入或混合方法，需要初始化高频注入 */
  if ((method == SENSORLESS_METHOD_HF_INJECTION || method == SENSORLESS_METHOD_HYBRID) &&
      !g_hf_enabled)
  {
    return sensorless_init_hf_injection();
  }

  return 0;
}

/**
 * @brief 获取当前使用的控制方法
 */
sensorless_method_t sensorless_get_method(void)
{
  return g_active_method;
}

/**
 * @brief 使能/禁用高频注入
 */
void sensorless_hf_injection_enable(uint8_t enable)
{
  if (!g_initialized)
  {
    return;
  }

  g_hf_enabled = enable;

  if (g_config.method == SENSORLESS_METHOD_HF_INJECTION ||
      g_config.method == SENSORLESS_METHOD_HYBRID)
  {
    HF_Injection_Enable(&g_hf_injection, enable);
  }
}

/**
 * @brief 检查高频注入是否收敛
 */
uint8_t sensorless_hf_injection_is_converged(void)
{
  if (!g_initialized || !g_hf_enabled)
  {
    return 0;
  }

  return HF_Injection_IsConverged(&g_hf_injection);
}

/**
 * @brief 设置高频注入初始位置
 */
void sensorless_hf_injection_set_initial_position(float initial_angle)
{
  if (!g_initialized)
  {
    return;
  }

  if (g_config.method == SENSORLESS_METHOD_HF_INJECTION ||
      g_config.method == SENSORLESS_METHOD_HYBRID)
  {
    HF_Injection_SetInitialPosition(&g_hf_injection, initial_angle);
  }
}

/**
 * @brief 强制切换到指定方法
 */
int sensorless_switch_method(sensorless_method_t method, uint8_t force)
{
  if (!g_initialized)
  {
    return -1;
  }

  if (force)
  {
    /* 强制切换 */
    g_active_method = method;
    return 0;
  }
  else
  {
    /* 按条件切换 - 在sensorless_method_switching()中处理 */
    g_config.method = method;
    return 0;
  }
}

/* 私有函数实现 */

/**
 * @brief 更新系统状态
 */
static void sensorless_update_state(void)
{
  if (!g_enabled)
  {
    g_state = SENSORLESS_STATE_STOPPED;
  }
  else
  {
    /* 根据磁链幅值和速度判断状态 */
    if (g_output.flux_magnitude < g_config.min_flux_threshold)
    {
      g_state = SENSORLESS_STATE_STARTING;
    }
    else if (fabsf(g_output.rotor_speed) < g_config.min_speed_threshold)
    {
      g_state = SENSORLESS_STATE_STARTING;
    }
    else
    {
      g_state = SENSORLESS_STATE_RUNNING;
    }
  }

  g_output.state = g_state;
}

/**
 * @brief 计算转子速度
 */
static void sensorless_calculate_speed(void)
{
  float angle_diff = g_output.rotor_angle - g_prev_angle;

  /* 处理角度跳变（-π到π的跳变） */
  if (angle_diff > M_PI)
  {
    angle_diff -= 2.0f * M_PI;
  }
  else if (angle_diff < -M_PI)
  {
    angle_diff += 2.0f * M_PI;
  }

  /* 计算瞬时角速度 */
  float instant_speed = angle_diff / g_config.control_ts;

  /* 低通滤波 */
  g_speed_lpf = g_speed_lpf * (1.0f - SPEED_LPF_COEFF) + instant_speed * SPEED_LPF_COEFF;

  /* 更新输出 */
  g_output.rotor_speed = g_speed_lpf;
  g_output.rotor_speed_mech = g_speed_lpf / g_config.motor_pole_pairs;

  /* 保存当前角度 */
  g_prev_angle = g_output.rotor_angle;
}

/**
 * @brief 验证输出有效性
 */
static void sensorless_validate_output(void)
{
  /* 检查磁链幅值 */
  if (g_output.flux_magnitude < g_config.min_flux_threshold * 0.1f)
  {
    g_output.valid = 0;
    return;
  }

  /* 检查角度范围 */
  if (g_output.rotor_angle < 0.0f || g_output.rotor_angle > 2.0f * M_PI)
  {
    g_output.valid = 0;
    return;
  }

  /* 检查速度合理性 */
  if (fabsf(g_output.rotor_speed) > 10000.0f)
  { /* 假设最大速度限制 */
    g_output.valid = 0;
    return;
  }

  /* 如果所有检查通过 */
  g_output.valid = (g_state == SENSORLESS_STATE_RUNNING) ? 1 : 0;
}

/**
 * @brief 初始化高频注入观测器
 */
static int sensorless_init_hf_injection(void)
{
  /* 配置高频注入参数 */
  hf_injection_params_t hf_params = {.injection_freq = g_config.hf_injection_freq,
                                     .injection_voltage = g_config.hf_injection_voltage,
                                     .Ts = g_config.control_ts,
                                     .Ld = g_config.motor_ld,
                                     .Lq = g_config.motor_lq,
                                     .delta_L = g_config.motor_ld - g_config.motor_lq,
                                     .cutoff_freq_hf = g_config.hf_cutoff_freq_hf,
                                     .cutoff_freq_lf = g_config.hf_cutoff_freq_lf,
                                     .speed_threshold = g_config.hf_switch_speed};

  /* 初始化高频注入观测器 */
  if (HF_Injection_Init(&g_hf_injection, &hf_params) != 0)
  {
    return -1;
  }

  g_hf_enabled = 1;
  return 0;
}

/**
 * @brief 执行磁链观测器方法
 */
static void sensorless_execute_flux_observer(Clark_t* voltage, Clark_t* current)
{
  /* 执行磁链观测 */
  flux_observer_execute(&g_flux_observer, voltage, current);

  /* 获取磁链观测结果 */
  float flux_alpha, flux_beta;
  flux_observer_get_flux(&g_flux_observer, &flux_alpha, &flux_beta);
  flux_observer_get_polar(&g_flux_observer, &g_output.flux_magnitude, &g_output.flux_angle);

  /* 更新转子角度（对于表贴式PMSM，磁链角度近似等于转子角度） */
  g_output.rotor_angle = g_output.flux_angle;
  g_output.hf_converged = 0;
}

/**
 * @brief 执行高频注入方法
 */
static void sensorless_execute_hf_injection(Clark_t* voltage, Clark_t* current,
                                            Clark_t* injection_voltage)
{
  if (!g_hf_enabled)
  {
    if (injection_voltage != NULL)
    {
      // injection_voltage->a = 0.0f;
      // injection_voltage->b = 0.0f;
    }
    return;
  }

  /* 生成高频注入信号 */
  Clark_t v_inj = {0};
  HF_Injection_GenerateSignal(&g_hf_injection, &v_inj);

  if (injection_voltage != NULL)
  {
    /* 将基础电压和高频注入电压相加 */
    if (voltage != NULL)
    {
      injection_voltage->a = voltage->a + v_inj.a;
      injection_voltage->b = voltage->b + v_inj.b;
    }
    else
    {
      injection_voltage->a = v_inj.a;
      injection_voltage->b = v_inj.b;
    }
  }

  /* 处理高频电流响应 */
  HF_Injection_ProcessResponse(&g_hf_injection, current);

  /* 获取估计结果 */
  g_output.rotor_angle = HF_Injection_GetPosition(&g_hf_injection);
  g_output.rotor_speed = HF_Injection_GetSpeed(&g_hf_injection);
  g_output.hf_converged = HF_Injection_IsConverged(&g_hf_injection);

  /* 高频注入时磁链信息不可用 */
  g_output.flux_magnitude = 0.0f;
  g_output.flux_angle = g_output.rotor_angle;
}

/**
 * @brief 执行混合方法
 */
static void sensorless_execute_hybrid(Clark_t* voltage, Clark_t* current,
                                      Clark_t* injection_voltage)
{
  float current_speed = fabsf(g_output.rotor_speed);

  if (current_speed < g_config.hf_switch_speed)
  {
    /* 低速使用高频注入 */
    sensorless_execute_hf_injection(voltage, current, injection_voltage);
  }
  else
  {
    /* 高速使用磁链观测器 */
    sensorless_execute_flux_observer(voltage, current);
    if (injection_voltage != NULL)
    {
      injection_voltage->a = 0.0f;
      injection_voltage->b = 0.0f;
    }
  }
}

/**
 * @brief 方法切换逻辑
 */
static void sensorless_method_switching(void)
{
  if (g_config.method != SENSORLESS_METHOD_HYBRID)
  {
    /* 非混合方法，不需要切换 */
    return;
  }

  float current_speed = fabsf(g_output.rotor_speed);

  /* 混合方法的切换逻辑 */
  if (current_speed < g_config.hf_switch_speed * 0.8f) /* 20%滞环 */
  {
    /* 切换到高频注入 */
    if (g_active_method != SENSORLESS_METHOD_HF_INJECTION)
    {
      /* 如果高频注入未初始化，先初始化 */
      if (!g_hf_enabled)
      {
        sensorless_init_hf_injection();
      }
      HF_Injection_Enable(&g_hf_injection, 1);
      g_active_method = SENSORLESS_METHOD_HF_INJECTION;
    }
  }
  else if (current_speed > g_config.hf_switch_speed * 1.2f) /* 20%滞环 */
  {
    /* 切换到磁链观测器 */
    if (g_active_method != SENSORLESS_METHOD_FLUX_OBSERVER)
    {
      if (g_hf_enabled)
      {
        HF_Injection_Enable(&g_hf_injection, 0);
      }
      g_active_method = SENSORLESS_METHOD_FLUX_OBSERVER;
    }
  }
}