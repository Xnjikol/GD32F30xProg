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
#include "transformation.h"


/* 私有变量 */
static flux_observer_t g_flux_observer; /*!< 磁链观测器实例 */
static sensorless_config_t g_config;    /*!< 配置参数 */
static sensorless_output_t g_output;    /*!< 输出数据 */
static sensorless_state_t g_state;      /*!< 当前状态 */
static uint8_t g_initialized = 0;       /*!< 初始化标志 */
static uint8_t g_enabled = 0;           /*!< 使能标志 */

/* 速度计算相关 */
static float g_prev_angle = 0.0f;          /*!< 上一次角度 */
static float g_speed_lpf = 0.0f;           /*!< 速度低通滤波值 */
static const float SPEED_LPF_COEFF = 0.1f; /*!< 速度滤波系数 */

/* 私有函数声明 */
static void sensorless_update_state(void);
static void sensorless_calculate_speed(void);
static void sensorless_validate_output(void);

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

  /* 初始化输出结构 */
  memset(&g_output, 0, sizeof(sensorless_output_t));

  /* 设置初始状态 */
  g_state = SENSORLESS_STATE_STOPPED;
  g_output.state = g_state;

  /* 重置内部变量 */
  g_prev_angle = 0.0f;
  g_speed_lpf = 0.0f;
  g_enabled = 0;
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

  /* 重置输出 */
  memset(&g_output, 0, sizeof(sensorless_output_t));

  /* 重置状态 */
  g_state = SENSORLESS_STATE_STOPPED;
  g_output.state = g_state;

  /* 重置内部变量 */
  g_prev_angle = 0.0f;
  g_speed_lpf = 0.0f;
  g_enabled = 0;
}

/**
 * @brief 无传感器控制主执行函数
 */
void sensorless_execute(Clarke_Data_t* voltage, Clarke_Data_t* current)
{
  if (!g_initialized || !g_enabled)
  {
    g_output.valid = 0;
    return;
  }

  /* 执行磁链观测 */
  flux_observer_execute(&g_flux_observer, voltage, current);

  /* 获取磁链观测结果 */
  float flux_alpha, flux_beta;
  flux_observer_get_flux(&g_flux_observer, &flux_alpha, &flux_beta);
  flux_observer_get_polar(&g_flux_observer, &g_output.flux_magnitude, &g_output.flux_angle);

  /* 更新转子角度（对于表贴式PMSM，磁链角度近似等于转子角度） */
  g_output.rotor_angle = g_output.flux_angle;

  /* 计算速度 */
  sensorless_calculate_speed();

  /* 更新状态 */
  sensorless_update_state();

  /* 验证输出有效性 */
  sensorless_validate_output();
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
  config->motor_rs = 0.185f;           /* 定子电阻 */
  config->motor_ls = 0.00032f;         /* 定子电感 */
  config->motor_flux_rated = 0.00656f; /* 额定磁链 */
  config->motor_pole_pairs = 4.0f;     /* 极对数 */
  config->control_ts = 0.0001f;        /* 控制周期 10kHz */
  config->lpf_cutoff_freq = 5.0f;      /* 低通滤波器截止频率 */
  config->min_speed_threshold = 10.0f; /* 最小速度阈值 */
  config->min_flux_threshold = 0.001f; /* 最小磁链阈值 */
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