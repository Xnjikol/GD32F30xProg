/**
 * @file sensorless_interface.h
 * @brief 无传感器控制接口层
 * @author FRECON
 * @date 2025年7月25日
 * @version 1.0
 *
 * 该文件提供无传感器控制的高级接口，封装底层算法实现
 */

#ifndef __SENSORLESS_INTERFACE_H__
#define __SENSORLESS_INTERFACE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "foc_types.h"
#include "main.h"

  /**
   * @brief 无传感器控制方法枚举
   */
  typedef enum
  {
    SENSORLESS_METHOD_FLUX_OBSERVER = 0, /*!< 磁链观测器 */
    SENSORLESS_METHOD_HF_INJECTION,      /*!< 高频注入 */
    SENSORLESS_METHOD_HYBRID             /*!< 混合方法 */
  } sensorless_method_t;

  /**
   * @brief 无传感器控制状态枚举
   */
  typedef enum
  {
    SENSORLESS_STATE_STOPPED = 0, /*!< 停止状态 */
    SENSORLESS_STATE_STARTING,    /*!< 启动状态 */
    SENSORLESS_STATE_RUNNING,     /*!< 运行状态 */
    SENSORLESS_STATE_ERROR        /*!< 错误状态 */
  } sensorless_state_t;

  /**
   * @brief 无传感器控制配置参数
   */
  typedef struct
  {
    /* 控制方法选择 */
    sensorless_method_t method; /*!< 无传感器控制方法 */
    
    /* 电机参数 */
    float motor_rs;         /*!< 定子电阻 (Ω) */
    float motor_ls;         /*!< 定子电感 (H) */
    float motor_ld;         /*!< d轴电感 (H) */
    float motor_lq;         /*!< q轴电感 (H) */
    float motor_flux_rated; /*!< 额定磁链 (Wb) */
    float motor_pole_pairs; /*!< 极对数 */

    /* 控制参数 */
    float control_ts;      /*!< 控制周期 (s) */
    float lpf_cutoff_freq; /*!< 低通滤波器截止频率 (Hz) */
    
    /* 高频注入参数 */
    float hf_injection_freq;    /*!< 高频注入频率 (Hz) */
    float hf_injection_voltage; /*!< 高频注入电压幅值 (V) */
    float hf_cutoff_freq_hf;    /*!< 高频滤波器截止频率 (Hz) */
    float hf_cutoff_freq_lf;    /*!< 低频滤波器截止频率 (Hz) */

    /* 阈值参数 */
    float min_speed_threshold; /*!< 最小速度阈值 (rad/s) */
    float min_flux_threshold;  /*!< 最小磁链阈值 (Wb) */
    float hf_switch_speed;     /*!< 高频注入切换速度阈值 (rad/s) */
  } sensorless_config_t;

  /**
   * @brief 无传感器控制输出结构体
   */
  typedef struct
  {
    float rotor_angle;        /*!< 转子电角度 (rad) */
    float rotor_speed;        /*!< 转子电角速度 (rad/s) */
    float rotor_speed_mech;   /*!< 转子机械角速度 (rad/s) */
    float flux_magnitude;     /*!< 磁链幅值 (Wb) */
    float flux_angle;         /*!< 磁链角度 (rad) */
    sensorless_state_t state; /*!< 当前状态 */
    sensorless_method_t active_method; /*!< 当前激活的方法 */
    uint8_t valid;            /*!< 数据有效标志 */
    uint8_t hf_converged;     /*!< 高频注入收敛标志 */
  } sensorless_output_t;

  /**
   * @brief 无传感器控制系统初始化
   * @param config 配置参数指针
   * @retval 0: 成功, -1: 失败
   */
  int sensorless_init(const sensorless_config_t* config);

  /**
   * @brief 无传感器控制系统复位
   */
  void sensorless_reset(void);

  /**
   * @brief 无传感器控制主执行函数
   * @param voltage αβ轴电压指针
   * @param current αβ轴电流指针
   * @param injection_voltage 输出高频注入电压指针 (可为NULL)
   */
  void sensorless_execute(Clark_t* voltage, Clark_t* current, Clark_t* injection_voltage);

  /**
   * @brief 获取无传感器控制输出
   * @param output 输出结构体指针
   */
  void sensorless_get_output(sensorless_output_t* output);

  /**
   * @brief 获取转子角度
   * @retval 转子电角度 (rad)
   */
  float sensorless_get_rotor_angle(void);

  /**
   * @brief 获取转子速度
   * @retval 转子电角速度 (rad/s)
   */
  float sensorless_get_rotor_speed(void);

  /**
   * @brief 获取转子机械速度
   * @retval 转子机械角速度 (rad/s)
   */
  float sensorless_get_rotor_speed_mech(void);

  /**
   * @brief 获取磁链幅值
   * @retval 磁链幅值 (Wb)
   */
  float sensorless_get_flux_magnitude(void);

  /**
   * @brief 获取当前状态
   * @retval 无传感器控制状态
   */
  sensorless_state_t sensorless_get_state(void);

  /**
   * @brief 检查数据有效性
   * @retval 1: 有效, 0: 无效
   */
  uint8_t sensorless_is_valid(void);

  /**
   * @brief 设置运行状态
   * @param enable 1: 使能, 0: 禁用
   */
  void sensorless_set_enable(uint8_t enable);

  /**
   * @brief 更新配置参数
   * @param config 新的配置参数
   * @retval 0: 成功, -1: 失败
   */
  int sensorless_update_config(const sensorless_config_t* config);

  /**
   * @brief 获取默认配置参数
   * @param config 配置参数输出指针
   */
  void sensorless_get_default_config(sensorless_config_t* config);

  /**
   * @brief 设置无传感器控制方法
   * @param method 控制方法
   * @retval 0: 成功, -1: 失败
   */
  int sensorless_set_method(sensorless_method_t method);

  /**
   * @brief 获取当前使用的控制方法
   * @retval 当前控制方法
   */
  sensorless_method_t sensorless_get_method(void);

  /**
   * @brief 使能/禁用高频注入
   * @param enable 1: 使能, 0: 禁用
   */
  void sensorless_hf_injection_enable(uint8_t enable);

  /**
   * @brief 检查高频注入是否收敛
   * @retval 1: 已收敛, 0: 未收敛
   */
  uint8_t sensorless_hf_injection_is_converged(void);

  /**
   * @brief 设置高频注入初始位置
   * @param initial_angle 初始角度 (rad)
   */
  void sensorless_hf_injection_set_initial_position(float initial_angle);

  /**
   * @brief 强制切换到指定方法
   * @param method 目标方法
   * @param force 1: 强制切换, 0: 按条件切换
   * @retval 0: 成功, -1: 失败
   */
  int sensorless_switch_method(sensorless_method_t method, uint8_t force);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORLESS_INTERFACE_H__ */