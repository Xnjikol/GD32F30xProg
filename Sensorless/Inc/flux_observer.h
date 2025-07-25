/**
 * @file flux_observer.h
 * @brief 磁链观测器头文件
 * @author FRECON
 * @date 2025年7月24日
 * @version 1.0
 *
 * 该文件定义了磁链观测器的数据结构和函数接口，
 * 用于无传感器控制中的磁链估计算法
 */

#ifndef __FLUX_OBSERVER_H__
#define __FLUX_OBSERVER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>

#include "filter.h"
#include "theta_calc.h"
#include "transformation.h"


  /**
   * @brief 磁链观测器参数结构体
   */
  typedef struct
  {
    float Rs;          /*!< 定子电阻 (Ω) */
    float Ls;          /*!< 定子电感 (H) */
    float Ts;          /*!< 采样周期 (s) */
    float flux_rated;  /*!< 额定磁链 (Wb) */
    float cutoff_freq; /*!< 低通滤波器截止频率 (Hz) */
  } flux_observer_params_t;

  /**
   * @brief 磁链观测器状态结构体
   */
  typedef struct
  {
    /* 输入量 */
    Clarke_Data_t* voltage; /*!< αβ轴电压 (V) */
    Clarke_Data_t* current; /*!< αβ轴电流 (A) */

    /* 观测量 */
    Clarke_Data_t* flux;      /*!< αβ轴磁链观测值 (Wb) */
    Clarke_Data_t* flux_prev; /*!< αβ轴磁链积分值 (Wb) */
    float flux_mag;           /*!< 磁链幅值 (Wb) */
    float flux_angle;         /*!< 磁链角度 (rad) */
    float flux_rated;         /*!< 额定幅值 (Wb) */

    /* 内部状态 */
    LowPassFilter_t* lpf_alpha;
    LowPassFilter_t* lpf_beta;

    float Rs;
    float Ts;
    float Ls;
    float cutoff_freq;

    /* 状态标志 */
    bool initialized; /*!< 初始化标志 */
  } flux_observer_t;

  /**
   * @brief 磁链观测器初始化
   * @param observer 磁链观测器结构体指针
   * @param params 磁链观测器参数结构体指针
   * @retval 0: 成功, -1: 失败
   */
  int flux_observer_init(flux_observer_t* observer, const flux_observer_params_t* params);

  /**
   * @brief 磁链观测器重置
   * @param observer 磁链观测器结构体指针
   */
  void flux_observer_reset(flux_observer_t* observer);

  /**
   * @brief 磁链观测器执行
   * @param observer 磁链观测器结构体指针
   * @param ualpha α轴电压 (V)
   * @param ubeta β轴电压 (V)
   * @param ialpha α轴电流 (A)
   * @param ibeta β轴电流 (A)
   */
  void flux_observer_execute(flux_observer_t* observer, Clarke_Data_t* voltage,
                             Clarke_Data_t* current);

  /**
   * @brief 获取磁链观测值
   * @param observer 磁链观测器结构体指针
   * @param flux_alpha α轴磁链输出指针 (Wb)
   * @param flux_beta β轴磁链输出指针 (Wb)
   */
  void flux_observer_get_flux(const flux_observer_t* observer, float* flux_alpha, float* flux_beta);

  /**
   * @brief 获取磁链幅值和角度
   * @param observer 磁链观测器结构体指针
   * @param flux_mag 磁链幅值输出指针 (Wb)
   * @param flux_angle 磁链角度输出指针 (rad)
   */
  void flux_observer_get_polar(const flux_observer_t* observer, float* flux_mag, float* flux_angle);

  /**
   * @brief 设置磁链观测器参数
   * @param observer 磁链观测器结构体指针
   * @param params 新的参数结构体指针
   * @retval 0: 成功, -1: 失败
   */
  int flux_observer_set_params(flux_observer_t* observer, const flux_observer_params_t* params);

  /**
   * @brief 获取磁链观测器参数
   * @param observer 磁链观测器结构体指针
   * @param params 参数输出结构体指针
   */
  void flux_observer_get_params(const flux_observer_t* observer, flux_observer_params_t* params);

#ifdef __cplusplus
}
#endif

#endif /* __FLUX_OBSERVER_H__ */
