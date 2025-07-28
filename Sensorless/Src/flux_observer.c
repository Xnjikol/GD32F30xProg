/**
 * @file flux_observer.c
 * @brief 磁链观测器实现文件
 * @author FRECON
 * @date 2025年7月24日
 * @version 1.0
 *
 * 该文件实现了基于电压模型的磁链观测器算法，
 * 用于无传感器控制中的磁链估计
 */

#include "flux_observer.h"

#include <math.h>
#include <string.h>

#include "filter.h"
#include "transformation.h"

/* 私有宏定义 */
#define FLUX_MIN_THRESHOLD 0.001f /*!< 磁链最小阈值 */

/* 私有函数声明 */
static void flux_observer_calc_polar(flux_observer_t* observer);

/**
 * @brief 磁链观测器初始化
 */
int flux_observer_init(flux_observer_t*              observer,
                       const flux_observer_params_t* params) {
    /* 参数有效性检查 */
    if (observer == NULL || params == NULL) {
        return -1;
    }

    if (params->Rs <= 0.0f || params->Ls <= 0.0f || params->Ts <= 0.0f
        || params->flux_rated <= 0.0f) {
        return -1;
    }

    /* 清零结构体 */
    memset(observer, 0, sizeof(flux_observer_t));

    /* 保存参数 */
    flux_observer_set_params(observer, params);

    /* 设置初始化标志 */
    observer->initialized = 1;

    return 0;
}

/**
 * @brief 磁链观测器重置
 */
void flux_observer_reset(flux_observer_t* observer) {
    if (observer == NULL || !observer->initialized) {
        return;
    }

    /* 重置观测状态 */
    observer->flux->a    = 0.0f;
    observer->flux->b    = 0.0f;
    observer->flux_mag   = 0.0f;
    observer->flux_angle = 0.0f;

    LowPassFilter_Reset(observer->lpf_alpha);
    LowPassFilter_Reset(observer->lpf_beta);
}

/**
 * @brief 磁链观测器执行
 * 基于电压模型: ψ = ∫(u - Rs*i)dt
 */
void flux_observer_execute(flux_observer_t* observer,
                           Clark_t*         voltage,
                           Clark_t*         current) {
    if (observer == NULL || !observer->initialized) {
        return;
    }

    /* 更新输入量 */
    observer->voltage = voltage;
    observer->current = current;

    /* 计算反电动势 */
    float emf_alpha = voltage->a - observer->Rs * current->a;
    float emf_beta  = voltage->b - observer->Rs * current->b;

    /* 积分计算磁链 (梯形积分) */
    observer->flux->a = observer->flux_prev->a + (emf_alpha * observer->Ts);
    observer->flux->b = observer->flux_prev->b + (emf_beta * observer->Ts);

    /* 低通滤波去除直流偏移 */
    if (observer->lpf_alpha == NULL) {
        observer->lpf_alpha
            = (LowPassFilter_t*) calloc(1, sizeof(LowPassFilter_t));
    }
    if (observer->lpf_beta == NULL) {
        observer->lpf_beta
            = (LowPassFilter_t*) calloc(1, sizeof(LowPassFilter_t));
    }

    // observer->flux_alpha_lpf = observer->flux_alpha_lpf * (1.0f -
    // observer->lpf_coeff) +
    //                            observer->flux_alpha * observer->lpf_coeff;
    // observer->flux_beta_lpf = observer->flux_beta_lpf * (1.0f -
    // observer->lpf_coeff) +
    //                           observer->flux_beta * observer->lpf_coeff;

    /* 去除直流分量 */
    observer->flux->a
        = LowPassFilter_Update(observer->lpf_alpha, observer->flux->a);
    observer->flux->b
        = LowPassFilter_Update(observer->lpf_beta, observer->flux->b);

    /* 保存当前值作为下次的前一次值 */
    observer->flux_prev->a = observer->flux->a;
    observer->flux_prev->b = observer->flux->b;

    /* 计算磁链幅值和角度 */
    flux_observer_calc_polar(observer);
}

/**
 * @brief 获取磁链观测值
 */
void flux_observer_get_flux(const flux_observer_t* observer,
                            float*                 flux_alpha,
                            float*                 flux_beta) {
    if (observer == NULL || !observer->initialized) {
        if (flux_alpha)
            *flux_alpha = 0.0f;
        if (flux_beta)
            *flux_beta = 0.0f;
        return;
    }

    if (flux_alpha)
        *flux_alpha = observer->flux->a;
    if (flux_beta)
        *flux_beta = observer->flux->b;
}

/**
 * @brief 获取磁链幅值和角度
 */
void flux_observer_get_polar(const flux_observer_t* observer,
                             float*                 flux_mag,
                             float*                 flux_angle) {
    if (observer == NULL || !observer->initialized) {
        if (flux_mag)
            *flux_mag = 0.0f;
        if (flux_angle)
            *flux_angle = 0.0f;
        return;
    }

    if (flux_mag)
        *flux_mag = observer->flux_mag;
    if (flux_angle)
        *flux_angle = observer->flux_angle;
}

/**
 * @brief 设置磁链观测器参数
 */
int flux_observer_set_params(flux_observer_t*              observer,
                             const flux_observer_params_t* params) {
    if (observer == NULL || params == NULL) {
        return -1;
    }

    /* 参数有效性检查 */
    if (params->Rs <= 0.0f || params->Ls <= 0.0f || params->Ts <= 0.0f
        || params->flux_rated <= 0.0f) {
        return -1;
    }

    /* 保存参数 */
    observer->Rs          = params->Rs;
    observer->Ls          = params->Ls;
    observer->Ts          = params->Ts;
    observer->flux_rated  = params->flux_rated;
    observer->cutoff_freq = params->cutoff_freq;

    return 0;
}

/**
 * @brief 获取磁链观测器参数
 */
void flux_observer_get_params(const flux_observer_t*  observer,
                              flux_observer_params_t* params) {
    if (observer == NULL || params == NULL || !observer->initialized) {
        return;
    }

    params->Rs          = observer->Rs;
    params->Ls          = observer->Ls;
    params->Ts          = observer->Ts;
    params->flux_rated  = observer->flux_rated;
    params->cutoff_freq = observer->cutoff_freq;
}

/* 私有函数实现 */
/**
 * @brief 计算磁链幅值和角度
 */
static void flux_observer_calc_polar(flux_observer_t* observer) {
    Clark_t* flux = observer->flux;
    /* 计算磁链幅值 */
    observer->flux_mag = sqrtf(flux->a * flux->a + flux->b * flux->b);

    /* 计算磁链角度 */
    if (observer->flux_mag > FLUX_MIN_THRESHOLD) {
        observer->flux_angle = atan2f(flux->b, flux->a);

        /* 角度范围调整到 [0, 2π] */
        observer->flux_angle = wrap_theta_2pi(observer->flux_angle);
    } else {
        /* 磁链过小时保持角度不变 */
        observer->flux_angle = 0.0f;
    }
}
