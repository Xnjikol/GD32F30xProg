#ifndef SMO_H
#define SMO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "filter.h"
#include "transformation.h"

    /*
 * smo.h
 * 滑模观测器（Sliding Mode Observer）头文件
 *
 * 简要说明：
 * - 使用 Clarke 变换的 alpha/beta 分量
 * - 基于电压/电流观测转子磁链并估计角度/速度
 * - 以 float 为默认数值类型（可按需改为 double 或定点）
 */

    /* 基本类型 */
    typedef float smo_ft;

    /* SMO 状态结构（对外可读） */
    typedef struct
    {
        /* 被估计量 */
        Clark_t Ihat; /* 电流估计 alpha/beta */
        Clark_t Ehat; /* 磁链估计 alpha/beta */
        Clark_t Fdbk; /* 反馈量 alpha/beta */

        /* 观测误差 */
        Clark_t Ierror; /* 电流误差 alpha/beta */

        /* 角度/速度估计 */
        float theta; /* 机械/电角度 [rad] */
        float speed; /* 转速 [rpm] */

        float pll_err;   /* PLL 误差 */
        float theta_err; /* 角度误差 */
        float speed_err; /* 速度误差 */
    } SMO_State_t;

    /* SMO 参数结构（可配置） */
    typedef struct
    {
        float Gain;             /* 滑模增益 */
        float K_i;              /* 抑制抖动的增益/低通项 */
        float Emf_cutoffFreq;   /* 角度/速度滤波器常数（可用于低通） */
        float Speed_cutoffFreq; /* 速度滤波器常数 */
    } SMO_Param_t;

    /* 主句柄 */
    typedef struct
    {
        SMO_Param_t param;
        SMO_State_t state;

        IIR1stFilter_t EmfFilterA;  /* 反电动势滤波器 */
        IIR1stFilter_t EmfFilterB;  /* 反电动势滤波器 */
        IIR2ndFilter_t OmegaFilter; /* 速度滤波器 */

        /* 内部用变量 */
        Clark_t sign;       /* 滑模符号函数输出 */
        Clark_t integrator; /* 积分器状态 */

        bool enabled; /* 使能标志 */
    } SMO_Handle_t;

    extern SMO_Handle_t SmoHandle;

    /* API 函数 */

    /* 初始化：设置默认参数并清零状态 */
    void SMO_Initialization(SMO_Handle_t* h, const SMO_Param_t* p);

    /* 重置状态（清估计和值） */
    void SMO_Reset(SMO_Handle_t* h);

    /*
 * 更新函数（每个采样周期调用）
 * 输入：
 *   v_alpha, v_beta - 线-中性或 Clarke 后的电压空间矢量分量
 *   i_alpha_meas, i_beta_meas - 测量的电流 alpha/beta
 * 返回：无；估计量保存在 h->state 中
 */
    void SMO_Update(SMO_Handle_t* h, Clark_t voltage, Clark_t current);

    /* 获取估计角度（rad） */
    smo_ft SMO_GetAngle(const SMO_Handle_t* h);

    /* 获取估计角速度（rad/s） */
    smo_ft SMO_GetSpeed(const SMO_Handle_t* h);

    /* 获取估计的电流或磁链（指针返回） */
    const SMO_State_t* SMO_GetState(const SMO_Handle_t* h);

    /* 小工具：符号函数（带边界带宽以减少抖动） */
    static inline smo_ft SMO_sign(smo_ft x, smo_ft eps)
    {
        if (x > eps)
            return 1.0f;
        if (x < -eps)
            return -1.0f;
        return x / (eps + 1e-12f);
    }

#ifdef __cplusplus
}
#endif

#endif /* SMO_H */