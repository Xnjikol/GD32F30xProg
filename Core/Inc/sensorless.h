#ifndef _SENSORLESS_H_
#define _SENSORLESS_H_

#include "filter.h"
#include "main.h"
#include "math_utils.h"

/* 前向声明 */
typedef struct FOC_t FOC_t;

/*======================*/
/*    Sensorless Config */
/*======================*/

/* 高频注入参数 */
#define HFI_INJECTION_FREQ 1000.0f /* 注入频率 (Hz) */
#define HFI_INJECTION_VOLTAGE 5.0f /* 注入电压 (V) */
#define HFI_FILTER_CUTOFF 200.0f   /* 滤波器截止频率 (Hz) */

/* 滑膜观测器参数 */
#define SMO_GAIN_K 200.0f        /* 滑膜增益 */
#define SMO_FILTER_CUTOFF 100.0f /* 反EMF滤波器截止频率 */
#define SMO_HYSTERESIS 0.1f      /* 滞环宽度 */

/* PLL锁相环参数 */
#define PLL_KP 500.0f        /* PLL比例增益 */
#define PLL_KI 50000.0f      /* PLL积分增益 */
#define PLL_MAX_FREQ 1000.0f /* PLL最大频率 (Hz) */

/* 速度切换阈值 */
#define SPEED_THRESHOLD_HFI_TO_SMO 200.0f /* 从HFI切换到SMO的速度阈值 (rpm) */
#define SPEED_THRESHOLD_SMO_TO_HFI 150.0f /* 从SMO切换到HFI的速度阈值 (rpm) */

/*======================*/
/*    Type Definitions  */
/*======================*/

/* 无位置传感器算法类型 */
typedef enum
{
    SENSORLESS_NONE,         /* 无算法 */
    SENSORLESS_HFI,          /* 高频注入 */
    SENSORLESS_SMO,          /* 滑膜观测器 */
    SENSORLESS_HYBRID,       /* 混合算法 */
    SENSORLESS_TWELVE_PULSE, /* 十二脉冲 */
    SENSORLESS_FLUX_OBSERVER /* 磁链观测器 */
} SensorlessAlgorithm_t;

/* 电机状态 */
typedef enum
{
    MOTOR_STOP,
    MOTOR_STARTUP,
    MOTOR_LOW_SPEED,
    MOTOR_HIGH_SPEED
} MotorState_t;

/* 高频注入结构 */
typedef struct
{
    float freq;      /* 注入频率 */
    float voltage;   /* 注入电压 */
    float theta;     /* 注入信号相位 */
    float cos_theta; /* cos(注入相位) */
    float sin_theta; /* sin(注入相位) */

    /* 注入信号 */
    float ud_inj; /* d轴注入电压 */
    float uq_inj; /* q轴注入电压 */

    /* 响应电流 */
    float id_response; /* d轴响应电流 */
    float iq_response; /* q轴响应电流 */

    /* 滤波器 */
    LowPassFilter_t lpf_d;  /* d轴低通滤波器 */
    LowPassFilter_t lpf_q;  /* q轴低通滤波器 */
    BandPassFilter_t bpf_d; /* d轴带通滤波器 */
    BandPassFilter_t bpf_q; /* q轴带通滤波器 */

    /* 解调信号 */
    float demod_d; /* d轴解调信号 */
    float demod_q; /* q轴解调信号 */

    /* 位置估计 */
    float theta_est; /* 估计的转子位置 */
    float error;     /* 位置误差 */

    /* 使能标志 */
    uint8_t enabled; /* 算法使能 */
} HFI_t;

/* 滑膜观测器结构 */
typedef struct
{
    float gain_k;     /* 滑膜增益 */
    float hysteresis; /* 滞环宽度 */

    /* 状态变量 */
    float ialpha_est; /* 估计的alpha轴电流 */
    float ibeta_est;  /* 估计的beta轴电流 */

    /* 反EMF */
    float ealpha;      /* alpha轴反EMF */
    float ebeta;       /* beta轴反EMF */
    float ealpha_filt; /* 滤波后的alpha轴反EMF */
    float ebeta_filt;  /* 滤波后的beta轴反EMF */

    /* 滤波器 */
    LowPassFilter_t lpf_ealpha; /* alpha轴反EMF低通滤波器 */
    LowPassFilter_t lpf_ebeta;  /* beta轴反EMF低通滤波器 */

    /* 位置和速度估计 */
    float theta_est; /* 估计的转子位置 */
    float speed_est; /* 估计的转子速度 */

    /* 使能标志 */
    uint8_t enabled; /* 算法使能 */
} SMO_t;

/* PLL锁相环结构 */
typedef struct
{
    float kp;       /* 比例增益 */
    float ki;       /* 积分增益 */
    float max_freq; /* 最大频率限制 */

    /* 状态变量 */
    float theta_est; /* 估计的转子位置 */
    float speed_est; /* 估计的转子速度 */
    float error;     /* 相位误差 */
    float integral;  /* 积分项 */

    /* 输入信号 */
    float sin_theta; /* sin(实际位置) */
    float cos_theta; /* cos(实际位置) */

    /* 使能标志 */
    uint8_t enabled; /* 算法使能 */
} PLL_t;

/* 十二脉冲状态 */
typedef enum
{
    TWELVE_PULSE_IDLE,
    TWELVE_PULSE_APPLYING,
    TWELVE_PULSE_MEASURING,
    TWELVE_PULSE_CALCULATING,
    TWELVE_PULSE_COMPLETED
} TwelvePulseState_t;

/* 十二脉冲结构 */
typedef struct
{
    float test_voltage;      /* 测试电压 */
    float test_time;         /* 脉冲持续时间 */
    float settle_time;       /* 稳定时间 */
    uint8_t current_pulse;   /* 当前脉冲索引 */
    float pulse_timer;       /* 脉冲计时器 */
    uint16_t measurement_samples; /* 测量样本数 */
    float theta_offset;      /* 初始位置偏移 */
    
    /* 状态控制 */
    TwelvePulseState_t state; /* 当前状态 */
    uint8_t completed;        /* 完成标志 */
    uint8_t enabled;          /* 使能标志 */
    float confidence;         /* 估计置信度 */

    /* 电流响应数据 */
    float id_response[12];      /* 12个脉冲的d轴电流响应 */
    float iq_response[12];      /* 12个脉冲的q轴电流响应 */
    float current_magnitude[12]; /* 电流幅值 */
    float response_quality[12];  /* 响应质量 */

    /* 位置估计 */
    float theta_est;         /* 估计的初始位置 */
    float theta_raw;         /* 原始位置估计 */
    float theta_filtered;    /* 滤波后的位置估计 */
} TwelvePulse_t;

/* 磁链观测器结构 */
typedef struct
{
    float Rs; /* 定子电阻 */
    float Ls; /* 定子电感 */

    /* 磁链估计 */
    float psi_alpha;      /* alpha轴磁链 */
    float psi_beta;       /* beta轴磁链 */
    float psi_alpha_filt; /* 滤波后的alpha轴磁链 */
    float psi_beta_filt;  /* 滤波后的beta轴磁链 */

    /* 滤波器 */
    HighPassFilter_t hpf_alpha; /* alpha轴高通滤波器 */
    HighPassFilter_t hpf_beta;  /* beta轴高通滤波器 */

    /* 位置和速度估计 */
    float theta_est; /* 估计的转子位置 */
    float speed_est; /* 估计的转子速度 */

    /* 使能标志 */
    uint8_t enabled; /* 算法使能 */
} FluxObserver_t;

/* 无位置传感器主结构 */
typedef struct
{
    SensorlessAlgorithm_t algorithm; /* 当前算法 */
    MotorState_t motor_state;        /* 电机状态 */

    /* 算法实例 */
    HFI_t hfi;                    /* 高频注入 */
    SMO_t smo;                    /* 滑膜观测器 */
    PLL_t pll;                    /* PLL锁相环 */
    TwelvePulse_t pulse;   /* 十二脉冲 */
    FluxObserver_t flux_observer; /* 磁链观测器 */

    /* 最终输出 */
    float theta_est; /* 最终估计位置 */
    float speed_est; /* 最终估计速度 */

    /* 切换逻辑 */
    float speed_threshold_up;   /* 向上切换阈值 */
    float speed_threshold_down; /* 向下切换阈值 */
    uint16_t switch_counter;    /* 切换计数器 */

    /* 使能标志 */
    uint8_t enabled; /* 整体使能 */
} Sensorless_t;

/*======================*/
/*    Global Variables  */
/*======================*/
extern Sensorless_t Sensorless;

/*======================*/
/*    Function Protos   */
/*======================*/

/* 主函数 */
void Sensorless_Init(void);
void Sensorless_Update(float ia, float ib, float ic, float ua, float ub, float uc);
void Sensorless_SetAlgorithm(SensorlessAlgorithm_t algorithm);
float Sensorless_GetTheta(void);
float Sensorless_GetSpeed(void);

/* 高频注入算法 */
void HFI_Init(HFI_t *hfi);
void HFI_Update(HFI_t *hfi, float id, float iq, float ud, float uq);
void HFI_GetInjectionVoltage(HFI_t *hfi, float *ud_inj, float *uq_inj);
float HFI_GetTheta(HFI_t *hfi);

/* 滑膜观测器算法 */
void SMO_Init(SMO_t *smo);
void SMO_Update(SMO_t *smo, float ialpha, float ibeta, float ualpha, float ubeta);
float SMO_GetTheta(SMO_t *smo);
float SMO_GetSpeed(SMO_t *smo);

/* PLL锁相环 */
void PLL_Init(PLL_t *pll);
void PLL_Update(PLL_t *pll, float sin_theta, float cos_theta);
float PLL_GetTheta(PLL_t *pll);
float PLL_GetSpeed(PLL_t *pll);

/* 十二脉冲算法 */
void TwelvePulse_Init(TwelvePulse_t *pulse);
void TwelvePulse_Start(TwelvePulse_t *pulse);
void TwelvePulse_Update(TwelvePulse_t *pulse, float id, float iq);
void TwelvePulse_CalculateInitialPosition(TwelvePulse_t *pulse);
float TwelvePulse_GetTheta(TwelvePulse_t *pulse);
float TwelvePulse_GetConfidence(TwelvePulse_t *pulse);
uint8_t TwelvePulse_IsCompleted(TwelvePulse_t *pulse);
void TwelvePulse_GetVoltageReference(TwelvePulse_t *pulse, float *ud_ref, float *uq_ref);

/* 磁链观测器 */
void FluxObserver_Init(FluxObserver_t *observer);
void FluxObserver_Update(FluxObserver_t *observer, float ialpha, float ibeta, float ualpha, float ubeta);
float FluxObserver_GetTheta(FluxObserver_t *observer);
float FluxObserver_GetSpeed(FluxObserver_t *observer);

/* 辅助函数 */
void Sensorless_StateTransition(MotorState_t new_state);

#endif /* _SENSORLESS_H_ */
