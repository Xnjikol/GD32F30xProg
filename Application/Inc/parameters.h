/*********************************************************************/
/*                        初始化参数头文件                              */
/*                                                                   */
/* 文件名: parameters.h                                               */
/* 描述: 包含电机控制所需的所有初始化参数和常量定义                          */
/*********************************************************************/

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

/*********************************************************************/
/*                        门极驱动极性定义                              */
/*********************************************************************/
#ifndef GATE_POLARITY_HIGH_ACTIVE
#    ifndef GATE_POLARITY_LOW_ACTIVE
#        define GATE_POLARITY_LOW_ACTIVE
#    endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) \
    && !defined(GATE_POLARITY_LOW_ACTIVE)
#    error \
        "请在foc.h中定义GATE_POLARITY_HIGH_ACTIVE或GATE_POLARITY_LOW_ACTIVE"
#endif

/*********************************************************************/
/*                        位置传感器类型定义                            */
/*********************************************************************/
#ifndef RESOLVER_POSITION
#    ifndef ENCODER_POSITION
#        define RESOLVER_POSITION
#    endif
#endif

#if !defined(RESOLVER_POSITION) && !defined(ENCODER_POSITION)
#    error "请在foc.h中定义RESOLVER_POSITION或ENCODER_POSITION"
#endif

/*********************************************************************/
/*                        时钟参数配置                                 */
/*********************************************************************/
/* 主频配置 */
#define MCU_MAIN_FREQ 120000000U /* MCU主频：120MHz */

/* 主定时器配置 */
#define MAIN_INT_TIMER_PRESCALER 0 /* 预分频器：不分频 */
#define MAIN_INT_TIMER_PERIOD \
    6000 /* 周期值：6000 (10kHz = 120MHz/6000/2) */
#define MAIN_INT_TIMER_DEADTIME_PERIOD \
    2000 /* 死区时间：2us (2us = 120MHz/6000/2*2000) */

/* 主循环(电流环)频率配置 */
#define MAIN_LOOP_FREQ                              \
    (MCU_MAIN_FREQ / (MAIN_INT_TIMER_PRESCALER + 1) \
     / MAIN_INT_TIMER_PERIOD / 2)              /* 10kHz */
#define MAIN_LOOP_TIME (1.0F / MAIN_LOOP_FREQ) /* 100us */

/* 转速环频率配置 */
#define SPEED_LOOP_PRESCALER 10.0F /* 转速环分频系数 */
#define SPEED_LOOP_FREQ \
    (MAIN_LOOP_FREQ / SPEED_LOOP_PRESCALER)      /* 1kHz */
#define SPEED_LOOP_TIME (1.0F / SPEED_LOOP_FREQ) /* 1ms */

/*********************************************************************/
/*                        电机物理参数                                 */
/*********************************************************************/
/* 电机电气参数 */
#define MOTOR_RS   1.25F  /* 定子电阻：1.25 Ω */
#define MOTOR_LD   0.006F /* d轴电感：6 mH */
#define MOTOR_LQ   0.009F /* q轴电感：9 mH */
#define MOTOR_FLUX 0.1F   /* 永磁体磁链：0.1 Wb */
#define MOTOR_PN   5.0F   /* 电机极对数：5 */

/* 位置传感器配置 */
#ifdef RESOLVER_POSITION
#    define MOTOR_POSITION_SCALE (65536U - 1U) /* 旋变分辨率：16位 */
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_POSITION_SCALE \
        (10000U - 1U) /* 编码器分辨率：10000线 */
#endif

/* 角度计算因子 */
#ifdef RESOLVER_POSITION
#    define MOTOR_THETA_FACTOR \
        (M_2PI / ((MOTOR_POSITION_SCALE + 1) * MOTOR_RESOLVER_PN))
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_THETA_FACTOR \
        (M_2PI / (float)(MOTOR_POSITION_SCALE + 1))
#endif

#define MOTOR_RESOLVER_PN     1.0F          /* 旋变极对数 */
#define MOTOR_POSITION_OFFSET 38775.000000F /* 位置传感器零点偏置 */

/*********************************************************************/
/*                        保护参数配置                                 */
/*********************************************************************/
/* 电压保护参数 */
#define PROTECT_VOLTAGE_RATE        220.0F /* 额定电压：220V */
#define PROTECT_VOLTAGE_FLUCTUATION 40.0F  /* 允许电压波动：±40V */

/* 电流和温度保护参数 */
#define PROTECT_CURRENT_MAX 10.0F /* 最大电流限制：10A */
#define PROTECT_TEMPERATURE 80.0F /* 最高温度限制：80℃ */

/*********************************************************************/
/*                        FOC控制参数配置                              */
/*********************************************************************/
/* 转速斜坡控制参数 */
#define RAMP_SPEED_SLOPE     50.0F    /* 速度变化率限制：50 rpm/s */
#define RAMP_SPEED_LIMIT_MAX 4000.0F  /* 最大转速限制：4000 rpm */
#define RAMP_SPEED_LIMIT_MIN -4000.0F /* 最小转速限制：-4000 rpm */
#define RAMP_SPEED_TIME      (SPEED_LOOP_TIME) /* 转速环采样周期 */

/* 转速环PID参数配置 */
#define PID_SPEED_LOOP_KP 0.005F /* 转速环比例系数 */
#define PID_SPEED_LOOP_KI 0.1F   /* 转速环积分系数 */
#define PID_SPEED_LOOP_KD 0.0F   /* 转速环微分系数 */

/* 转速环输出限制 */
#define PID_SPEED_LOOP_MAX_OUTPUT \
    (0.7F * PROTECT_CURRENT_MAX) /* q轴最大电流限制 */
#define PID_SPEED_LOOP_MIN_OUTPUT (-1.0F * PID_SPEED_LOOP_MAX_OUTPUT)
#define PID_SPEED_LOOP_INTEGRAL_LIMIT \
    PID_SPEED_LOOP_MAX_OUTPUT                 /* 积分限幅值 */
#define PID_SPEED_LOOP_TIME (SPEED_LOOP_TIME) /* 转速环采样周期 */

/* 电流环d轴PID参数配置 */
#define PID_CURRENT_D_LOOP_KP 7.5398223686F   /* d轴比例系数 */
#define PID_CURRENT_D_LOOP_KI 1570.796326794F /* d轴积分系数 */
#define PID_CURRENT_D_LOOP_KD 0.0F            /* d轴微分系数 */

/* d轴输出限制 */
#define PID_CURRENT_D_LOOP_MAX_OUTPUT 50.0F /* 最大输出电压：Udc/√3 */
#define PID_CURRENT_D_LOOP_MIN_OUTPUT \
    (-1.0F * PID_CURRENT_D_LOOP_MAX_OUTPUT)
#define PID_CURRENT_D_LOOP_INTEGRAL_LIMIT \
    PID_CURRENT_D_LOOP_MAX_OUTPUT                /* 积分限幅值 */
#define PID_CURRENT_D_LOOP_TIME (MAIN_LOOP_TIME) /* d轴采样周期 */

/* 电流环q轴PID参数配置 */
#define PID_CURRENT_Q_LOOP_KP 11.30973355F    /* q轴比例系数 */
#define PID_CURRENT_Q_LOOP_KI 1570.796326794F /* q轴积分系数 */
#define PID_CURRENT_Q_LOOP_KD 0.0F            /* q轴微分系数 */

/* q轴输出限制 */
#define PID_CURRENT_Q_LOOP_MAX_OUTPUT 50.0F /* 最大输出电压：Udc/√3 */
#define PID_CURRENT_Q_LOOP_MIN_OUTPUT \
    (-1.0F * PID_CURRENT_Q_LOOP_MAX_OUTPUT)
#define PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT \
    PID_CURRENT_Q_LOOP_MAX_OUTPUT                /* 积分限幅值 */
#define PID_CURRENT_Q_LOOP_TIME (MAIN_LOOP_TIME) /* q轴采样周期 */

/*********************************************************************/
/*                        无位置运行参数配置                            */
/*********************************************************************/
/* 无位置传感器策略控制 */
#define SENSORLESS_HYSTERESIS   50.0F  /* 无传感器滞环宽度：50rpm */
#define SENSORLESS_SWITCH_SPEED 200.0F /* 无传感器切换速度：200rpm */

/* 高频信号注入参数 */
#define HF_INJECTION_FREQ 500.0F /* 注入信号频率：500Hz */
#define HF_INJECTION_AMP  40.0F  /* 注入信号幅值：40V */

/* 高频信号注入带通滤波器参数 */
#define HFI_RESPONSE_FREQ      HF_INJECTION_FREQ /* 中心频率 */
#define HFI_RESPONSE_BANDWIDTH 40.0F          /* 滤波器频带宽度：40Hz */
#define HFI_SAMPLE_TIME        MAIN_LOOP_TIME /* 采样周期：与主循环相同 */
#define HFI_SAMPLE_FREQ        MAIN_LOOP_FREQ /* 采样周期：与主循环相同 */

/* 高频信号注入法低通滤波器参数 */
#define HFI_LOW_PASS_CUTOFF_FREQ 100.0F /* 误差信号截止频率：100Hz */

/* 高频注入PLL跟踪器参数 */
#define HFI_PLL_KP             1200.0F /* PLL比例系数 */
#define HFI_PLL_KI             18E3F   /* PLL积分系数 */
#define HFI_PLL_KD             0.0F    /* PLL微分系数 */
#define HFI_PLL_MAX_OUTPUT     4000.0F /* PLL最大输出 */
#define HFI_PLL_MIN_OUTPUT     (-1 * HFI_PLL_MAX_OUTPUT) /* PLL最小输出 */
#define HFI_PLL_INTEGRAL_LIMIT HFI_PLL_MAX_OUTPUT /* PLL积分限幅值 */

/* 滑模观测器参数 */
#define SMO_SIGMOID_AMPLITUDE 0.1F   /* S函数幅值 */
#define SMO_SIGMOID_FACTOR    0.1F   /* S函数因子 */
#define SMO_GAIN_K1           200.0F /* 观测器增益K1 */
#define SMO_GAIN_K2           0.01F  /* 观测器增益K2 */

/* 滑模观测器PLL跟踪器参数 */
#define SMO_PLL_KP             500.0F  /* PLL比例系数 */
#define SMO_PLL_KI             4E4F    /* PLL积分系数 */
#define SMO_PLL_KD             0.0F    /* PLL微分系数 */
#define SMO_PLL_MAX_OUTPUT     4000.0F /* PLL最大输出 */
#define SMO_PLL_MIN_OUTPUT     (-1 * SMO_PLL_MAX_OUTPUT) /* PLL最小输出 */
#define SMO_PLL_INTEGRAL_LIMIT SMO_PLL_MAX_OUTPUT /* PLL积分限幅值 */

/* 滑模观测器低通滤波器参数 */
#define SMO_LPF_CUTOFF_FREQ 500.0F /* 低通滤波器截止频率：500Hz */
#define SMO_LPF_ORDER       2      /* 低通滤波器阶数：2 */
#define SMO_SAMPLING_FREQ \
    MAIN_LOOP_FREQ /* 采样频率：与主循环频率相同 */

#endif