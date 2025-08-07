#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <stdbool.h>

/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#    ifndef GATE_POLARITY_LOW_ACTIVE
#        define GATE_POLARITY_LOW_ACTIVE /* Define here */
#    endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) && !defined(GATE_POLARITY_LOW_ACTIVE)
#    error \
        "Please define GATE_POLARITY_HIGH_ACTIVE or GATE_POLARITY_LOW_ACTIVE in foc.h"
#endif

#ifndef RESOLVER_POSITION
#    ifndef ENCODER_POSITION
#        define RESOLVER_POSITION /* Define here */
#    endif
#endif

/*********************************************************************/
/*                        时钟 参数区                                  */
/*********************************************************************/
#define MCU_MAIN_FREQ 120000000U /* 120MHz */
#define MAIN_INT_TIMER_PRESCALER 0
/* 10kHz, 120MHz / 6000 / 2 */
#define MAIN_INT_TIMER_PERIOD 6000
/* 2us, 120MHz / 6000 / 2 * 2000 = 2us */
#define MAIN_INT_TIMER_DEADTIME_PERIOD 2000
/* 电流环频率和时间 */
#define MAIN_LOOP_FREQ \
    (MCU_MAIN_FREQ / (MAIN_INT_TIMER_PRESCALER + 1) / MAIN_INT_TIMER_PERIOD / 2)
#define MAIN_LOOP_TIME (1.0F / MAIN_LOOP_FREQ) /* 10kHz */
/* 转速环频率和时间 */
#define SPEED_LOOP_PRESCALER 10.0F
#define SPEED_LOOP_FREQ (MAIN_LOOP_FREQ / SPEED_LOOP_PRESCALER) /* 1kHz */
#define SPEED_LOOP_TIME (1.0F / SPEED_LOOP_FREQ)                /* 1e-3s */

/*********************************************************************/
/*                        电机 参数                                  */
/*********************************************************************/
#define MOTOR_RS 1.25F
#define MOTOR_LD 0.006F
#define MOTOR_LQ 0.009F
#define MOTOR_FLUX 0.1F
#define MOTOR_PN 5.0F
#ifdef RESOLVER_POSITION
#    define MOTOR_POSITION_SCALE (65536U - 1U)
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_POSITION_SCALE (10000U - 1U)
#endif
#ifdef RESOLVER_POSITION
#    define MOTOR_THETA_FACTOR \
        (M_2PI / ((MOTOR_POSITION_SCALE + 1) * MOTOR_RESOLVER_PN))
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_THETA_FACTOR (M_2PI / (float) (MOTOR_POSITION_SCALE + 1))
#endif
#define MOTOR_RESOLVER_PN 1.0F
#define MOTOR_POSITION_OFFSET 39833.000000F

/*********************************************************************/
/*                        保护 参数                                  */
/*********************************************************************/
#define PROTECT_VOLTAGE_RATE 220.0F       /* 220V */
#define PROTECT_VOLTAGE_FLUCTUATION 40.0F /* 40V */
#define PROTECT_CURRENT_MAX 10.0F         /* 10A */
#define PROTECT_TEMPERATURE 80.0F         /* 80℃ */

/*********************************************************************/
/*                        FOC 参数                                  */
/*********************************************************************/
/*  转速斜坡 控制参数 */
#define RAMP_SPEED_SLOPE 50.0F  // limit to 50 rpm/s
#define RAMP_SPEED_LIMIT_MIN -1800.0F
#define RAMP_SPEED_LIMIT_MAX 1800.0F

/* PID 控制参数 转速环 */
#define PID_SPEED_LOOP_KP 0.005F
#define PID_SPEED_LOOP_KI 0.03F
#define PID_SPEED_LOOP_KD 0.0F
#define PID_SPEED_LOOP_MAX_OUTPUT (0.7F * PROTECT_CURRENT_MAX)  // Maximum Iq
#define PID_SPEED_LOOP_MIN_OUTPUT -1 * PID_SPEED_LOOP_MAX_OUTPUT
#define PID_SPEED_LOOP_INTEGRAL_LIMIT PID_SPEED_LOOP_MAX_OUTPUT
#define PID_SPEED_LOOP_TIME (SPEED_LOOP_TIME)  // Speed loop time

/* PID 控制参数 电流环 d轴 */
#define PID_CURRENT_D_LOOP_KP 73.8274273F
#define PID_CURRENT_D_LOOP_KI 408.40704496F
#define PID_CURRENT_D_LOOP_KD 0.0F
#define PID_CURRENT_D_LOOP_MAX_OUTPUT 50.0F  // Maximum Udc/sqrt(3)
#define PID_CURRENT_D_LOOP_MIN_OUTPUT -1 * PID_CURRENT_D_LOOP_MAX_OUTPUT
#define PID_CURRENT_D_LOOP_INTEGRAL_LIMIT PID_CURRENT_D_LOOP_MAX_OUTPUT
#define PID_CURRENT_D_LOOP_TIME (CURRENT_LOOP_TIME)  // Current loop time

/* PID 控制参数 电流环 q轴 */
#define PID_CURRENT_Q_LOOP_KP 27.646015F
#define PID_CURRENT_Q_LOOP_KI 408.40704496F
#define PID_CURRENT_Q_LOOP_KD 0.0F
#define PID_CURRENT_Q_LOOP_MAX_OUTPUT 50.0F  // Maximum Udc/sqrt(3)
#define PID_CURRENT_Q_LOOP_MIN_OUTPUT -1 * PID_CURRENT_Q_LOOP_MAX_OUTPUT
#define PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT PID_CURRENT_Q_LOOP_MAX_OUTPUT
#define PID_CURRENT_Q_LOOP_TIME (CURRENT_LOOP_TIME)  // Current loop time

/*********************************************************************/
/*                        无位置 参数                                  */
/*********************************************************************/
/* 高频注入参数 */
#define HF_INJECTION_FREQ 1000.0F /* 高频注入频率 */
#define HF_INJECTION_AMP 15.0F    /* 高频注入幅值 */
#define HF_PLL_KP 0.1F
#define HF_PLL_KI 0.01F
#define HF_PLL_KD 0.0F

/* 滑模控制参数 */
#define SMO_SIGMOID_AMPLITUDE 0.1F /* 滑模控制幅值 */
#define SMO_SIGMOID_FACTOR 0.1F    /* 滑模控制因子 */
#define SMO_GAIN_K1 0.1F
#define SMO_GAIN_K2 0.01F

/*********************************************************************/
/*                        函数声明                                  */
/*********************************************************************/
/* 初始化函数 */
bool Initialization_Variables(void);

#endif  // INITIALIZATION_H
