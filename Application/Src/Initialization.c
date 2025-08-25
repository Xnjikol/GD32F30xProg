#include "Initialization.h"
#include "adc.h"
#include "can.h"
#include "com.h"
#include "foc.h"
#include "foc_types.h"
#include "gd32f30x_gpio.h"
#include "gpio.h"
#include "hardware_interface.h"
#include "main.h"
#include "motor.h"
#include "parameters.h"
#include "position_sensor.h"
#include "protect.h"
#include "systick.h"
#include "tim.h"
#include "usart.h"

volatile uint32_t  DWT_Count = 0;
SystemTimeConfig_t sys_time_cfg
    = {.current   = {.val = MAIN_LOOP_TIME, .inv = MAIN_LOOP_FREQ},
       .speed     = {.val = SPEED_LOOP_TIME, .inv = SPEED_LOOP_FREQ},
       .prescaler = SPEED_LOOP_PRESCALER};

bool Init_Foc_Parameters(void) {
    Foc_Set_SampleTime(&sys_time_cfg);

    Foc_Set_ResetFlag(true);  // 初始为复位状态

    PID_Handler_t pid_cfg = {0};

    pid_cfg = (PID_Handler_t){
        .Kp            = PID_SPEED_LOOP_KP,
        .Ki            = PID_SPEED_LOOP_KI,
        .Kd            = PID_SPEED_LOOP_KD,
        .MaxOutput     = PID_SPEED_LOOP_MAX_OUTPUT,
        .MinOutput     = PID_SPEED_LOOP_MIN_OUTPUT,
        .IntegralLimit = PID_SPEED_LOOP_INTEGRAL_LIMIT,
        .Ts            = PID_SPEED_LOOP_TIME,
        .Reset         = true,
        .output        = 0.0F};
    Foc_Set_Pid_Speed_Handler(&pid_cfg);

    pid_cfg = (PID_Handler_t){
        .Kp            = PID_CURRENT_D_LOOP_KP,
        .Ki            = PID_CURRENT_D_LOOP_KI,
        .Kd            = PID_CURRENT_D_LOOP_KD,
        .MaxOutput     = PID_CURRENT_D_LOOP_MAX_OUTPUT,
        .MinOutput     = PID_CURRENT_D_LOOP_MIN_OUTPUT,
        .IntegralLimit = PID_CURRENT_D_LOOP_INTEGRAL_LIMIT,
        .Ts            = PID_CURRENT_D_LOOP_TIME,
        .Reset         = true,
        .output        = 0.0F};
    Foc_Set_Pid_CurD_Handler(&pid_cfg);

    pid_cfg = (PID_Handler_t){
        .Kp            = PID_CURRENT_Q_LOOP_KP,
        .Ki            = PID_CURRENT_Q_LOOP_KI,
        .Kd            = PID_CURRENT_Q_LOOP_KD,
        .MaxOutput     = PID_CURRENT_Q_LOOP_MAX_OUTPUT,
        .MinOutput     = PID_CURRENT_Q_LOOP_MIN_OUTPUT,
        .IntegralLimit = PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT,
        .Ts            = PID_CURRENT_Q_LOOP_TIME,
        .Reset         = true,
        .output        = 0.0F};
    Foc_Set_Pid_CurQ_Handler(&pid_cfg);

    RampGenerator_t ramp_cfg = {.slope     = RAMP_SPEED_SLOPE,
                                .limit_min = RAMP_SPEED_LIMIT_MIN,
                                .limit_max = RAMP_SPEED_LIMIT_MAX,
                                .Ts        = RAMP_SPEED_TIME,
                                .target    = 0.0F,
                                .value     = 0.0F};
    Foc_Set_Ramp_Speed_Handler(&ramp_cfg);

    return true;
}

bool Init_Motor_Parameters(void) {
    Motor_Parameter_t motor_param
        = {.Rs              = MOTOR_RS,
           .Ld              = MOTOR_LD,
           .Lq              = MOTOR_LQ,
           .Flux            = MOTOR_FLUX,
           .Pn              = MOTOR_PN,
           .inv_MotorPn     = 1.0F / MOTOR_PN,
           .Resolver_Pn     = MOTOR_RESOLVER_PN,
           .Position_Offset = MOTOR_POSITION_OFFSET,
           .Position_Scale  = MOTOR_POSITION_SCALE,
           .theta_factor    = MOTOR_THETA_FACTOR};
    Motor_Initialization(&motor_param);
    Motor_Set_SampleTime(&sys_time_cfg);
    Motor_Set_SpeedPrescaler(SPEED_LOOP_PRESCALER);
    Motor_Set_Filter(10.0F, SPEED_LOOP_FREQ);
}

bool Init_Protect_Parameters(void) {
    Protect_Parameter_t protect_param
        = {.Udc_rate        = PROTECT_VOLTAGE_RATE,
           .Udc_fluctuation = PROTECT_VOLTAGE_FLUCTUATION,
           .I_Max           = PROTECT_CURRENT_MAX,
           .Temperature     = PROTECT_TEMPERATURE,
           .Flag            = No_Protect};
    return Protect_Initialization(&protect_param);
}

bool Initialization_Variables(void) {
    Init_Foc_Parameters();
    Init_Protect_Parameters();
    return true;
}

/* open fan and relay */
static inline void Init_Relay_Fan(void) {
    gpio_bit_set(SOFT_OPEN_PORT, SOFT_OPEN_PIN);
    // gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
}

static inline void Init_Dwt(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能DWT模块
    DWT->CYCCNT = 0;                                 // 清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // 启用CYCCNT
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
static inline void Init_Nvic(void) {
    // 设置中断优先级分组
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 1U, 0U);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 2, 0);
    nvic_irq_enable(ADC0_1_IRQn, 3, 0);
    nvic_irq_enable(TIMER3_IRQn, 4, 0);

    nvic_irq_enable(DMA0_Channel3_IRQn, 5, 0);
    /* SysTick_IRQn 009U */

    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

static inline void Init_Exti(void) {
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_7);
}

bool Initialization_System(void) {
    systick_config();  // systick provides delay_ms
    TIM1_Init();       // TIM1 provides delay_us
    Init_Dwt();
    /* initialize Serial port */
    //< For USART DMA, USART must be initialized before DMA >//
    USART_Init(&husart0);
    USART_DMA_Init();
    /* initialize GPIO */
    GPIO_Init();
    /* initialize Position_Sensor */
    Position_Sensor_Init();
    /* initialize Timer */
    TIM0_PWM_Init(MAIN_INT_TIMER_PRESCALER,
                  MAIN_INT_TIMER_PERIOD,
                  MAIN_INT_TIMER_DEADTIME_PERIOD);
    /* initialize external interrupt */
    /* initialize ADC */
    Adc_Initialization();
    /* initialize CAN and CCP */
    Can_Initialization();
    /* open fan and relay */
    Init_Relay_Fan();
    /* configure NVIC and enable interrupt */
    Init_Exti();
    Init_Nvic();
    Com_Initialization();
    return true;
}
