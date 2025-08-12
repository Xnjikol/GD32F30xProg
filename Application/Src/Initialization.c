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
#include "parameters.h"
#include "position_sensor.h"
#include "protect.h"
#include "systick.h"
#include "tim.h"
#include "usart.h"

bool Init_Foc_Parameters(void) {
    SystemTimeConfig_t sys_time_cfg
        = {.current = {.val = MAIN_LOOP_FREQ, .inv = MAIN_LOOP_TIME},
           .speed   = {.val = SPEED_LOOP_FREQ, .inv = SPEED_LOOP_TIME},
           .prescaler = SPEED_LOOP_PRESCALER};
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

bool Init_Protect_Parameters(void) {
    Protect_Parameter_t protect_param
        = {.Udc_rate        = PROTECT_VOLTAGE_RATE,
           .Udc_fluctuation = PROTECT_VOLTAGE_FLUCTUATION,
           .I_Max           = PROTECT_CURRENT_MAX,
           .Temperature     = PROTECT_TEMPERATURE,
           .Flag            = No_Protect};
    return Protect_Init(&protect_param);
}

bool Initialization_Variables(void) {
    Init_Foc_Parameters();
    Init_Protect_Parameters();
    return true;
}

/* open fan and relay */
void Init_Relay_Fan(void) {
    gpio_bit_set(SOFT_OPEN_PORT, SOFT_OPEN_PIN);
    // gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
}

bool Initialization_Execute(void) {
    systick_config();  // systick provides delay_ms
    TIM1_Init();       // TIM1 provides delay_us
    Main_Dwt_Init();
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
    CAN_Init();
    /* open fan and relay */
    Init_Relay_Fan();
    /* configure NVIC and enable interrupt */
    Main_Config_Exti();
    Main_Nvic_config();
    COM_ProtocolInit();
    return true;
}
