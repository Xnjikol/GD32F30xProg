#include "main.h"  // IWYU pragma: export
#include "Initialization.h"
#include "adc.h"
#include "can.h"
#include "com.h"
#include "gd32f30x_gpio.h"
#include "gpio.h"
#include "hardware_interface.h"
#include "position_sensor.h"
#include "systick.h"
#include "tim.h"
#include "usart.h"

volatile uint32_t DWT_Count = 0;

bool pin = false;

/*!
    \brief      main function
*/
int main(void) {
    Initialization_Execute();
    while (1) {
        COM_CANProtocol();
        COM_SCIProtocol();
        // COM_DAQProtocol(systick_ms); Use CCP DAQ may cause PiSnoop display
        // offline
        Peripheral_Update_Temperature();
        Peripheral_Update_Break();

        pin = gpio_input_bit_get(GPIOE, GPIO_PIN_15);
        // DWT_Count = DWT->CYCCNT; // 读取DWT计数器
    }
}

void Main_Dwt_Init(void) {
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
void Main_Nvic_config(void) {
    nvic_priority_group_set(
        NVIC_PRIGROUP_PRE2_SUB2);  // 设置中断优先级分组
    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 1U, 0U);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 2, 0);
    nvic_irq_enable(ADC0_1_IRQn, 3, 0);
    nvic_irq_enable(TIMER3_IRQn, 4, 0);

    nvic_irq_enable(DMA0_Channel3_IRQn, 5, 0);
    /* SysTick_IRQn 009U */

    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

void Main_Config_Exti(void) {
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_7);
}
