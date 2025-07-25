/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"

#include "can.h"
#include "foc.h"
#include "gd32f30x.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "main_int.h"
#include "systick.h"

extern volatile uint16_t Device_Stop;

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void) {}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
  /* if Hard Fault exception occurs, go to infinite loop */
  while (1)
  {
  }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
  /* if Memory Manage exception occurs, go to infinite loop */
  while (1)
  {
  }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
  /* if Bus Fault exception occurs, go to infinite loop */
  while (1)
  {
  }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
  /* if Usage Fault exception occurs, go to infinite loop */
  while (1)
  {
  }
}


/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void) {}


/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
  delay_decrement();
  systick_ms++;
}

void DMA0_Channel3_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
  {
    dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
    Peripheral_SCISendCallback();
  }
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
  can_receive_message_struct rx_msg;
  can_message_receive(CAN0, CAN_FIFO0, &rx_msg);
  CAN_Buffer_Put(&rx_msg);
}

void ADC0_1_IRQHandler(void)
{
  Main_Int_Handler();
}

void EXTI5_9_IRQHandler(void)
{
  if (RESET != exti_interrupt_flag_get(EXTI_7))
  {
    TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    Device_Stop = 1;
    exti_interrupt_flag_clear(EXTI_7);
  }
}

extern Protect_Flags_t Protect_Flag;
extern bool Software_BRK;

void TIMER0_BRK_IRQHandler(void)
{
  if (timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_BRK))
  {
    // 清除 Break 中断标志
    timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_BRK);
    Device_Stop = 1;
    if (Software_BRK == false)
    {
      Protect.Flag |= Hardware_Fault;
      timer_interrupt_disable(TIMER0, TIMER_INT_BRK);  // 禁用BRK中断
      timer_primary_output_config(TIMER0, DISABLE);
    }
  }
}

/* */
void TIMER3_IRQHandler(void)
{
  if (timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_CH2))
  {
    // 清除 CH2 中断标志
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_CH2);

    TIMER_CNT(TIMER3) = 0;
  }
}
