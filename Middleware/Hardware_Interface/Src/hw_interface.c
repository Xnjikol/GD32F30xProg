#include "hw_interface.h"

#include <stddef.h>
#include <string.h>

#include "adc.h"
#include "can.h"
#include "gd32f30x_can.h"
#include "gpio.h"
#include "position_sensor.h"
#include "tim.h"
#include "usart.h"
#include "theta_calc.h"
#include "filter.h"
#include "foc.h"

/* ================ 静态变量 ================ */
static FOC_Parameter_t* s_foc_param = NULL;
static Motor_Parameter_t* s_motor_param = NULL;
static DeviceState_t* s_device_state = NULL;

static HW_ProtectParams_t s_protect_params = {0};
static volatile bool s_device_stop = true;
static bool s_software_brake = false;

/* ================ 静态函数声明 ================ */
static inline void hw_update_current(void);
static inline void hw_update_voltage(void);
static inline void hw_update_position(void);
static inline void hw_update_gate_state(void);
static inline void hw_current_protect(float Ia, float Ib, float Ic, float I_Max);
static inline void hw_voltage_protect(float Udc, float Udc_rate, float Udc_fluctuation);
static inline void hw_temperature_protect(void);
static inline bool hw_can_receive_to_frame(const can_receive_message_struct* hw_msg, can_frame_t* frame);

/* ================ 公共接口函数实现 ================ */

bool HW_Interface_Init(FOC_Parameter_t* foc_param, 
                      Motor_Parameter_t* motor_param,
                      DeviceState_t* device_state)
{
  if (!foc_param || !motor_param || !device_state)
    return false;

  /* 保存参数指针 */
  s_foc_param = foc_param;
  s_motor_param = motor_param;
  s_device_state = device_state;

  /* 初始化保护参数 */
  s_protect_params.voltage_rate = VOLTAGE_RATE_DEFAULT;
  s_protect_params.voltage_fluctuation = VOLTAGE_FLUCTUATION_DEFAULT;
  s_protect_params.current_max = CURRENT_THRESHOLD_DEFAULT;
  s_protect_params.temperature_max = TEMPERATURE_THRESHOLD_DEFAULT;
  s_protect_params.flags = HW_PROTECT_NONE;

  /* 初始化状态 */
  s_device_stop = true;
  s_software_brake = false;

  return true;
}

void HW_Interface_Process(void)
{
  /* 按顺序更新所有外设数据 */
  hw_update_current();
  hw_update_voltage();
  hw_update_position();
  hw_update_gate_state();
  
  /* 执行保护检查 */
  if (s_foc_param && s_foc_param->Iabc_fdbk)
  {
    hw_current_protect(s_foc_param->Iabc_fdbk->a, 
                      s_foc_param->Iabc_fdbk->b, 
                      s_foc_param->Iabc_fdbk->c, 
                      s_foc_param->I_Max);
  }
  
  hw_voltage_protect(s_foc_param ? s_foc_param->Udc : 0.0f, 
                    s_protect_params.voltage_rate,
                    s_protect_params.voltage_fluctuation);
                    
  hw_temperature_protect();
}

bool HW_Interface_GetPWMOutput(Phase_t* tcm_phase)
{
  if (!tcm_phase || !s_foc_param || !s_foc_param->Tcm)
    return false;

  tcm_phase->a = s_foc_param->Tcm->a;
  tcm_phase->b = s_foc_param->Tcm->b;
  tcm_phase->c = s_foc_param->Tcm->c;
  
  return true;
}

/* ================ 状态管理接口实现 ================ */

void HW_Interface_SetDeviceStop(bool stop)
{
  s_device_stop = stop;
  if (s_foc_param)
  {
    s_foc_param->Stop = stop;
  }
}

bool HW_Interface_GetDeviceStop(void)
{
  return s_device_stop;
}

void HW_Interface_EnableHardwareProtect(void)
{
  timer_interrupt_enable(TIMER0, TIMER_INT_BRK);
}

void HW_Interface_DisableHardwareProtect(void)
{
  timer_interrupt_disable(TIMER0, TIMER_INT_BRK);
}

HW_ProtectFlags_t HW_Interface_GetProtectFlags(void)
{
  return s_protect_params.flags;
}

void HW_Interface_ClearProtectFlags(HW_ProtectFlags_t flags_to_clear)
{
  s_protect_params.flags &= ~flags_to_clear;
}

/* ================ 通信接口实现 ================ */

bool HW_Interface_CANSend(const can_frame_t* frame)
{
  if (!frame)
    return false;

  can_transmit_message_struct msg = {0};
  
  msg.tx_sfid = frame->id;
  msg.tx_dlen = frame->dlc;
  memcpy(msg.tx_data, frame->data, frame->dlc);
  msg.tx_ft = CAN_FT_DATA;
  msg.tx_ff = CAN_FF_STANDARD;

  return (can_message_transmit(CAN0, &msg) != CAN_NOMAILBOX);
}

bool HW_Interface_CANReceive(can_frame_t* frame)
{
  if (!frame)
    return false;

  can_receive_message_struct hw_msg = {0};
  
  if (can_receive_message_length_get(CAN0, CAN_FIFO0) == 0)
    return false;

  can_message_receive(CAN0, CAN_FIFO0, &hw_msg);

  return hw_can_receive_to_frame(&hw_msg, frame);
}

void HW_Interface_SCISend(float* data, uint8_t count)
{
  if (!data || count == 0)
    return;

  /* 这里可以添加SCI发送的具体实现 */
  /* 例如使用DMA或中断方式发送浮点数据 */
}

/* ================ 校准和配置接口实现 ================ */

void HW_Interface_CalibrateADC(void)
{
  ADC_Calibration();
}

void HW_Interface_GetSystemFrequency(void)
{
  float f = 0.0F;
  float Ts = 0.0F;
  float PWM_ARR = 0.0F;

  cal_fmain(&f, &Ts, &PWM_ARR);
  
  if (s_device_state)
  {
    s_device_state->main_Freq = f;
    s_device_state->main_Ts = Ts;
    s_device_state->speed_Freq = f / SPEED_LOOP_PRESCALER;
    s_device_state->speed_Ts = Ts * SPEED_LOOP_PRESCALER;
  }
  
  if (s_foc_param)
  {
    s_foc_param->freq = f;
    s_foc_param->Ts = Ts;
    s_foc_param->PWM_ARR = PWM_ARR;
  }
}

void HW_Interface_InitProtectParams(void)
{
  /* 保护参数在初始化时已设置，这里可以重新初始化 */
  s_protect_params.voltage_rate = VOLTAGE_RATE_DEFAULT;
  s_protect_params.voltage_fluctuation = VOLTAGE_FLUCTUATION_DEFAULT;
  s_protect_params.current_max = CURRENT_THRESHOLD_DEFAULT;
  s_protect_params.temperature_max = TEMPERATURE_THRESHOLD_DEFAULT;
  s_protect_params.flags = HW_PROTECT_NONE;
}

/* ================ 静态函数实现 ================ */

static inline void hw_update_current(void)
{
  if (!s_foc_param || !s_foc_param->Iabc_fdbk)
    return;

  float ia, ib, ic;
  ADC_Read_Injection(&ia, &ib, &ic);
  
  /* 更新电流反馈 */
  s_foc_param->Iabc_fdbk->a = ia;
  s_foc_param->Iabc_fdbk->b = ib; 
  s_foc_param->Iabc_fdbk->c = ic;
}

static inline void hw_update_voltage(void)
{
  if (!s_foc_param)
    return;

  float udc, inv_udc;
  ADC_Read_Regular(&udc, &inv_udc);
  
  /* 更新电压反馈 */
  s_foc_param->Udc = udc;
  s_foc_param->inv_Udc = inv_udc;
}

static inline void hw_update_position(void)
{
  if (!s_motor_param || !s_foc_param)
    return;

  // TODO: 实现位置传感器读取和角度更新
  // ReadPositionSensor(&position_data);
  // FOC_UpdateThetaAndSpeed(s_foc_param, s_motor_param);
}

static inline void hw_update_gate_state(void)
{
  // TODO: 实现门极状态更新
  // Gate_state();
}

static inline void hw_current_protect(float Ia, float Ib, float Ic, float I_Max)
{
  if ((Ia > 0.9f * I_Max || Ia < -0.9f * I_Max) || 
      (Ib > 0.9f * I_Max || Ib < -0.9f * I_Max) ||
      (Ic > 0.9f * I_Max || Ic < -0.9f * I_Max))
  {
    static uint16_t current_count = 0;
    current_count++;
    if (current_count > 10)
    {
      s_device_stop = true;
      s_protect_params.flags |= HW_PROTECT_OVER_CURRENT;
      current_count = 0;
    }
  }

  float I_rms = sqrtf((Ia * Ia + Ib * Ib + Ic * Ic) / 3.0f);
  if (I_rms > I_Max)
  {
    s_device_stop = true;
    s_protect_params.flags |= HW_PROTECT_OVER_MAXIMUM_CURRENT;
  }
}

static inline void hw_voltage_protect(float Udc, float Udc_rate, float Udc_fluctuation)
{
  if (Udc > (Udc_rate + Udc_fluctuation))
  {
    s_device_stop = true;
    s_protect_params.flags |= HW_PROTECT_OVER_VOLTAGE;
  }
  else if (Udc < (Udc_rate - Udc_fluctuation))
  {
    s_device_stop = true;
    s_protect_params.flags |= HW_PROTECT_LOW_VOLTAGE;
  }
}

static inline void hw_temperature_protect(void)
{
  extern float Temperature; /* 来自 adc.h 的全局变量 */
  
  if (Temperature > 0.35f * s_protect_params.temperature_max)
  {
    gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
  }
  
  if (Temperature > s_protect_params.temperature_max)
  {
    s_device_stop = true;
    s_protect_params.flags |= HW_PROTECT_OVER_HEAT;
  }
}

static inline bool hw_can_receive_to_frame(const can_receive_message_struct* hw_msg, can_frame_t* frame)
{
  if (!hw_msg || !frame)
    return false;

  frame->id = hw_msg->rx_sfid;
  frame->dlc = hw_msg->rx_dlen;
  memcpy(frame->data, hw_msg->rx_data, hw_msg->rx_dlen);

  return true;
}

/* ================ 设备状态控制接口实现 ================ */

static bool s_software_brk = false;

bool HW_Interface_GetSoftwareBRK(void)
{
  return s_software_brk;
}

void HW_Interface_SetHardwareFault(void)
{
  s_protect_params.flags |= HW_PROTECT_HARDWARE_FAULT;
}

void HW_Interface_SCISendCallback(void)
{
  // SCI发送完成回调处理
  // 可以在这里处理发送完成的逻辑
}

void HW_Interface_TemperatureProtect(void)
{
  // 温度保护检查实现
  hw_temperature_protect();
}

void HW_Interface_GateState(void)
{
  // 栅极状态检查实现
  // 可以根据需要实现具体的栅极状态检查逻辑
}
