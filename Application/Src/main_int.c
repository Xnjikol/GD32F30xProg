#include "main_int.h"

#include <math.h>

#include "foc.h"
#include "hw_interface.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/* ================ 外部变量声明 ================ */
extern FOC_Parameter_t FOC;
extern Motor_Parameter_t Motor;
extern DeviceState_t Device;

/* ================ 静态变量 ================ */
static bool s_hw_interface_initialized = false;

/* ================ 公共函数实现 ================ */

void Motor_Init(Motor_Parameter_t* motor, DeviceState_t* device)
{
  if (!motor || !device)
    return;

  /* 电机基本参数初始化 */
  motor->Rs = 0.65f;                      // 定子电阻 (Ω)
  motor->Ld = 0.1175f;                    // d轴电感 (H)
  motor->Lq = 0.044f;                     // q轴电感 (H)
  motor->Flux = 0.00656f;                 // 磁链 (Wb)
  motor->Pn = 2.0f;                       // 极对数
  motor->Resolver_Pn = 1.0f;              // 旋变极对数
  motor->inv_MotorPn = 1.0f / motor->Pn;  // 极对数倒数

  /* 位置相关参数初始化 */
  motor->Position = 0.0f;                   // 位置反馈初始值
  motor->Position_Scale = 10000.0f - 1.0f;  // 位置量程
  motor->Position_Offset = 107.0f;          // 零位偏移
  motor->Mech_Theta = 0.0f;                 // 机械角度
  motor->Elec_Theta = 0.0f;                 // 电气角度
  motor->Speed = 0.0f;                      // 转速反馈

  /* 角度转换系数计算 */
#ifdef Resolver_Position
  motor->theta_factor = 2.0f * M_PI / ((motor->Position_Scale + 1.0f) * motor->Resolver_Pn);
#endif
#ifdef Encoder_Position
  motor->theta_factor = 2.0f * M_PI / (motor->Position_Scale + 1.0f);
#endif

  /* 设置指向设备时间频率参数的指针 */
  motor->main_Ts_ptr = &device->main_Ts;
  motor->main_Freq_ptr = &device->main_Freq;
  motor->speed_Ts_ptr = &device->speed_Ts;
  motor->speed_Freq_ptr = &device->speed_Freq;
}

void Main_Int_Init(void)
{
  Device.basic_init_done = false;
  Device.full_init_done = false;
  Device.system_params_valid = false;

  /* FOC系统初始化 */
  if (FOC_Init(&FOC))
  {
    /* Motor参数初始化 */
    Motor_Init(&Motor, &Device);

    /* 初始化硬件接口 */
    if (HW_Interface_Init(&FOC, &Motor, &Device))
    {
      s_hw_interface_initialized = true;
      Device.basic_init_done = true;
    }
  }
}

void Main_Int_Handler(void)
{
  if (adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOIC))
  {
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    /* 通过硬件接口层更新所有外设数据 */
    HW_Interface_Process();

    /* 执行FOC算法 */
    switch (Device.Mode)
    {
      case INIT:
      {
        /* 基础初始化：仅获取系统参数 */
        Main_Int_Init();
        Device.Mode = WAITING;
        if (Device.system_params_valid)
        {
          Device.basic_init_done = true;

          FOC.Mode = IDLE;
        }
        break;
      }

      case WAITING:
      {
        /* 等待状态 */
        HW_Interface_SetDeviceStop(true);
        break;
      }

      case SETUP:
      {
        HW_Interface_SetDeviceStop(true);
        /* 设置状态，进行完整的FOC参数初始化 */
        /* FOC完整初始化 */
        if (!FOC.initialized)
        {
          /* 初始化状态 */
          if (FOC.Udc > 200.0f)  // 200V
          {
            /* 基础初始化 */
            HW_Interface_InitProtectParams();
            HW_Interface_GetSystemFrequency();

            /* 设置基本参数 */
            Main_Int_Init();

            HW_Interface_EnableHardwareProtect();
            HW_Interface_CalibrateADC();
            FOC.Mode = IDLE;
            FOC.initialized = true;
            /* 标记完整初始化完成 */
            Device.full_init_done = true;
            Device.Mode = READY;
          }
        }
        break;
      }

      case READY:
      {
        HW_Interface_SetDeviceStop(true);
        /* 就绪状态，可以开始运行 */
        Device.Mode = RUNNING;
        FOC.Mode = IDLE;
        break;
      }

      case RUNNING:
      {
        /* 运行状态，一直执行FOC控制 */
        /* Stop变量用于控制IGBT管和重置积分值，但不改变状态和FOC模式 */
        if (HW_Interface_GetDeviceStop())
        {
          /* 设备停止时：关闭IGBT，重置积分值，但继续执行FOC算法 */
          /* 不改变FOC.Mode，只处理停止相关的逻辑 */
          /* TODO: 在这里可以添加PID积分重置逻辑 */
          /* TODO: 在这里可以添加IGBT关闭逻辑 */
        }

        /* 无论Stop状态如何，都执行FOC主函数 */
        FOC_Main(&FOC);
        break;
      }

      case SHUTDOWN:
      {
        /* 关机状态 */
        HW_Interface_SetDeviceStop(true);
        FOC.Mode = IDLE;

        /* 检查是否可以恢复 */
        HW_ProtectFlags_t flags = HW_Interface_GetProtectFlags();
        if (flags == HW_PROTECT_NONE)
        {
          Device.Mode = READY;
        }
        break;
      }

      default:
        Device.Mode = WAITING;
        break;
    }

    /* 检查保护状态 */
    // HW_ProtectFlags_t protect_flags = HW_Interface_GetProtectFlags();
    // if (protect_flags != HW_PROTECT_NONE && Device.Mode != SHUTDOWN)
    // {
    //   Device.Mode = SHUTDOWN;
    //   HW_Interface_SetDeviceStop(true);
    // }

    /* 获取PWM输出并设置到硬件 */
    Phase_t tcm_output;
    if (HW_Interface_GetPWMOutput(&tcm_output))
    {
      timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, (uint16_t) tcm_output.a);
      timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, (uint16_t) tcm_output.b);
      timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, (uint16_t) tcm_output.c);
    }
  }
}

/* ================ 便利函数实现 ================ */

void Main_Int_TriggerFullInit(void)
{
  if (Device.Mode == WAITING)
  {
    Device.Mode = SETUP;
  }
}

bool Main_Int_IsBasicReady(void)
{
  return Device.Mode >= WAITING;
}

bool Main_Int_IsFullyReady(void)
{
  return Device.Mode >= READY;
}

void Main_Int_SetDeviceStop(bool stop)
{
  HW_Interface_SetDeviceStop(stop);
}

bool Main_Int_GetDeviceStop(void)
{
  return HW_Interface_GetDeviceStop();
}

void Main_Int_EnableHardwareProtect(bool enable)
{
  if (enable)
  {
    HW_Interface_EnableHardwareProtect();
  }
  else
  {
    HW_Interface_DisableHardwareProtect();
  }
}

void Main_Int_ClearProtectFlags(HW_ProtectFlags_t flags)
{
  HW_Interface_ClearProtectFlags(flags);
}

HW_ProtectFlags_t Main_Int_GetProtectFlags(void)
{
  return HW_Interface_GetProtectFlags();
}

/* ================ 通信接口 ================ */

bool Main_Int_CANSend(const can_frame_t* frame)
{
  return HW_Interface_CANSend(frame);
}

bool Main_Int_CANReceive(can_frame_t* frame)
{
  return HW_Interface_CANReceive(frame);
}

void Main_Int_SCISend(float* data, uint8_t count)
{
  HW_Interface_SCISend(data, count);
}
