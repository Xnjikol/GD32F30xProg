#include "main_int.h"

#include <stdlib.h>


float theta_mech = 0.0F;
float theta_elec = 0.0F;
float theta_factor = 0.0F;  // Sensor data to mechanic angle conversion factor

float Speed_Ref = 0.0F;
float Speed_Fdbk = 0.0F;

DeviceState_t Device;
Motor_Parameter_t Motor;
FOC_Parameter_t FOC;
VF_Parameter_t OpenLoop_VF;
IF_Parameter_t OpenLoop_IF;
PID_Handler_t Pid_CurLoop_d;
PID_Handler_t Pid_CurLoop_q;
PID_Handler_t Pid_SpdLoop;
RampGenerator_t Rmp_Speed;
Clark_t VoltageClark_Ref;
Clark_t CurrentClark_Fdbk;
Phase_t CurrentPhase_Fdbk;
Phase_t TcmPhase_Ref;
Park_t CurrentPark_Fdbk;
Park_t CurrentPark_Ref;
Park_t VoltagePark_Ref;

static inline void Main_Int_Parameter_Init(void);
static inline void Main_Int_Basic_Init(void);
static inline void Theta_Process(Motor_Parameter_t*, float);
static inline void UpdateThetaAndSpeed(FOC_Parameter_t* foc, Motor_Parameter_t* motor);

/*!
    \brief      主中断函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Handler(void)
{
  if (adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOIC))
  {
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    // 基础外设更新（无论初始化状态如何都要执行）
    Peripheral_UpdateCurrent();
    Peripheral_GateState();
    Peripheral_UpdateUdc();
    Peripheral_UpdatePosition();

    switch (Device.Mode)
    {
      case INIT:
      {
        // 基础初始化：仅获取系统参数，不进行完整的FOC初始化
        Main_Int_Basic_Init();

        if (Device.system_params_valid)
        {
          Device.basic_init_done = true;
          Device.Mode = WAITING;
          FOC.Mode = IDLE;
        }
        break;
      }

      case WAITING:
      {
        // 基础就绪状态：系统参数已获取，等待完整初始化触发
        // 继续运行基本逻辑但不进行FOC控制
        if (FOC.Udc > 200.0F)
        {
          Peripheral_EnableHardwareProtect();
        }

        // 检查是否触发完整初始化（通过CCP协议修改Device.Mode）
        // 这里不做状态转换，由外部CCP协议触发
        FOC.Mode = IDLE;
        break;
      }

      case SETUP:
      {
        // 完整初始化：用户触发的完整参数初始化
        Main_Int_Parameter_Init();

        if (FOC.Udc > 200.0F)
        {
          Peripheral_EnableHardwareProtect();
        }
        Protect.Flag = No_Protect;
        Device.full_init_done = true;
        Device.Mode = READY;
        FOC.Mode = IDLE;
        break;
      }

      case READY:
      {
        // 就绪状态：准备进入运行状态
        FOC.Mode = IDLE;
        Device.Mode = RUNNING;
        break;
      }

      case RUNNING:
      {
        // 运行状态：正常FOC控制
        UpdateThetaAndSpeed(&FOC, &Motor);
        FOC.Stop = Device_Stop;
        FOC.SpeedRef = Speed_Ref;
        FOC_Main(&FOC);
        break;
      }

      case SHUTDOWN:
      {
        Peripheral_DisableHardwareProtect();
        break;
      }

      default:
        break;
    }

    Peripheral_SetPWMChangePoint();
  }
}

/*!
    \brief      基础初始化函数 - 仅获取系统参数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Basic_Init(void)
{
  static bool basic_init_started = false;

  if (!basic_init_started)
  {
    // 初始化基础保护参数
    Peripheral_InitProtectParameter();

    // 获取系统频率 - 这是关键步骤
    Peripheral_GetSystemFrequency();

    // 标记基础初始化已开始
    basic_init_started = true;
  }

  // 检查系统参数是否已获取到有效值
  if (FOC.Ts > 0.0F && FOC.freq > 0.0F)
  {
    Device.system_params_valid = true;
  }
}

void Main_Int_Parameter_Init(void)
{
  // 注意：基础系统参数（Ts, freq等）已在Main_Int_Basic_Init中获取，这里不再重复

  memset(&OpenLoop_VF, 0, sizeof(VF_Parameter_t));
  memset(&FOC, 0, sizeof(FOC_Parameter_t));
  memset(&Pid_CurLoop_d, 0, sizeof(PID_Handler_t));
  memset(&Pid_CurLoop_q, 0, sizeof(PID_Handler_t));
  memset(&Pid_SpdLoop, 0, sizeof(PID_Handler_t));
  memset(&VoltageClark_Ref, 0, sizeof(Clark_t));
  memset(&CurrentClark_Fdbk, 0, sizeof(Clark_t));
  memset(&Rmp_Speed, 0, sizeof(RampGenerator_t));
  memset(&Motor, 0, sizeof(Motor_Parameter_t));
  memset(&CurrentPhase_Fdbk, 0, sizeof(Phase_t));
  memset(&TcmPhase_Ref, 0, sizeof(Phase_t));
  memset(&CurrentPark_Fdbk, 0, sizeof(Park_t));
  memset(&CurrentPark_Ref, 0, sizeof(Park_t));
  memset(&VoltagePark_Ref, 0, sizeof(Park_t));

  // 由于基础初始化已完成，这里只需要校准ADC
  Peripheral_InitProtectParameter();  // 已在基础初始化中完成
  Peripheral_GetSystemFrequency();    // 已在基础初始化中完成
  Peripheral_CalibrateADC();

  FOC.Iabc_fdbk = &CurrentPhase_Fdbk;
  FOC.Iclark_fdbk = &CurrentClark_Fdbk;
  FOC.Idq_ref = &CurrentPark_Ref;
  FOC.Idq_fdbk = &CurrentPark_Fdbk;
  FOC.Tcm = &TcmPhase_Ref;

  // 开环相关变量
  FOC.Hnd_vf = &OpenLoop_VF;
  FOC.Hnd_if = &OpenLoop_IF;

  // 电流环相关变量
  FOC.Hnd_curloop = (CurLoop_t*) calloc(1, sizeof(CurLoop_t));
  FOC.Hnd_curloop->fdbk = (Park_t*) calloc(1, sizeof(Park_t));
  FOC.Hnd_curloop->ref = (Park_t*) calloc(1, sizeof(Park_t));
  FOC.Hnd_curloop->handler_d = &Pid_CurLoop_d;
  FOC.Hnd_curloop->handler_q = &Pid_CurLoop_q;
  FOC.Hnd_curloop->reset = 0;  // 初始化停止标志

  // 转速环相关变量
  FOC.Hnd_spdloop = (SpdLoop_t*) calloc(1, sizeof(SpdLoop_t));
  FOC.Hnd_spdloop->hnd_speed = &Pid_SpdLoop;
  FOC.Hnd_spdloop->hnd_ramp = &Rmp_Speed;
  FOC.Hnd_spdloop->reset = 0;                                    // 初始化停止标志
  FOC.Hnd_spdloop->prescaler = (uint16_t) SPEED_LOOP_PRESCALER;  // 设置分频数
  FOC.Hnd_spdloop->counter = 0;                                  // 初始化分频计数器

  FOC.Udq_ref = &VoltagePark_Ref;
  FOC.Uclark_ref = &VoltageClark_Ref;

  Motor.Rs = 1.25F;
  Motor.Ld = 0.006F;
  Motor.Lq = 0.009F;
  Motor.Flux = 0.1F;
  Motor.Pn = 5.0F;
  Motor.Position_Scale = 65536 - 1;
  Motor.Resolver_Pn = 1.0F;
  Motor.inv_MotorPn = 1.0F / 2.0F;  // Pn
  Motor.Position_Offset = 39833.000000F;

#ifdef Resolver_Position
  theta_factor = M_2PI / ((Motor.Position_Scale + 1) * Motor.Resolver_Pn);
#endif
#ifdef Encoder_Position
  theta_factor = M_2PI / (float) (Motor.Position_Scale + 1);
#endif
  Rmp_Speed.slope = 50.0F;  // limit to 50 rpm/s
  Rmp_Speed.limit_min = -1800.0F;
  Rmp_Speed.limit_max = 1800.0F;
  Rmp_Speed.value = 0.0F;
  Rmp_Speed.target = 0.0F;
  Rmp_Speed.Ts = FOC.Ts * SPEED_LOOP_PRESCALER;  // Speed loop time;

  Pid_SpdLoop.Kp = 0.005F;
  Pid_SpdLoop.Ki = 0.03F;
  Pid_SpdLoop.Kd = 0.0F;
  Pid_SpdLoop.MaxOutput = 0.7F * FOC.I_Max;  // Maximum Iq
  Pid_SpdLoop.MinOutput = -0.7F * FOC.I_Max;
  Pid_SpdLoop.IntegralLimit = 0.7F * FOC.I_Max;
  Pid_SpdLoop.previous_error = 0.0F;
  Pid_SpdLoop.integral = 0.0F;
  Pid_SpdLoop.output = 0.0F;
  Pid_SpdLoop.Ts = FOC.Ts * SPEED_LOOP_PRESCALER;  // Speed loop time;

  Pid_CurLoop_d.Kp = 73.8274273F;
  Pid_CurLoop_d.Ki = 408.40704496F;
  Pid_CurLoop_d.Kd = 0.0F;
  Pid_CurLoop_d.MaxOutput = 50.0F;  // Maximum Udc/sqrt(3)
  Pid_CurLoop_d.MinOutput = -50.0F;
  Pid_CurLoop_d.IntegralLimit = 50.0F;
  Pid_CurLoop_d.previous_error = 0.0F;
  Pid_CurLoop_d.integral = 0.0F;
  Pid_CurLoop_d.output = 0.0F;
  Pid_CurLoop_d.Ts = FOC.Ts;  // Current loop time
  Pid_CurLoop_d.Reset = true;

  Pid_CurLoop_q.Kp = 27.646015F;
  Pid_CurLoop_q.Ki = 408.40704496F;
  Pid_CurLoop_q.Kd = 0.0F;
  Pid_CurLoop_q.MaxOutput = 50.0F;
  Pid_CurLoop_q.MinOutput = -50.0F;
  Pid_CurLoop_q.IntegralLimit = 50.0F;
  Pid_CurLoop_q.previous_error = 0.0F;
  Pid_CurLoop_q.integral = 0.0F;
  Pid_CurLoop_q.output = 0.0F;
  Pid_CurLoop_q.Ts = FOC.Ts;  // Current loop time
  Pid_CurLoop_q.Reset = true;

  OpenLoop_VF.Vref_Ud = 0.0F;
  OpenLoop_VF.Vref_Uq = 0.0F;
  OpenLoop_VF.Freq = 0.0F;
  OpenLoop_VF.Theta = 0.0F;

  OpenLoop_IF.Id_Ref = 0.0F;
  OpenLoop_IF.Iq_Ref = 0.0F;
  OpenLoop_IF.IF_Freq = 0.0F;
  OpenLoop_IF.Theta = 0.0F;
  OpenLoop_IF.Sensor_State = Disable;
}

void FOC_UpdateMainFrequency(float freq, float Ts, float PWM_ARR)
{
  FOC.freq = freq;
  FOC.Ts = Ts;
  FOC.PWM_ARR = PWM_ARR;
}

static inline void Theta_Process(Motor_Parameter_t* motor, float freq)
{
  freq = freq / SPEED_LOOP_PRESCALER;
  // 位置传感器数据处理
  float delta = motor->Position - motor->Position_Offset;
  if (delta < 0)
  {
    delta += (float) (motor->Position_Scale + 1);
  }

  motor->Mech_Theta = delta * theta_factor;

  motor->Elec_Theta = motor->Mech_Theta * motor->Pn;
  motor->Elec_Theta = wrap_theta_pi(motor->Elec_Theta);

  static uint16_t cnt_speed = 0;
  cnt_speed++;
  if (cnt_speed >= SPEED_LOOP_PRESCALER)
  {
    cnt_speed = 0;

    static float last_theta = 0.0F;
    float delta_theta = motor->Mech_Theta - last_theta;

    delta_theta = wrap_theta_pi(delta_theta);

    last_theta = motor->Mech_Theta;

    motor->Speed = radps2rpm(delta_theta * freq);

    static LowPassFilter_t hLPF_speed = {.initialized = false};
    if (!hLPF_speed.initialized)
    {  // 初始化低通滤波器
      LowPassFilter_Init(&hLPF_speed, 10.0F, freq);
      hLPF_speed.initialized = true;
    }
    motor->Speed = LowPassFilter_Update(&hLPF_speed, motor->Speed);
  }
}


static inline void UpdateThetaAndSpeed(FOC_Parameter_t* foc, Motor_Parameter_t* motor)
{
  Theta_Process(motor, foc->freq);
  foc->Theta = motor->Elec_Theta;
  foc->SpeedFdbk = motor->Speed;
}