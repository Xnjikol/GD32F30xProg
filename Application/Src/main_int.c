#include "main_int.h"

#include <stdlib.h>


float theta_mech = 0.0F;
float theta_elec = 0.0F;
float theta_factor = 0.0F;  // Sensor data to mechanic angle conversion factor

float Speed_Ref = 0.0F;
float Speed_Fdbk = 0.0F;

DeviceState_t Device = {INIT};
Motor_Parameter_t Motor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Handler_t Id_PID;
PID_Handler_t Iq_PID;
PID_Handler_t Speed_PID;
RampGenerator_t Speed_Ramp;
Clarke_Data_t Inv_Park;
Clarke_Data_t Clarke;
Phase_Data_t Phase_Current;
Park_Data_t DQ_Current;
Park_Data_t DQ_Current_ref;
Park_Data_t DQ_Voltage_ref;

static inline void Main_Int_Parameter_Init(void);
static inline void Theta_Process(Motor_Parameter_t*, float);
static inline void Write_Variables();
static inline void Read_Variables();
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

    Peripheral_UpdateCurrent();
    Peripheral_GateState();
    Peripheral_UpdateUdc();
    Peripheral_UpdatePosition();

    Write_Variables();

    switch (Device.Mode)
    {
      case INIT:
      {
        Main_Int_Parameter_Init();

        if (FOC.Udc > 200.0F)
        {
          Peripheral_EnableHardwareProtect();
        }
        Protect.Flag = No_Protect;
        FOC.Mode = IDLE;
        Device.Mode = READY;
        break;
      }
      case READY:
      {
        FOC.Mode = IDLE;
        Device.Mode = RUNNING;
        break;
      }
      case RUNNING:
      {
        UpdateThetaAndSpeed(&FOC, &Motor);
        FOC_Main(&FOC, &VF, &IF, &Clarke);
        switch (FOC.Mode)
        {
          case EXIT:
          {
            Peripheral_DisableHardwareProtect();
            break;
          }
          default:
            break;
        }
        break;
      }
      default:
        break;
    }

    Read_Variables();
    Peripheral_SetPWMChangePoint();
  }
}

void Main_Int_Parameter_Init(void)
{
  memset(&VF, 0, sizeof(VF_Parameter_t));
  memset(&FOC, 0, sizeof(FOC_Parameter_t));
  memset(&Id_PID, 0, sizeof(PID_Handler_t));
  memset(&Iq_PID, 0, sizeof(PID_Handler_t));
  memset(&Speed_PID, 0, sizeof(PID_Handler_t));
  memset(&Inv_Park, 0, sizeof(Clarke_Data_t));
  memset(&Speed_Ramp, 0, sizeof(RampGenerator_t));
  memset(&Motor, 0, sizeof(Motor_Parameter_t));
  memset(&Phase_Current, 0, sizeof(Phase_Data_t));
  memset(&DQ_Current, 0, sizeof(Park_Data_t));
  memset(&DQ_Current_ref, 0, sizeof(Park_Data_t));
  memset(&DQ_Voltage_ref, 0, sizeof(Park_Data_t));

  Peripheral_InitProtectParameter();
  Peripheral_GetSystemFrequency();
  Peripheral_CalibrateADC();

  FOC.Iabc_fdbk = &Phase_Current;
  FOC.current = (Current_Loop_t*) calloc(1, sizeof(Current_Loop_t));
  FOC.current->fdbk = &DQ_Current;
  FOC.current->ref = &DQ_Current_ref;
  FOC.current->handler_d = &Id_PID;
  FOC.current->handler_q = &Iq_PID;

  FOC.speed = (Speed_Loop_t*) calloc(1, sizeof(Speed_Loop_t));
  FOC.speed->handler = &Speed_PID;
  FOC.speed->ramp = &Speed_Ramp;

  FOC.Udq_ref = &DQ_Voltage_ref;
  FOC.Uclark_ref = &Inv_Park;

  Motor.Rs = 0.65F;
  Motor.Ld = 0.001F;
  Motor.Lq = 0.001F;
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
  Speed_PID.Kp = 0.005F;
  Speed_PID.Ki = 0.03F;
  Speed_PID.Kd = 0.0F;
  Speed_PID.MaxOutput = 0.7F * FOC.I_Max;  // Maximum Iq
  Speed_PID.MinOutput = -0.7F * FOC.I_Max;
  Speed_PID.IntegralLimit = 0.7F * FOC.I_Max;
  Speed_PID.previous_error = 0.0F;
  Speed_PID.integral = 0.0F;
  Speed_PID.output = 0.0F;
  Speed_PID.Ts = FOC.Ts * SPEED_LOOP_PRESCALER;  // Speed loop time;

  Speed_Ramp.slope = 50.0F;  // limit to 50 rpm/s
  Speed_Ramp.limit_min = -1800.0F;
  Speed_Ramp.limit_max = 1800.0F;
  Speed_Ramp.value = 0.0F;
  Speed_Ramp.target = 0.0F;
  Speed_Ramp.Ts = FOC.Ts * SPEED_LOOP_PRESCALER;  // Speed loop time;

  Id_PID.Kp = 73.8274273F;
  Id_PID.Ki = 408.40704496F;
  Id_PID.Kd = 0.0F;
  Id_PID.MaxOutput = 50.0F;  // Maximum Udc/sqrt(3)
  Id_PID.MinOutput = -50.0F;
  Id_PID.IntegralLimit = 50.0F;
  Id_PID.previous_error = 0.0F;
  Id_PID.integral = 0.0F;
  Id_PID.output = 0.0F;
  Id_PID.Ts = FOC.Ts;  // Current loop time
  Id_PID.Reset = true;

  Iq_PID.Kp = 27.646015F;
  Iq_PID.Ki = 408.40704496F;
  Iq_PID.Kd = 0.0F;
  Iq_PID.MaxOutput = 50.0F;
  Iq_PID.MinOutput = -50.0F;
  Iq_PID.IntegralLimit = 50.0F;
  Iq_PID.previous_error = 0.0F;
  Iq_PID.integral = 0.0F;
  Iq_PID.output = 0.0F;
  Iq_PID.Ts = FOC.Ts;  // Current loop time
  Iq_PID.Reset = true;

  VF.Vref_Ud = 0.0F;
  VF.Vref_Uq = 0.0F;
  VF.Freq = 0.0F;
  VF.Theta = 0.0F;

  IF.Id_ref = 0.0F;
  IF.Iq_ref = 0.0F;
  IF.IF_Freq = 0.0F;
  IF.Theta = 0.0F;
  IF.Sensor_State = Disable;
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
  motor->Elec_Theta = wrap_theta_2pi(motor->Elec_Theta);

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

static inline void Write_Variables()
{
  FOC.Stop = STOP;
  FOC.speed->ref = Speed_Ref;
  // FOC.Theta = Sensor.Elec_Theta;
  // FOC.speed->fdbk = Sensor.Speed;
}

static inline void Read_Variables() {}

static inline void UpdateThetaAndSpeed(FOC_Parameter_t* foc, Motor_Parameter_t* motor)
{
  Theta_Process(motor, foc->freq);
  foc->Theta = motor->Elec_Theta;
  foc->speed->fdbk = motor->Speed;
}