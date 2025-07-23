#include "main_int.h"

Motor_Parameter_t Motor;
Sensor_Parameter_t Sensor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
PID_Controller_t Speed_PID;
RampGenerator_t Speed_Ramp;
Clarke_t Inv_Park;
Clarke_t Clarke;
PhaseABC_t Phase_Current;
Park_t DQ_Current;
Park_t DQ_Current_ref;
Park_t DQ_Voltage_ref;

float theta_mech = 0.0F;
float theta_elec = 0.0F;
float theta_factor = 0.0F;  // Sensor data to mechanic angle conversion factor

float Speed_Ref = 0.0F;

LowPassFilter_t Speed_Filter = {.alpha = 0.9685841F, .prev_output = 0.0F, .initialized = false};

static inline void Main_Int_Parameter_Init(void);
static inline void Theta_Process(float, float, float*, float*);

/*!
    \brief      Main interrupt handler function.
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

    Theta_Process(Sensor.Position, Motor.Position_Offset, &Sensor.Elec_Theta, &Sensor.Speed);

    FOC_Main(&FOC, &Motor, &VF, &IF, &Id_PID, &Iq_PID, &Speed_PID, &Speed_Ramp, &Inv_Park, &Clarke,
             &Speed_Ref);

    switch (FOC.Mode)
    {
      case INIT:
      {
        Peripheral_InitProtectParameter();
        Peripheral_GetSystemFrequency();
        Peripheral_CalibrateADC();
        Main_Int_Parameter_Init();
        if (FOC.Udc > 200.0F)
        {
          Peripheral_EnableHardwareProtect();
        }
        Protect.Flag = No_Protect;
        FOC.Mode = IDLE;
        break;
      }
      case IDLE:
      case VF_MODE:
      case IF_MODE:
      case Speed:
      {
        break;
      }
      // case Identify:
      // {
      //   float DMA_Buffer[3];
      //   DMA_Buffer[0] = VoltageInjector.Vq;
      //   DMA_Buffer[1] = FOC.Ipark->q;
      //   DMA_Buffer[2] = (float) VoltageInjector.Count;
      //   justfloat(DMA_Buffer, 3);
      //   break;
      // }
      case EXIT:
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

void Main_Int_Parameter_Init(void)
{
  memset(&VF, 0, sizeof(VF_Parameter_t));
  memset(&FOC, 0, sizeof(FOC_Parameter_t));
  memset(&Id_PID, 0, sizeof(PID_Controller_t));
  memset(&Iq_PID, 0, sizeof(PID_Controller_t));
  memset(&Speed_PID, 0, sizeof(PID_Controller_t));
  memset(&Inv_Park, 0, sizeof(Clarke_t));
  memset(&Speed_Ramp, 0, sizeof(RampGenerator_t));
  memset(&Motor, 0, sizeof(Motor_Parameter_t));
  memset(&Phase_Current, 0, sizeof(PhaseABC_t));
  memset(&DQ_Current, 0, sizeof(Park_t));
  memset(&DQ_Current_ref, 0, sizeof(Park_t));
  memset(&DQ_Voltage_ref, 0, sizeof(Park_t));

  FOC.Iphase = &Phase_Current;
  FOC.Ipark = &DQ_Current;
  FOC.Upark_ref = &DQ_Voltage_ref;
  FOC.Ipark_ref = &DQ_Current_ref;

  STOP = 1;

  Motor.Rs = 0.65F;
  Motor.Ld = 0.001F;
  Motor.Lq = 0.001F;
  Motor.Flux = 0.1F;
  Motor.Pn = 2.0F;
  Motor.Position_Scale = 10000 - 1;
  Motor.Resolver_Pn = 1.0F;
  Motor.inv_MotorPn = 1.0F / 2.0F;  // Pn
  Motor.Position_Offset = 107.0F;

#ifdef Resolver_Position
  theta_factor = M_2PI / ((Motor.Position_Scale + 1) * Motor.Resolver_Pn);
#endif
#ifdef Encoder_Position
  theta_factor = M_2PI / (float) (Motor.Position_Scale + 1);
#endif
  Speed_PID.Kp = 0.0F;
  Speed_PID.Ki = 0.0F;
  Speed_PID.Kd = 0.0F;
  Speed_PID.MaxOutput = 0.7F * FOC.I_Max;  // Maximum Iq
  Speed_PID.MinOutput = -0.7F * FOC.I_Max;
  Speed_PID.IntegralLimit = 0.7F * FOC.I_Max;
  Speed_PID.previous_error = 0.0F;
  Speed_PID.integral = 0.0F;
  Speed_PID.output = 0.0F;
  Speed_PID.Ts = FOC.Ts / SPEED_LOOP_PRESCALER;  // Speed loop time;

  Speed_Ramp.slope = 50.0F;  // limit to 50 rpm/s
  Speed_Ramp.limit_min = -1800.0F;
  Speed_Ramp.limit_max = 1800.0F;
  Speed_Ramp.value = 0.0F;
  Speed_Ramp.target = 0.0F;
  Speed_Ramp.Ts = FOC.Ts / SPEED_LOOP_PRESCALER;  // Speed loop time;

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

void FOC_UpdateMainFrequency(float f, float Ts, float PWM_ARR)
{
  FOC.f = f;
  FOC.Ts = Ts;
  FOC.PWM_ARR = PWM_ARR;
}

static inline void Theta_Process(float pos, float offset, float* theta, float* speed)
{
  // 位置传感器数据处理
  float delta = pos - offset;
  if (delta < 0)
  {
    delta += (float) (Motor.Position_Scale + 1);
  }

  float theta_mech = delta * theta_factor;

  float theta_elec = theta_mech * Motor.Pn;
  *theta = wrap_theta_2pi(theta_elec);

  static float last_theta = 0.0F;
  float delta_theta = theta_mech - last_theta;

  delta_theta = wrap_theta_2pi(delta_theta);

  last_theta = theta_mech;

  float temp_speed = delta_theta * FOC.Ts * 60.0F / M_2PI;
  if (!Speed_Filter.initialized)
  {
    Speed_Filter.prev_output = temp_speed;
    LowPassFilter_Init(&Speed_Filter, 10.0F, 1.0F / FOC.Ts);
    Speed_Filter.initialized = true;
  }
  *speed = LowPassFilter_Update(&Speed_Filter, temp_speed);
}
