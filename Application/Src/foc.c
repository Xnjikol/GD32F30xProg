#include "foc.h"

#include "hardware_interface.h"
#include "position_sensor.h"


static inline float Get_Theta(float, float);

static inline float RampGenerator(RampGenerator_t*);
static inline void SVPWM_Generate(float, float, float, FOC_Parameter_t*);

// SECTION - FOC Main
void FOC_Main(FOC_Parameter_t* foc, VF_Parameter_t* vf, IF_Parameter_t* if_p,
              Clarke_Data_t* I_clarke)
{
  ClarkeTransform(foc->Iabc_fdbk->a, foc->Iabc_fdbk->b, foc->Iabc_fdbk->c, I_clarke);
  Current_Loop_t* hCurrent = foc->current;
  Park_Data_t* idq_ref = hCurrent->ref;
  Park_Data_t* idq_fdbk = hCurrent->fdbk;
  PID_Handler_t* hPid_id = hCurrent->handler_d;
  PID_Handler_t* hPid_iq = hCurrent->handler_q;

  Speed_Loop_t* hSpeed = foc->speed;
  PID_Handler_t* hPid_speed = hSpeed->handler;
  RampGenerator_t* speed_ramp = hSpeed->ramp;

  Clarke_Data_t* inv_park = foc->Uclark_ref;

  switch (foc->Mode)
  {
    case IDLE:
    {
      foc->Stop = 1;
      break;
    }
    case VF_MODE:
    {
      vf->Theta = Get_Theta(vf->Freq, vf->Theta);
      foc->Theta = vf->Theta;
      foc->Udq_ref->d = vf->Vref_Ud;
      foc->Udq_ref->q = vf->Vref_Uq;
      break;
    }
    // SECTION - IF Mode
    case IF_MODE:
    {
      if (if_p->Sensor_State == Enable)
      {
        if_p->Theta = foc->Theta;
      }
      else
      {
        if_p->Theta = Get_Theta(if_p->IF_Freq, if_p->Theta);
      }
      foc->Theta = if_p->Theta;

      ParkTransform(I_clarke->a, I_clarke->b, foc->Theta, idq_fdbk);

      idq_ref->d = if_p->Id_ref;
      idq_ref->q = if_p->Iq_ref;

      if (hPid_id->Reset)
      {
        PID_SetIntegral(hPid_id, foc->Stop, 0.0F);
      }
      Pid_Update(idq_ref->d - idq_fdbk->d, foc->Stop, hPid_id);

      if (hPid_iq->Reset)
      {
        PID_SetIntegral(hPid_iq, foc->Stop, 0.0F);
      }
      Pid_Update(idq_ref->q - idq_fdbk->q, foc->Stop, hPid_iq);

      foc->Udq_ref->d = hPid_id->output;
      foc->Udq_ref->q = hPid_iq->output;

      break;
    }
    // !SECTION
    // SECTION - Speed Mode
    case Speed:
    {
      ParkTransform(I_clarke->a, I_clarke->b, foc->Theta, idq_fdbk);

      static uint16_t Cnt_speed = 0;
      Cnt_speed++;
      if (Cnt_speed > 9)
      {
        Cnt_speed = 0;
        speed_ramp->target = hSpeed->ref;  // Update target speed

        Pid_Update(RampGenerator(speed_ramp) - hSpeed->fdbk, foc->Stop, hPid_speed);
      }

      idq_ref->q = hPid_speed->output;  // Iq_ref = Speed_PID.output
      idq_ref->d = 0.0F;                // Id_ref = 0

      Pid_Update(idq_ref->d - idq_fdbk->d, foc->Stop, hPid_id);
      Pid_Update(idq_ref->q - idq_fdbk->q, foc->Stop, hPid_iq);

      foc->Udq_ref->d = hPid_id->output;
      foc->Udq_ref->q = hPid_iq->output;

      break;
    }
    // !SECTION
    case EXIT:
    {
      foc->Stop = 1;
      idq_fdbk->d = 0.0F;  // Id_ref = 0
      idq_fdbk->q = 0.0F;  // Iq_ref = Speed_PID.output
      foc->Udq_ref->d = 0.0F;
      foc->Udq_ref->q = 0.0F;
      break;
    }
    // SECTION - Identify Mode
    // case Identify:
    // {
    //   ParkTransform(I_clarke.a, I_clarke.b, foc->Theta, &foc);

    //   SquareWaveGenerater(&VoltageInjector, &foc);
    //   foc->Udq_ref->d = VoltageInjector.Vd;
    //   foc->Udq_ref->q = VoltageInjector.Vq;
    //   break;
    // }
    // !SECTION
    default:
    {
      foc->Stop = 1;
      foc->Mode = IDLE;
      break;
    }
  }

  InvParkTransform(foc->Udq_ref->d, foc->Udq_ref->q, foc->Theta, inv_park);
  SVPWM_Generate(inv_park->a, inv_park->b, foc->inv_Udc, foc);
}

// SECTION - Ramp Generator
static inline float RampGenerator(RampGenerator_t* ramp)
{
  if (STOP == 1)
  {
    ramp->value = 0.0F;
    ramp->target = 0.0F;
  }

  float delta = ramp->target - ramp->value;
  float step = ramp->slope * ramp->Ts;

  if (delta > step)
  {
    ramp->value += step;
  }
  else if (delta < -step)
  {
    ramp->value -= step;
  }
  else
  {
    ramp->value = ramp->target;
  }

  if (ramp->value > ramp->limit_max)
  {
    ramp->value = ramp->limit_max;
  }
  if (ramp->value < ramp->limit_min)
  {
    ramp->value = ramp->limit_min;
  }

  return ramp->value;
}

static inline float Get_Theta(float Freq, float Theta)
{
  // 电角度递推：θ += ω·Ts，ω = 2π·f
  Theta += M_2PI * Freq * FOC.Ts;
  if (Theta > M_2PI)
  {
    Theta -= M_2PI;
  }
  else if (Theta < 0.0F)
  {
    Theta += M_2PI;
  }
  return Theta;
}

static inline void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, FOC_Parameter_t* foc)
{
  uint8_t sector = 0;
  float Vref1 = Ubeta;
  float Vref2 = (+SQRT3 * Ualpha - Ubeta) * 0.5F;
  float Vref3 = (-SQRT3 * Ualpha - Ubeta) * 0.5F;

  // 判断扇区（1~6）
  if (Vref1 > 0)
    sector += 1;
  if (Vref2 > 0)
    sector += 2;
  if (Vref3 > 0)
    sector += 4;

  // Clarke to T1/T2 projection
  float X = SQRT3 * Ubeta * inv_Vdc;
  float Y = (+1.5F * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;
  float Z = (-1.5F * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;

  float T1 = 0.0F, T2 = 0.0F;

  switch (sector)
  {
    case 1:
      T1 = Z;
      T2 = Y;
      break;
    case 2:
      T1 = Y;
      T2 = -X;
      break;
    case 3:
      T1 = -Z;
      T2 = X;
      break;
    case 4:
      T1 = -X;
      T2 = Z;
      break;
    case 5:
      T1 = X;
      T2 = -Y;
      break;
    case 6:
      T1 = -Y;
      T2 = -Z;
      break;
    default:
      T1 = 0.0F;
      T2 = 0.0F;
      break;
  }

  // 过调制处理
  float T_sum = T1 + T2;
  if (T_sum > 1.0F)
  {
    T1 /= T_sum;
    T2 /= T_sum;
  }

  // 中心对称调制时间计算
  float T0 = (1.0F - T1 - T2) * 0.5F;
  float Ta = T0;
  float Tb = T0 + T1;
  float Tc = Tb + T2;

  float Tcm1 = 0.0F;
  float Tcm2 = 0.0F;
  float Tcm3 = 0.0F;

  // 扇区映射到ABC换相点
  switch (sector)
  {
    case 1:
      Tcm1 = Tb;
      Tcm2 = Ta;
      Tcm3 = Tc;
      break;
    case 2:
      Tcm1 = Ta;
      Tcm2 = Tc;
      Tcm3 = Tb;
      break;
    case 3:
      Tcm1 = Ta;
      Tcm2 = Tb;
      Tcm3 = Tc;
      break;
    case 4:
      Tcm1 = Tc;
      Tcm2 = Tb;
      Tcm3 = Ta;
      break;
    case 5:
      Tcm1 = Tc;
      Tcm2 = Ta;
      Tcm3 = Tb;
      break;
    case 6:
      Tcm1 = Tb;
      Tcm2 = Tc;
      Tcm3 = Ta;
      break;
    default:
      Tcm1 = 0.5F;
      Tcm2 = 0.5F;
      Tcm3 = 0.5F;
      break;
  }

  foc->Tcm1 = Tcm1;
  foc->Tcm2 = Tcm2;
  foc->Tcm3 = Tcm3;
}
