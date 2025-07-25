#include "foc.h"

#include "hardware_interface.h"
#include "position_sensor.h"


static inline float Get_Theta(float, float);
static inline void SVPWM_Generate(float, float, float, FOC_Parameter_t*);
static inline void Speed_Loop_Control(Speed_Loop_t* speed_loop, Park_t* idq_ref);
static inline void Current_Loop_Control(Current_Loop_t* hnd, Park_t* out);

// SECTION - FOC Main
void FOC_Main(FOC_Parameter_t* foc, VF_Parameter_t* vf, IF_Parameter_t* if_p)
{
  ClarkeTransform(foc->Iabc_fdbk, foc->IClark_fdbk);
  Current_Loop_t* hCurrent = foc->current;
  Park_t* idq_ref = hCurrent->ref;
  Park_t* idq_fdbk = hCurrent->fdbk;
  Clark_t* inv_park = foc->Uclark_ref;

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

      ParkTransform(foc->IClark_fdbk, foc->Theta, idq_fdbk);

      idq_ref->d = if_p->Id_ref;
      idq_ref->q = if_p->Iq_ref;

      // 更新电流环的停止标志
      hCurrent->reset = foc->Stop;

      Current_Loop_Control(hCurrent, foc->Udq_ref);

      break;
    }
    // !SECTION
    // SECTION - Speed Mode
    case Speed:
    {
      ParkTransform(foc->IClark_fdbk, foc->Theta, idq_fdbk);

      Speed_Loop_t* hSpeed = foc->speed;
      // 更新转速环的停止标志
      hSpeed->reset = foc->Stop;

      Speed_Loop_Control(hSpeed, idq_ref);

      // 更新电流环的停止标志
      hCurrent->reset = foc->Stop;

      Current_Loop_Control(hCurrent, foc->Udq_ref);

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
    default:
    {
      foc->Stop = 1;
      foc->Mode = IDLE;
      break;
    }
  }

  InvParkTransform(foc->Udq_ref, foc->Theta, inv_park);
  SVPWM_Generate(inv_park->a, inv_park->b, foc->inv_Udc, foc);
}

// SECTION - Speed Loop Control
static inline void Speed_Loop_Control(Speed_Loop_t* hnd, Park_t* out)
{
  hnd->counter++;
  if (hnd->counter >= hnd->prescaler)
  {
    hnd->counter = 0;
    hnd->ramp->target = hnd->ref;  // Update target speed

    float speed_ramp = RampGenerator(hnd->ramp, hnd->reset);
    Pid_Update(speed_ramp - hnd->fdbk, hnd->reset, hnd->handler);
    // PID输出在这里被更新到 hnd->handler->output
  }

  // 每次调用都读取当前PID输出值（保持控制连续性）
  out->q = hnd->handler->output;  // Iq_ref = Speed_PID.output
  out->d = 0.0F;                  // Id_ref = 0 (按要求暂时设为0)
}

// SECTION - Current Loop Control
static inline void Current_Loop_Control(Current_Loop_t* hnd, Park_t* out)
{
  // D轴电流控制
  if (hnd->handler_d->Reset)
  {
    PID_SetIntegral(hnd->handler_d, hnd->reset, 0.0F);
  }
  Pid_Update(hnd->ref->d - hnd->fdbk->d, hnd->reset, hnd->handler_d);

  // Q轴电流控制
  if (hnd->handler_q->Reset)
  {
    PID_SetIntegral(hnd->handler_q, hnd->reset, 0.0F);
  }
  Pid_Update(hnd->ref->q - hnd->fdbk->q, hnd->reset, hnd->handler_q);

  // 输出DQ轴电压指令
  out->d = hnd->handler_d->output;
  out->q = hnd->handler_q->output;
}

// SECTION - Ramp Generator

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
