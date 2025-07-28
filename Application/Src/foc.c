#include "foc.h"

#include "hardware_interface.h"
#include "position_sensor.h"


static inline float Get_Theta(float, float);
static inline void Speed_Loop_Control(SpdLoop_t* speed_loop, Park_t* idq_ref);
static inline void Current_Loop_Control(CurLoop_t* hnd, Park_t* out);

// SECTION - FOC Main
void FOC_Update(FOC_Parameter_t* foc)
{
  ClarkeTransform(foc->Iabc_fdbk, foc->Iclark_fdbk);

  switch (foc->Mode)
  {
    case IDLE:
    {
      foc->Stop = 1;
      break;
    }
    case VF_MODE:
    {
      VF_Parameter_t* hnd_vf = foc->Hnd_vf;
      hnd_vf->Theta = Get_Theta(hnd_vf->Freq, hnd_vf->Theta);
      foc->Theta = hnd_vf->Theta;
      ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_ref);
      foc->Udq_ref->d = hnd_vf->Vref_Ud;
      foc->Udq_ref->q = hnd_vf->Vref_Uq;
      break;
    }
    // SECTION - IF Mode
    case IF_MODE:
    {
      IF_Parameter_t* hnd_if = foc->Hnd_if;
      if (hnd_if->Sensor_State == Enable)
      {
        hnd_if->Theta = foc->Theta;
      }
      else
      {
        hnd_if->Theta = Get_Theta(hnd_if->IF_Freq, hnd_if->Theta);
      }
      foc->Theta = hnd_if->Theta;

      ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_ref);

      foc->Idq_ref->d = hnd_if->Id_Ref;
      foc->Idq_ref->q = hnd_if->Iq_Ref;

      // 更新电流环参数
      CurLoop_t* hCurrent = foc->Hnd_curloop;
      memcpy(hCurrent->ref, foc->Idq_ref, sizeof(*(foc->Idq_ref)));
      memcpy(hCurrent->fdbk, foc->Idq_fdbk, sizeof(*(foc->Idq_fdbk)));
      hCurrent->reset = foc->Stop;  // 更新电流环的停止标志

      Current_Loop_Control(hCurrent, foc->Udq_ref);

      break;
    }
    case Speed:
    {
      ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_fdbk);

      SpdLoop_t* hSpeed = foc->Hnd_spdloop;
      hSpeed->ref = foc->SpeedRef;    // 更新目标速度
      hSpeed->fdbk = foc->SpeedFdbk;  // 更新实际速度
      hSpeed->reset = foc->Stop;      // 更新转速环的停止标志

      Speed_Loop_Control(hSpeed, foc->Idq_ref);

      // 更新电流环参数
      CurLoop_t* hCurrent = foc->Hnd_curloop;
      memcpy(hCurrent->ref, foc->Idq_ref, sizeof(*(foc->Idq_ref)));
      memcpy(hCurrent->fdbk, foc->Idq_fdbk, sizeof(*(foc->Idq_fdbk)));
      hCurrent->reset = foc->Stop;  // 更新电流环的停止标志

      Current_Loop_Control(hCurrent, foc->Udq_ref);

      break;
    }
    // !SECTION
    case SHUTDOWN:
    {
      foc->Stop = 1;
      foc->Idq_fdbk->d = 0.0F;  // Id_Ref = 0
      foc->Idq_fdbk->q = 0.0F;  // Iq_Ref = Pid_SpdLoop.output
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

  InvParkTransform(foc->Udq_ref, foc->Theta, foc->Uclark_ref);
}

// SECTION - Speed Loop Control
static inline void Speed_Loop_Control(SpdLoop_t* hnd, Park_t* out)
{
  hnd->counter++;
  if (hnd->counter >= hnd->prescaler)
  {
    hnd->counter = 0;
    hnd->hnd_ramp->target = hnd->ref;  // Update target speed

    float speed_ramp = RampGenerator(hnd->hnd_ramp, hnd->reset);
    Pid_Update(speed_ramp - hnd->fdbk, hnd->reset, hnd->hnd_speed);
    // PID输出在这里被更新到 hnd->hnd_speed->output
  }

  // 每次调用都读取当前PID输出值（保持控制连续性）
  out->q = hnd->hnd_speed->output;  // Iq_Ref = Pid_SpdLoop.output
  out->d = 0.0F;                    // Id_Ref = 0 (按要求暂时设为0)
}

// SECTION - Current Loop Control
static inline void Current_Loop_Control(CurLoop_t* hnd, Park_t* out)
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
