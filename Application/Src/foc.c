#include "foc.h"

#include "hardware_interface.h"
#include "position_sensor.h"


static inline float Get_Theta(float, float);
static inline void SVPWM_Generate(Clark_t*, float, Phase_t*);
static inline void Speed_Loop_Control(SpdLoop_t* speed_loop, Park_t* idq_ref);
static inline void Current_Loop_Control(CurLoop_t* hnd, Park_t* out);

// SECTION - FOC Main
void FOC_Main(FOC_Parameter_t* foc, VF_Parameter_t* vf, IF_Parameter_t* if_p)
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
      vf->Theta = Get_Theta(vf->Freq, vf->Theta);
      foc->Theta = vf->Theta;
      ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_ref);
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

      ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_ref);

      foc->Idq_ref->d = if_p->Id_ref;
      foc->Idq_ref->q = if_p->Iq_ref;

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
      hSpeed->reset = foc->Stop;  // 更新转速环的停止标志

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
    case EXIT:
    {
      foc->Stop = 1;
      foc->Idq_fdbk->d = 0.0F;  // Id_ref = 0
      foc->Idq_fdbk->q = 0.0F;  // Iq_ref = Pid_SpdLoop.output
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
  SVPWM_Generate(foc->Uclark_ref, foc->inv_Udc, foc->Tcm);
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
  out->q = hnd->hnd_speed->output;  // Iq_ref = Pid_SpdLoop.output
  out->d = 0.0F;                    // Id_ref = 0 (按要求暂时设为0)
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

static inline void SVPWM_Generate(Clark_t* u_ref, float inv_Vdc, Phase_t* out)
{
  float alpha = u_ref->a;
  float beta = u_ref->b;
  uint8_t sector = 0;
  float v_ref1 = beta;
  float v_ref2 = (+SQRT3 * alpha - beta) * 0.5F;
  float v_ref3 = (-SQRT3 * alpha - beta) * 0.5F;

  // 判断扇区（1~6）
  if (v_ref1 > 0)
    sector += 1;
  if (v_ref2 > 0)
    sector += 2;
  if (v_ref3 > 0)
    sector += 4;

  // Clarke to t1/t2 projection
  float X = SQRT3 * beta * inv_Vdc;
  float Y = (+1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;
  float Z = (-1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;

  float t1 = 0.0F, t2 = 0.0F;

  switch (sector)
  {
    case 1:
      t1 = Z;
      t2 = Y;
      break;
    case 2:
      t1 = Y;
      t2 = -X;
      break;
    case 3:
      t1 = -Z;
      t2 = X;
      break;
    case 4:
      t1 = -X;
      t2 = Z;
      break;
    case 5:
      t1 = X;
      t2 = -Y;
      break;
    case 6:
      t1 = -Y;
      t2 = -Z;
      break;
    default:
      t1 = 0.0F;
      t2 = 0.0F;
      break;
  }

  // 过调制处理
  float T_sum = t1 + t2;
  if (T_sum > 1.0F)
  {
    t1 /= T_sum;
    t2 /= T_sum;
  }

  // 中心对称调制时间计算
  float t0 = (1.0F - t1 - t2) * 0.5F;
  float ta = t0;
  float tb = t0 + t1;
  float tc = tb + t2;

  Phase_t tcm = {0.0F, 0.0F, 0.0F};

  // 扇区映射到ABC换相点
  switch (sector)
  {
    case 1:
      tcm.a = tb;
      tcm.b = ta;
      tcm.c = tc;
      break;
    case 2:
      tcm.a = ta;
      tcm.b = tc;
      tcm.c = tb;
      break;
    case 3:
      tcm.a = ta;
      tcm.b = tb;
      tcm.c = tc;
      break;
    case 4:
      tcm.a = tc;
      tcm.b = tb;
      tcm.c = ta;
      break;
    case 5:
      tcm.a = tc;
      tcm.b = ta;
      tcm.c = tb;
      break;
    case 6:
      tcm.a = tb;
      tcm.b = tc;
      tcm.c = ta;
      break;
    default:
      tcm.a = 0.5F;
      tcm.b = 0.5F;
      tcm.c = 0.5F;
      break;
  }

  *out = tcm;
}
