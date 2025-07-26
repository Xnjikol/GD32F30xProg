#include "foc.h"

#include "hw_interface.h"
#include "position_sensor.h"

/* ================ FOC模块内部全局变量定义 ================ */
// 三相电流、电压结构体
static Phase_t Foc_CurPhase_Fdbk = {0};  // 三相电流反馈
static Phase_t Foc_Pwm_Tcm = {0};        // 三相PWM输出

// Clarke和Park变换结构体
static Clark_t Foc_CurClark_Fdbk = {0};  // αβ轴电流反馈
static Clark_t Fov_VolClark_Fdbk = {0};  // αβ轴电压指令
static Park_t Foc_CurPark_Ref = {0};     // dq轴电流参考
static Park_t Foc_CurPark_Fdbk = {0};    // dq轴电流反馈
static Park_t Foc_VolPark_Ref = {0};     // dq轴电压参考

// 速度环和电流环结构体
static SpdLoop_t Foc_SpdLoop = {0};  // 速度环控制
static CurLoop_t Foc_CurLoop = {0};  // 电流环控制

// VF和IF控制结构体
static VF_Parameter_t Foc_VF_Param = {0};  // VF模式参数
static IF_Parameter_t Foc_IF_Param = {0};  // IF模式参数

// PID控制器
static PID_Handler_t Foc_PidSpeed = {0};  // 速度环PID
static PID_Handler_t Foc_PidCurD = {0};   // d轴电流PID
static PID_Handler_t Foc_PidCurQ = {0};   // q轴电流PID

// 斜坡发生器
static RampGenerator_t FOC_RampSpd = {0};  // 速度斜坡发生器

static inline float Get_Theta(float, float, float);
static inline void SVPWM_Generate(Clark_t*, float, Phase_t*);
static inline void Speed_Loop_Control(SpdLoop_t*, Park_t*);
static inline void Current_Loop_Control(CurLoop_t*, Park_t*);

// SECTION - FOC Init
bool FOC_Init(FOC_Parameter_t* foc)
{
  if (!foc)
    return false;

  /* 基础参数初始化 */
  foc->initialized = false;
  foc->Stop = true; /* 初始状态为停止 */
  foc->Mode = IDLE; /* 初始模式为空闲 */
  foc->SpeedRef = 0.0f;
  foc->SpeedFdbk = 0.0f;
  foc->I_Max = 10.0f; /* 默认最大电流 */
  foc->Udc = 0.0f;
  foc->inv_Udc = 0.0f;
  foc->Theta = 0.0f;
  foc->PWM_ARR = 4200.0f; /* PWM周期，根据实际情况调整 */

  /* 连接指针到全局变量 */
  foc->Iabc_fdbk = &Foc_CurPhase_Fdbk;
  foc->Tcm = &Foc_Pwm_Tcm;
  foc->Iclark_fdbk = &Foc_CurClark_Fdbk;
  foc->Uclark_ref = &Fov_VolClark_Fdbk;
  foc->Idq_ref = &Foc_CurPark_Ref;
  foc->Idq_fdbk = &Foc_CurPark_Fdbk;
  foc->Udq_ref = &Foc_VolPark_Ref;
  foc->Hnd_spdloop = &Foc_SpdLoop;
  foc->Hnd_curloop = &Foc_CurLoop;
  foc->Hnd_vf = &Foc_VF_Param;
  foc->Hnd_if = &Foc_IF_Param;

  /* 速度环初始化 */
  Foc_SpdLoop.ref = 0.0f;
  Foc_SpdLoop.fdbk = 0.0f;
  Foc_SpdLoop.reset = true;
  Foc_SpdLoop.prescaler = 10; /* 使用SPEED_LOOP_PRESCALER值 */
  Foc_SpdLoop.counter = 0;
  Foc_SpdLoop.hnd_ramp = &FOC_RampSpd;
  Foc_SpdLoop.hnd_speed = &Foc_PidSpeed;

  /* 电流环初始化 */
  Foc_CurLoop.ref = &Foc_CurPark_Ref;
  Foc_CurLoop.fdbk = &Foc_CurPark_Fdbk;
  Foc_CurLoop.reset = true;
  Foc_CurLoop.handler_d = &Foc_PidCurD;
  Foc_CurLoop.handler_q = &Foc_PidCurQ;

  /* PID控制器基础初始化 */
  // 速度PID初始化
  Foc_PidSpeed.Kp = 1.0f;
  Foc_PidSpeed.Ki = 0.1f;
  Foc_PidSpeed.Kd = 0.0f;
  Foc_PidSpeed.integral = 0.0f;
  Foc_PidSpeed.previous_error = 0.0f;
  Foc_PidSpeed.MaxOutput = 10.0f;
  Foc_PidSpeed.MinOutput = -10.0f;
  Foc_PidSpeed.output = 0.0f;
  Foc_PidSpeed.IntegralLimit = 5.0f;
  Foc_PidSpeed.Ts = 0.001f; /* 1ms，稍后会更新 */
  Foc_PidSpeed.Reset = false;

  // Id PID初始化
  Foc_PidCurD.Kp = 0.5f;
  Foc_PidCurD.Ki = 50.0f;
  Foc_PidCurD.Kd = 0.0f;
  Foc_PidCurD.integral = 0.0f;
  Foc_PidCurD.previous_error = 0.0f;
  Foc_PidCurD.MaxOutput = 24.0f; /* 电压限制 */
  Foc_PidCurD.MinOutput = -24.0f;
  Foc_PidCurD.output = 0.0f;
  Foc_PidCurD.IntegralLimit = 10.0f;
  Foc_PidCurD.Ts = 0.0001f; /* 100us，稍后会更新 */
  Foc_PidCurD.Reset = false;

  // Iq PID初始化
  Foc_PidCurQ.Kp = 0.5f;
  Foc_PidCurQ.Ki = 50.0f;
  Foc_PidCurQ.Kd = 0.0f;
  Foc_PidCurQ.integral = 0.0f;
  Foc_PidCurQ.previous_error = 0.0f;
  Foc_PidCurQ.MaxOutput = 24.0f; /* 电压限制 */
  Foc_PidCurQ.MinOutput = -24.0f;
  Foc_PidCurQ.output = 0.0f;
  Foc_PidCurQ.IntegralLimit = 10.0f;
  Foc_PidCurQ.Ts = 0.0001f; /* 100us，稍后会更新 */
  Foc_PidCurQ.Reset = false;

  /* VF参数初始化 */
  Foc_VF_Param.Vref_Ud = 0.0f;
  Foc_VF_Param.Vref_Uq = 0.0f;
  Foc_VF_Param.Freq = 0.0f;
  Foc_VF_Param.Theta = 0.0f;

  /* IF参数初始化 */
  Foc_IF_Param.Id_Ref = 0.0f;
  Foc_IF_Param.Iq_Ref = 0.0f;
  Foc_IF_Param.IF_Freq = 0.0f;
  Foc_IF_Param.Theta = 0.0f;
  Foc_IF_Param.Sensor_State = Disable;

  /* 斜坡发生器初始化 */
  FOC_RampSpd.value = 0.0f;
  FOC_RampSpd.target = 0.0f;
  FOC_RampSpd.slope = 100.0f;       /* 默认斜坡率 */
  FOC_RampSpd.limit_min = -3000.0f; /* 最小速度 */
  FOC_RampSpd.limit_max = 3000.0f;  /* 最大速度 */
  FOC_RampSpd.Ts = 0.0001f;         /* 采样周期，稍后会更新 */
  return true;
}

// SECTION - FOC Main
void FOC_Main(FOC_Parameter_t* foc)
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
      hnd_vf->Theta = Get_Theta(hnd_vf->Freq, hnd_vf->Theta, foc->Ts);
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
        hnd_if->Theta = Get_Theta(hnd_if->IF_Freq, hnd_if->Theta, foc->Ts);
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

static inline float Get_Theta(float Freq, float Theta, float Ts)
{
  // 电角度递推：θ += ω·Ts，ω = 2π·f
  Theta += M_2PI * Freq * Ts;
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

/* ================ FOC参数更新函数实现 ================ */

void FOC_UpdateThetaAndSpeed(FOC_Parameter_t* foc, const Motor_Parameter_t* motor)
{
  if (!foc || !motor)
    return;

  /* 更新角度和速度反馈 */
  foc->Theta = motor->Elec_Theta;
  foc->SpeedFdbk = motor->Speed;
}

void FOC_UpdateCurrentFeedback(FOC_Parameter_t* foc, const Phase_t* current_phase)
{
  if (!foc || !current_phase || !foc->Iabc_fdbk)
    return;

  /* 更新三相电流反馈 */
  foc->Iabc_fdbk->a = current_phase->a;
  foc->Iabc_fdbk->b = current_phase->b;
  foc->Iabc_fdbk->c = current_phase->c;
}

void FOC_UpdateVoltageFeedback(FOC_Parameter_t* foc, float udc, float inv_udc)
{
  if (!foc)
    return;

  /* 更新直流母线电压反馈 */
  foc->Udc = udc;
  foc->inv_Udc = inv_udc;
}

bool FOC_GetPWMOutput(const FOC_Parameter_t* foc, Phase_t* tcm_output)
{
  if (!foc || !tcm_output || !foc->Tcm)
    return false;

  /* 获取PWM输出 */
  tcm_output->a = foc->Tcm->a;
  tcm_output->b = foc->Tcm->b;
  tcm_output->c = foc->Tcm->c;

  return true;
}

void FOC_UpdateFrequencyParams(FOC_Parameter_t* foc, float freq, float ts, float pwm_arr)
{
  if (!foc)
    return;

  /* 更新频率参数 */
  foc->freq = freq;
  foc->Ts = ts;
  foc->PWM_ARR = pwm_arr;
}

void FOC_SetMode(FOC_Parameter_t* foc, FOC_Mode_t mode)
{
  if (!foc)
    return;

  foc->Mode = mode;
}

void FOC_SetStopFlag(FOC_Parameter_t* foc, bool stop)
{
  if (!foc)
    return;

  foc->Stop = stop;
}

void FOC_SetSpeedReference(FOC_Parameter_t* foc, float speed_ref)
{
  if (!foc)
    return;

  foc->SpeedRef = speed_ref;
}
