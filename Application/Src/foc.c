#include "foc.h"
#include "foc_types.h"
#include "pid.h"
#include "signal.h"
#include "theta_calc.h"
#include "transformation.h"

#include "hardware_interface.h"
#include "position_sensor.h"

static FOC_Mode_t      Foc_Mode               = IDLE;   // 当前FOC模式
static bool            Foc_Reset              = false;  // FOC复位标志
static float           Foc_Current_Ts         = 0.0F;   // 电流环采样周期
static float           Foc_Current_Freq       = 0.0F;   // 电流环频率
static uint16_t        Foc_Speed_Prescaler    = 0;      // 电流环分频数
static float           Foc_Speed_Ts           = 0.0F;   // 转速环采样周期
static float           Foc_Speed_Freq         = 0.0F;   // 转速环频率
static float           Foc_Speed_Ref          = 0.0F;   // 参考速度
static float           Foc_Speed_Ramp         = 0.0F;   // 实际指令转速
static float           Foc_Speed_Fdbk         = 0.0F;   // 实际转速反馈
static float           Foc_Theta              = 0.0F;   // 当前电机角度（弧度）
static VF_Parameter_t  Foc_VF_Param           = {0};
static IF_Parameter_t  Foc_IF_Param           = {0};
static Clark_t         Foc_Iclark_Fdbk        = {0};
static Park_t          Foc_Idq_Ref            = {0};
static Park_t          Foc_Idq_Fdbk           = {0};
static Clark_t         Foc_Uclark_Ref         = {0};
static Park_t          Foc_Udq_Ref            = {0};
static PID_Handler_t   Foc_Pid_Speed_Handler  = {0};
static PID_Handler_t   Foc_Pid_CurD_Handler   = {0};
static PID_Handler_t   Foc_Pid_CurQ_Handler   = {0};
static RampGenerator_t Foc_Ramp_Speed_Handler = {0};
static SineWave_t      Foc_SineWave_Handler   = {0};

void Foc_Set_SampleTime(SystemTimeConfig_t* config) {
    Foc_Current_Ts      = config->current.time;  // 电流环采样周期
    Foc_Current_Freq    = config->current.freq;  // 电流环频率
    Foc_Speed_Ts        = config->speed.time;    // 转速环采样周期
    Foc_Speed_Freq      = config->speed.freq;    // 转速环频率
    Foc_Speed_Prescaler = config->prescaler;     // 转速环分频数
}

void Foc_Set_State(bool reset) {
    Foc_Reset = reset;  // 设置复位标志
}

bool Foc_Get_State(void) {
    return Foc_Reset;  // 获取复位标志状态
}

void Foc_Set_Angle(float angle) {
    Foc_Theta = wrap_theta_2pi(angle);  // 确保角度在 [0, 2π) 范围内
}

void Foc_Set_Speed(float speed) {
    Foc_Speed_Fdbk = speed;  // 设置参考速度
}

void Foc_Set_Speed_and_Angle(AngleResult_t* angle_speed) {
    Foc_Set_Speed(angle_speed->speed);
    Foc_Set_Angle(angle_speed->theta);  // 设置角度
}

void Foc_Set_Iclark_Fdbk(Clark_t* current) {
    Foc_Iclark_Fdbk = *current;  // 设置电流反馈
}

Clark_t Foc_Get_Iclark_Fdbk(void) {
    return Foc_Iclark_Fdbk;  // 获取αβ轴电流反馈
}

void Foc_Set_Idq_Ref(Park_t* idq_ref) {
    Foc_Idq_Ref = *idq_ref;  // 设置DQ轴电流参考
}

Park_t Foc_Get_Idq_Ref(void) {
    return Foc_Idq_Ref;  // 获取DQ轴电流参考
}

void Foc_Set_Idq_Fdbk(Park_t* idq_fdbk) {
    Foc_Idq_Fdbk = *idq_fdbk;  // 设置DQ轴电流反馈
}

Park_t Foc_Get_Idq_Fdbk(void) {
    return Foc_Idq_Fdbk;  // 获取DQ轴电流反馈
}

void Foc_Set_Udq_Ref(Park_t* udq_ref) {
    Foc_Udq_Ref = *udq_ref;  // 设置DQ轴电压参考
}

Park_t Foc_Get_Udq_Ref(void) {
    return Foc_Udq_Ref;  // 获取DQ轴电压参考
}

void Foc_Set_Uclark_Ref(Clark_t* uclark_ref) {
    Foc_Uclark_Ref = *uclark_ref;  // 设置αβ轴电压参考
}

Clark_t Foc_Get_Uclark_Ref(void) {
    return Foc_Uclark_Ref;  // 获取αβ轴电压参考
}

void Foc_Set_Vf_Param(VF_Parameter_t* vf_param) {
    Foc_VF_Param = *vf_param;  // 设置VF参数
}

void Foc_Set_If_Param(IF_Parameter_t* if_param) {
    Foc_IF_Param = *if_param;  // 设置IF参数
}

void Foc_Set_Pid_Speed_Handler(PID_Handler_t* handler) {
    Foc_Pid_Speed_Handler = *handler;  // 设置速度环PID控制器
}

void Foc_Set_Pid_CurD_Handler(PID_Handler_t* handler) {
    Foc_Pid_CurD_Handler = *handler;  // 设置D轴电流环PID控制器
}

void Foc_Set_Pid_CurQ_Handler(PID_Handler_t* handler) {
    Foc_Pid_CurQ_Handler = *handler;  // 设置Q轴电流环PID控制器
}

void Foc_Set_Ramp_Speed_Handler(RampGenerator_t* handler) {
    Foc_Ramp_Speed_Handler = *handler;  // 设置速度环斜坡生成器
}

static inline Park_t Foc_SpeedLoop_Update(float ref, float fdbk, bool reset) {
    static uint16_t counter = 0;
    counter++;
    if (counter < Foc_Speed_Prescaler) {
        return Foc_Idq_Ref;  // 如果未到达分频点，直接返回参考值
    }
    counter                       = 0;
    Foc_Ramp_Speed_Handler.target = ref;  // 更新目标速度
    Park_t output                 = {0};
    float  ramp    = RampGenerator(&Foc_Ramp_Speed_Handler, reset);
    Foc_Speed_Ramp = ramp;
    output.q       = Pid_Update(ramp - fdbk, reset, &Foc_Pid_Speed_Handler);
    output.d       = 0.0F;  // D轴电流参考为0

    return output;  // 返回DQ轴电流参考
}

static inline Park_t Foc_CurrentLoop_Update(Park_t* ref,
                                            Park_t* fdbk,
                                            bool    reset) {
    Park_t output = {0};

    Pid_Update(ref->d - fdbk->d, reset, &Foc_Pid_CurD_Handler);
    Pid_Update(ref->q - fdbk->q, reset, &Foc_Pid_CurQ_Handler);

    output.d = Foc_Pid_CurD_Handler.output;
    output.q = Foc_Pid_CurQ_Handler.output;

    return output;
}

static inline Park_t Foc_VfMode_Update(bool reset) {
    static bool reset_prev = true;
    Park_t      output     = {0};
    if (reset_prev && !reset) {
        SineWave_Init(&Foc_SineWave_Handler,
                      1.0F,
                      Foc_VF_Param.Freq,
                      Foc_VF_Param.Theta,
                      Foc_Current_Ts);
    }
    output.d = Foc_VF_Param.Vref_Ud;
    output.q = Foc_VF_Param.Vref_Uq;
    Foc_VF_Param.Theta
        = SineWaveGenerator(&Foc_SineWave_Handler, reset);  // 更新电压环角度

    reset_prev = reset;
    return output;
}

static inline Park_t Foc_IfMode_Update(bool reset) {
    static bool reset_prev = true;
    Park_t      output     = {0};
    // 如果传感器状态为启用，则直接使用参考值，否则使用正弦波生成器
    if (Foc_IF_Param.Sensor_State == Enable) {
        Foc_VF_Param.Theta = Foc_VF_Param.Theta;
    } else {
        if (reset_prev && !reset) {
            SineWave_Init(&Foc_SineWave_Handler,
                          1.0F,
                          Foc_IF_Param.IF_Freq,
                          Foc_IF_Param.Theta,
                          Foc_Current_Ts);
        }
        Foc_VF_Param.Theta = SineWaveGenerator(&Foc_SineWave_Handler, reset);
    }

    output.d = Foc_IF_Param.Id_Ref;
    output.q = Foc_IF_Param.Iq_Ref;

    return output;
}

static inline Park_t Foc_SpeedMode_Update(bool reset) {
    Park_t output = {0};
    // 更新转速环
    Foc_Idq_Ref = Foc_SpeedLoop_Update(Foc_Speed_Ref, Foc_Speed_Fdbk, reset);
    // 更新电流环
    output = Foc_CurrentLoop_Update(&Foc_Idq_Ref, &Foc_Idq_Fdbk, reset);

    return output;
}

Park_t Foc_Calc_Udq_Ref() {
    Park_t output = {0};

    switch (Foc_Mode) {
    case VF_MODE: {
        output = Foc_VfMode_Update(Foc_Reset);  // VF模式
        break;
    }
    case IF_MODE: {
        output = Foc_IfMode_Update(Foc_Reset);  // IF模式
        break;
    }
    case Speed: {
        output = Foc_SpeedMode_Update(Foc_Reset);  // 转速模式
        break;
    }
    default: {
        output.d = 0.0F;  // 默认情况下，D轴电压参考为0
        output.q = 0.0F;  // Q轴电压参考为0
        break;
    }
    }

    return output;  // 返回DQ轴电压参考
}

static inline void Speed_Loop_Control(SpdLoop_t* speed_loop, Park_t* idq_ref);
static inline void Current_Loop_Control(CurLoop_t* hnd, Park_t* out);

// SECTION - FOC Main
void FOC_Update(FOC_Parameter_t* foc) {
    switch (foc->Mode) {
    case IDLE: {
        foc->Stop = 1;
        break;
    }
    case VF_MODE: {
        VF_Parameter_t* hnd_vf   = foc->Hnd_vf;
        SawtoothWave_t* sawtooth = hnd_vf->hnd_sawtooth;
        if (!sawtooth->initialized) {
            SawtoothWave_Init(sawtooth, M_2PI, hnd_vf->Freq, 0.0F, foc->Ts);
            sawtooth->initialized = true;
        }
        sawtooth->frequency = hnd_vf->Freq;
        hnd_vf->Theta       = SawtoothWaveGenerator(sawtooth, foc->Stop);
        foc->Theta          = hnd_vf->Theta;
        ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_ref);
        foc->Udq_ref->d = hnd_vf->Vref_Ud;
        foc->Udq_ref->q = hnd_vf->Vref_Uq;
        break;
    }
    // SECTION - IF Mode
    case IF_MODE: {
        IF_Parameter_t* hnd_if = foc->Hnd_if;
        if (hnd_if->Sensor_State == Enable) {
            hnd_if->Theta = foc->Theta;
        } else {
            SawtoothWave_t* sawtooth = hnd_if->hnd_sawtooth;
            if (!sawtooth->initialized) {
                SawtoothWave_Init(
                    sawtooth, M_2PI, hnd_if->IF_Freq, 0.0F, foc->Ts);
                sawtooth->initialized = true;
            }
            sawtooth->frequency = hnd_if->IF_Freq;
            // 生成锯齿波角度
            hnd_if->Theta = SawtoothWaveGenerator(sawtooth, foc->Stop);
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
    case Speed: {
        ParkTransform(foc->Iclark_fdbk, foc->Theta, foc->Idq_fdbk);

        SpdLoop_t* hSpeed = foc->Hnd_spdloop;
        hSpeed->ref       = foc->SpeedRef;   // 更新目标速度
        hSpeed->fdbk      = foc->SpeedFdbk;  // 更新实际速度
        hSpeed->reset     = foc->Stop;       // 更新转速环的停止标志

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
    case SHUTDOWN: {
        foc->Stop        = 1;
        foc->Idq_fdbk->d = 0.0F;  // Id_Ref = 0
        foc->Idq_fdbk->q = 0.0F;  // Iq_Ref = Pid_SpdLoop.output
        foc->Udq_ref->d  = 0.0F;
        foc->Udq_ref->q  = 0.0F;
        break;
    }
    default: {
        foc->Stop = 1;
        foc->Mode = IDLE;
        break;
    }
    }

    InvParkTransform(foc->Udq_ref, foc->Theta, foc->Uclark_ref);
}

// SECTION - Speed Loop Control
static inline void Speed_Loop_Control(SpdLoop_t* hnd, Park_t* out) {
    hnd->counter++;
    if (hnd->counter >= hnd->prescaler) {
        hnd->counter          = 0;
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
static inline void Current_Loop_Control(CurLoop_t* hnd, Park_t* out) {
    // D轴电流控制
    if (hnd->handler_d->Reset) {
        PID_SetIntegral(hnd->handler_d, hnd->reset, 0.0F);
    }
    Pid_Update(hnd->ref->d - hnd->fdbk->d, hnd->reset, hnd->handler_d);

    // Q轴电流控制
    if (hnd->handler_q->Reset) {
        PID_SetIntegral(hnd->handler_q, hnd->reset, 0.0F);
    }
    Pid_Update(hnd->ref->q - hnd->fdbk->q, hnd->reset, hnd->handler_q);

    // 输出DQ轴电压指令
    out->d = hnd->handler_d->output;
    out->q = hnd->handler_q->output;
}
