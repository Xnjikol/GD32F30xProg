#include "main_int.h"

#include <stdlib.h>

float theta_mech = 0.0F;
float theta_elec = 0.0F;

float Speed_Ref  = 0.0F;
float Speed_Fdbk = 0.0F;

DeviceState_t     Device;
Motor_Parameter_t Motor;
FOC_Parameter_t   FOC;

// Sensorless control global variables
Sensorless_Parameter_t Sensorless;
sensorless_config_t    SensorlessConfig;
sensorless_output_t    SensorlessOutput;

VF_Parameter_t  OpenLoop_VF;
IF_Parameter_t  OpenLoop_IF;
PID_Handler_t   Pid_CurLoop_d;
PID_Handler_t   Pid_CurLoop_q;
PID_Handler_t   Pid_SpdLoop;
RampGenerator_t Rmp_Speed;
Clark_t         VoltageClark_Ref;
Clark_t         CurrentClark_Fdbk;
Phase_t         CurrentPhase_Fdbk;
Phase_t         TcmPhase_Ref;
Park_t          CurrentPark_Fdbk;
Park_t          CurrentPark_Ref;
Park_t          VoltagePark_Ref;

static inline void Main_Int_Parameter_Init(void);
static inline void Main_Int_Basic_Init(void);
static inline void UpdateThetaAndSpeed(FOC_Parameter_t*   foc,
                                       Motor_Parameter_t* motor);
static inline void SVPWM_Generate(Clark_t*, float, Phase_t*);

/*!
    \brief      主中断函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Handler(void) {
    if (adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOIC)) {
        adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

        // 基础外设更新（无论初始化状态如何都要执行）
        Peripheral_UpdateCurrent();
        Peripheral_GateState();
        Peripheral_UpdateUdc();
        Peripheral_UpdatePosition(&Motor);

        switch (Device.Mode) {
        case INIT: {
            // 基础初始化：仅获取系统参数，不进行完整的FOC初始化
            Main_Int_Basic_Init();
            Sensorless_Init(&Sensorless,
                            &SensorlessConfig,
                            &SensorlessOutput,
                            &Motor,
                            &Device);
            if (Device.system_params_valid) {
                Device.basic_init_done = true;
                Device.Mode            = WAITING;
                FOC.Mode               = IDLE;
            }
            break;
        }

        case WAITING: {
            // 基础就绪状态：系统参数已获取，等待完整初始化触发
            // 继续运行基本逻辑但不进行FOC控制
            if (FOC.Udc > 200.0F) {
                Peripheral_EnableHardwareProtect();
            }

            // 检查是否触发完整初始化（通过CCP协议修改Device.Mode）
            // 这里不做状态转换，由外部CCP协议触发
            FOC.Mode = IDLE;
            break;
        }

        case SETUP: {
            // 完整初始化：用户触发的完整参数初始化
            if (FOC.Udc > 200.0F) {
                Main_Int_Parameter_Init();
                Peripheral_EnableHardwareProtect();
            }
            Protect.Flag          = No_Protect;
            Device.full_init_done = true;
            Device.Mode           = READY;
            FOC.Mode              = IDLE;
            break;
        }

        case READY: {
            // 就绪状态：准备进入运行状态
            FOC.Mode    = IDLE;
            Device.Mode = RUNNING;
            break;
        }

        case RUNNING: {
            // 运行状态：正常FOC控制
            UpdateThetaAndSpeed(&FOC, &Motor);
            FOC.Stop     = Device_Stop;
            FOC.SpeedRef = Speed_Ref;
            FOC_Update(&FOC);
            SVPWM_Generate(FOC.Uclark_ref, FOC.inv_Udc, FOC.Tcm);
            break;
        }

        case SHUTDOWN: {
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
void Main_Int_Basic_Init(void) {
    static bool basic_init_started = false;

    if (!basic_init_started) {
        // 初始化基础保护参数
        Peripheral_InitProtectParameter();

        // 获取系统频率 - 这是关键步骤
        Peripheral_GetSystemFrequency();

        // 标记基础初始化已开始
        basic_init_started = true;
    }

    // 检查系统参数是否已获取到有效值
    if (Device.main_Ts > 0.0F && Device.main_Freq > 0.0F) {
        Device.system_params_valid = true;
    }
}

void Main_Int_Parameter_Init(void) {
    // 注意：基础系统参数（Ts,
    // freq等）已在Main_Int_Basic_Init中获取，这里不再重复

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

    // 设置 Motor 结构体中的指针，指向 Device 中的时间和频率变量
    Motor.main_Ts_ptr    = &Device.main_Ts;
    Motor.main_Freq_ptr  = &Device.main_Freq;
    Motor.speed_Ts_ptr   = &Device.speed_Ts;
    Motor.speed_Freq_ptr = &Device.speed_Freq;

    // 由于基础初始化已完成，这里只需要校准ADC
    Peripheral_InitProtectParameter();  // 已在基础初始化中完成
    Peripheral_GetSystemFrequency();    // 已在基础初始化中完成
    Peripheral_CalibrateADC();

    FOC.Iabc_fdbk   = &CurrentPhase_Fdbk;
    FOC.Iclark_fdbk = &CurrentClark_Fdbk;
    FOC.Idq_ref     = &CurrentPark_Ref;
    FOC.Idq_fdbk    = &CurrentPark_Fdbk;
    FOC.Tcm         = &TcmPhase_Ref;

    // 开环相关变量
    FOC.Hnd_vf = &OpenLoop_VF;
    FOC.Hnd_if = &OpenLoop_IF;

    // 电流环相关变量
    FOC.Hnd_curloop            = (CurLoop_t*) calloc(1, sizeof(CurLoop_t));
    FOC.Hnd_curloop->fdbk      = (Park_t*) calloc(1, sizeof(Park_t));
    FOC.Hnd_curloop->ref       = (Park_t*) calloc(1, sizeof(Park_t));
    FOC.Hnd_curloop->handler_d = &Pid_CurLoop_d;
    FOC.Hnd_curloop->handler_q = &Pid_CurLoop_q;
    FOC.Hnd_curloop->reset     = 0;  // 初始化停止标志

    // 转速环相关变量
    FOC.Hnd_spdloop            = (SpdLoop_t*) calloc(1, sizeof(SpdLoop_t));
    FOC.Hnd_spdloop->hnd_speed = &Pid_SpdLoop;
    FOC.Hnd_spdloop->hnd_ramp  = &Rmp_Speed;
    FOC.Hnd_spdloop->reset     = 0;  // 初始化停止标志
    FOC.Hnd_spdloop->prescaler = (uint16_t) SPEED_LOOP_PRESCALER;  // 设置分频数
    FOC.Hnd_spdloop->counter   = 0;  // 初始化分频计数器

    FOC.Udq_ref    = &VoltagePark_Ref;
    FOC.Uclark_ref = &VoltageClark_Ref;

    Motor.Rs              = 1.25F;
    Motor.Ld              = 0.006F;
    Motor.Lq              = 0.009F;
    Motor.Flux            = 0.1F;
    Motor.Pn              = 5.0F;
    Motor.Position_Scale  = 65536 - 1;
    Motor.Resolver_Pn     = 1.0F;
    Motor.inv_MotorPn     = 1.0F / 2.0F;  // Pn
    Motor.Position_Offset = 39833.000000F;

    // 设置theta_factor
#ifdef Resolver_Position
    Motor.theta_factor
        = M_2PI / ((Motor.Position_Scale + 1) * Motor.Resolver_Pn);
#endif
#ifdef Encoder_Position
    Motor.theta_factor = M_2PI / (float) (Motor.Position_Scale + 1);
#endif
    Rmp_Speed.slope     = 50.0F;  // limit to 50 rpm/s
    Rmp_Speed.limit_min = -1800.0F;
    Rmp_Speed.limit_max = 1800.0F;
    Rmp_Speed.value     = 0.0F;
    Rmp_Speed.target    = 0.0F;
    Rmp_Speed.Ts        = Device.speed_Ts;  // Speed loop time;

    Pid_SpdLoop.Kp             = 0.005F;
    Pid_SpdLoop.Ki             = 0.03F;
    Pid_SpdLoop.Kd             = 0.0F;
    Pid_SpdLoop.MaxOutput      = 0.7F * FOC.I_Max;  // Maximum Iq
    Pid_SpdLoop.MinOutput      = -0.7F * FOC.I_Max;
    Pid_SpdLoop.IntegralLimit  = 0.7F * FOC.I_Max;
    Pid_SpdLoop.previous_error = 0.0F;
    Pid_SpdLoop.integral       = 0.0F;
    Pid_SpdLoop.output         = 0.0F;
    Pid_SpdLoop.Ts             = Device.speed_Ts;  // Speed loop time;

    Pid_CurLoop_d.Kp             = 73.8274273F;
    Pid_CurLoop_d.Ki             = 408.40704496F;
    Pid_CurLoop_d.Kd             = 0.0F;
    Pid_CurLoop_d.MaxOutput      = 50.0F;  // Maximum Udc/sqrt(3)
    Pid_CurLoop_d.MinOutput      = -50.0F;
    Pid_CurLoop_d.IntegralLimit  = 50.0F;
    Pid_CurLoop_d.previous_error = 0.0F;
    Pid_CurLoop_d.integral       = 0.0F;
    Pid_CurLoop_d.output         = 0.0F;
    Pid_CurLoop_d.Ts             = Device.main_Ts;  // Current loop time
    Pid_CurLoop_d.Reset          = true;

    Pid_CurLoop_q.Kp             = 27.646015F;
    Pid_CurLoop_q.Ki             = 408.40704496F;
    Pid_CurLoop_q.Kd             = 0.0F;
    Pid_CurLoop_q.MaxOutput      = 50.0F;
    Pid_CurLoop_q.MinOutput      = -50.0F;
    Pid_CurLoop_q.IntegralLimit  = 50.0F;
    Pid_CurLoop_q.previous_error = 0.0F;
    Pid_CurLoop_q.integral       = 0.0F;
    Pid_CurLoop_q.output         = 0.0F;
    Pid_CurLoop_q.Ts             = Device.main_Ts;  // Current loop time
    Pid_CurLoop_q.Reset          = true;

    OpenLoop_VF.Vref_Ud = 0.0F;
    OpenLoop_VF.Vref_Uq = 0.0F;
    OpenLoop_VF.Freq    = 0.0F;
    OpenLoop_VF.Theta   = 0.0F;
    OpenLoop_VF.hnd_sawtooth
        = (SawtoothWave_t*) calloc(1, sizeof(SawtoothWave_t));

    OpenLoop_IF.Id_Ref       = 0.0F;
    OpenLoop_IF.Iq_Ref       = 0.0F;
    OpenLoop_IF.IF_Freq      = 0.0F;
    OpenLoop_IF.Theta        = 0.0F;
    OpenLoop_IF.Sensor_State = Disable;
    OpenLoop_IF.hnd_sawtooth
        = (SawtoothWave_t*) calloc(1, sizeof(SawtoothWave_t));
}

void FOC_UpdateMainFrequency(float freq, float Ts, float PWM_ARR) {
    // 更新主中断相关参数
    Device.main_Freq = freq;
    Device.main_Ts   = Ts;

    // 计算转速环相关参数
    Device.speed_Freq = freq / SPEED_LOOP_PRESCALER;
    Device.speed_Ts   = Ts * SPEED_LOOP_PRESCALER;

    // 同时更新FOC结构体中的参数（为了兼容性）
    FOC.freq    = freq;
    FOC.Ts      = Ts;
    FOC.PWM_ARR = PWM_ARR;
}

static inline void SVPWM_Generate(Clark_t* u_ref, float inv_Vdc, Phase_t* out) {
    float   alpha  = u_ref->a;
    float   beta   = u_ref->b;
    uint8_t sector = 0;
    float   v_ref1 = beta;
    float   v_ref2 = (+SQRT3 * alpha - beta) * 0.5F;
    float   v_ref3 = (-SQRT3 * alpha - beta) * 0.5F;

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

    switch (sector) {
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
    if (T_sum > 1.0F) {
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
    switch (sector) {
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

static inline void UpdateThetaAndSpeed(FOC_Parameter_t*   foc,
                                       Motor_Parameter_t* motor) {
    // theta 和 speed 现在由 Peripheral_UpdatePosition 直接更新
    foc->Theta     = motor->Elec_Theta;
    foc->SpeedFdbk = motor->Speed;
}