#include "foc.h"
#include "math_utils.h"
#include "filter.h"     // 添加滤波器模块支持
#include "sensorless.h" // 添加无位置传感器支持

uint16_t STOP = 1;

volatile Protect_Flags_t Protect_Flag;
Motor_Parameter_t Motor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
PID_Controller_t Speed_PID;
RampGenerator_t Speed_Ramp;
InvPark_t Inv_Park;
Clarke_t Clarke;
Park_t Park;

uint16_t auto_adjust = 0;

float I_Max = 10.0f;
float theta_mech = 0.0f;
float theta_elec = 0.0f;
float theta_factor = 0.0f; // Sensor data to mechanic angle conversion factor
float theta_error = 0.0f;  // 位置误差

float Speed_Ref = 0.0f;

float View_Angle = 0.0f;
float View_Speed = 0.0f;

// 无位置传感器相关变量
extern Sensorless_t Sensorless;
float Sensorless_Theta = 0.0f;
float Sensorless_Speed = 0.0f;

static inline void Current_Protect(void);
static inline float Get_Theta(float Freq, float Theta);
static inline void Parameter_Init(void);
static inline void Theta_Process(void);
static inline float RampGenerator(RampGenerator_t *ramp);
static inline void Speed_Loop(RampGenerator_t *Ramp,
                              float Ref, float Fdbk,
                              PID_Controller_t *Handle);

extern uint16_t receive;
// SECTION - FOC Main
void FOC_Main(void)
{
    ADC_Read_Injection();
    Current_Protect();
    Gate_state();
    ReadPosition();
    Theta_Process();

    ClarkeTransform(Ia, Ib, Ic, &Clarke);

    switch (FOC.Mode)
    {
    case INIT:
    {
        Parameter_Init();

        /* 初始化十二脉冲算法 */
        TwelvePulse_Init(&Sensorless.pulse);

        if (Udc > 200.0f) // 200V
        {
            timer_interrupt_enable(TIMER0, TIMER_INT_BRK); // 启用BRK中断
            ADC_Calibration();
        }
        FOC.Mode = IDLE;
        break;
    }
    case IDLE:
    {
        STOP = 1;
        break;
    }
    case VF_MODE:
    {
        VF.Theta = Get_Theta(VF.Freq, VF.Theta);
        FOC.Theta = VF.Theta;
        FOC.Ud_ref = VF.Vref_Ud;
        FOC.Uq_ref = VF.Vref_Uq;
        break;
    }
    // SECTION - IF Mode
    case IF_MODE:
    {
        if (IF.Sensor_State == ENABLE)
        {
            IF.Theta = FOC.Theta;
        }
        else
        {
            IF.Theta = Get_Theta(IF.IF_Freq, IF.Theta);
        }
        FOC.Theta = IF.Theta;

        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);
        FOC.Id_ref = IF.Id_ref;
        FOC.Iq_ref = IF.Iq_ref;

        PID_Controller(FOC.Id_ref, Park.Id, &Id_PID);
        PID_Controller(FOC.Iq_ref, Park.Iq, &Iq_PID);

        FOC.Ud_ref = Id_PID.output;
        FOC.Uq_ref = Iq_PID.output;

        break;
    }
    // !SECTION
    // SECTION - Speed Mode
    case Speed_Mode:
    {

        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);
        Speed_Loop(&Speed_Ramp, Speed_Ref, FOC.Speed, &Speed_PID);

        FOC.Iq_ref = Speed_PID.output; // Iq_ref = Speed_PID.output
        FOC.Id_ref = 0;                // Id_ref = 0

        PID_Controller(FOC.Id_ref, Park.Id, &Id_PID);
        PID_Controller(FOC.Iq_ref, Park.Iq, &Iq_PID);

        FOC.Ud_ref = Id_PID.output;
        FOC.Uq_ref = Iq_PID.output;

        break;
    }
    // !SECTION
    // SECTION - TwelvePulse Mode
    case Pulse_Mode:
    {
        FOC.Theta = 0.0f; // 重置Theta，确保从零开始

        /* 十二脉冲初始定位模式 */
        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);

        /* 更新十二脉冲算法 */
        TwelvePulse_Update(&Sensorless.pulse, Park.Id, Park.Iq);

        /* 获取测试电压 */
        float ud_test = 0.0f, uq_test = 0.0f;
        TwelvePulse_GetVoltageReference(&Sensorless.pulse, &ud_test, &uq_test);

        /* 设置电压参考 */
        FOC.Ud_ref = ud_test;
        FOC.Uq_ref = uq_test;

        /* 设置电流参考为0（仅电压控制） */
        FOC.Id_ref = 0.0f;
        FOC.Iq_ref = 0.0f;

        /* 如果十二脉冲完成，切换到无位置传感器模式 */
        if (TwelvePulse_IsCompleted(&Sensorless.pulse))
        {
            /* 使用估计的初始位置 */
            FOC.Theta = TwelvePulse_GetTheta(&Sensorless.pulse);

            theta_error = MathUtils_AngleDifference(FOC.Theta, theta_mech);

            /* 切换到无位置传感器模式 */
            // FOC.Mode = NoSensorMode;

            // /* 启用无位置传感器算法 */
            // FOC.SensorlessEnabled = 1;
            // Sensorless.enabled = 1;
            // Sensorless_SetAlgorithm(SENSORLESS_HYBRID);
        }

        break;
    }
    // !SECTION
    // SECTION - Sensorless Mode
    case NoSensorMode:
    {
        /* 无位置传感器模式 */
        FOC_Sensorless_Update();

        /* 使用无位置传感器估计的角度和速度 */
        FOC.Theta = Sensorless_Theta;
        FOC.Speed = Sensorless_Speed;

        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);

        /* 速度控制 */
        Speed_Loop(&Speed_Ramp, Speed_Ref, FOC.Speed, &Speed_PID);

        FOC.Iq_ref = Speed_PID.output;
        FOC.Id_ref = 0.0f; // 无位置传感器通常设Id_ref=0

        PID_Controller(FOC.Id_ref, Park.Id, &Id_PID);
        PID_Controller(FOC.Iq_ref, Park.Iq, &Iq_PID);

        FOC.Ud_ref = Id_PID.output;
        FOC.Uq_ref = Iq_PID.output;

        /* 如果使用高频注入，叠加注入电压 */
        if (Sensorless.algorithm == SENSORLESS_HFI || Sensorless.algorithm == SENSORLESS_HYBRID)
        {
            float ud_inj, uq_inj;
            HFI_GetInjectionVoltage(&Sensorless.hfi, &ud_inj, &uq_inj);
            FOC.Ud_ref += ud_inj;
            FOC.Uq_ref += uq_inj;
        }

        break;
    }
    // !SECTION
    case EXIT:
    {
        STOP = 1;
        timer_interrupt_disable(TIMER0, TIMER_INT_BRK); // 禁用BRK中断
        FOC.Id_ref = 0.0f;                              // Id_ref = 0
        FOC.Iq_ref = 0.0f;                              // Iq_ref = Speed_PID.output
        FOC.Ud_ref = 0.0f;
        FOC.Uq_ref = 0.0f;

        break;
    }
    default:
    {
        STOP = 1;
        FOC.Mode = IDLE;
        break;
    }
    }

    InvParkTransform(FOC.Ud_ref, FOC.Uq_ref, FOC.Theta, &Inv_Park);
    SVPWM_Generate(Inv_Park.Ualpha, Inv_Park.Ubeta, inv_Udc, FOC.PWM_ARR);
}

static inline void Speed_Loop(
    RampGenerator_t *Ramp,
    float Ref,
    float Fdbk,
    PID_Controller_t *Handle)
{
    static uint16_t count = 0;
    count++;
    if (count > 9)
    {
        count = 0;
        Ramp->target = Ref;
        PID_Controller(RampGenerator(Ramp), Fdbk, Handle);
    }
}

void Gate_state(void)
{

    if (Protect_Flag != No_Protect)
    {
        STOP = 1;
    }
    if (STOP)
    {
        // 软件触发 BRK
        Software_BRK = ENABLE;
        TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    }
    else
    {
        // STOP = 0，尝试恢复
        if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
        {
            Software_BRK = DISABLE;
            timer_primary_output_config(TIMER0, ENABLE); // 恢复 MOE
            STOP = 0;
        }
        else
        {
            STOP = 1;
        }
    }
}

void Current_Protect(void)
{
    if ((Ia > 0.9 * I_Max || Ia < -0.9 * I_Max) ||
        (Ib > 0.9 * I_Max || Ib < -0.9 * I_Max) ||
        (Ic > 0.9 * I_Max || Ic < -0.9 * I_Max))
    {
        uint16_t Current_Count = 0;
        Current_Count++;
        if (Current_Count > 10)
        {
            STOP = 1;
            Protect_Flag |= Over_Current;
            Current_Count = 0;
        }
    }
    if ((Ia > I_Max || Ia < -1 * I_Max) || (Ib > I_Max || Ib < -1 * I_Max) ||
        (Ic > I_Max || Ic < -1 * I_Max))
    {
        STOP = 1;
        Protect_Flag |= Over_Maximum_Current;
    }
}

void Temperature_Protect(void)
{
    if (Temperature > 35.0f)
    {
        gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
    }
    if (Temperature > 90.0f)
    {
        STOP = 1;
        Protect_Flag |= Over_Heat;
    }
}

// SECTION - Parameter Init
void Parameter_Init(void)
{
    memset(&VF, 0, sizeof(VF_Parameter_t));
    memset(&FOC, 0, sizeof(FOC_Parameter_t));
    memset(&Id_PID, 0, sizeof(PID_Controller_t));
    memset(&Iq_PID, 0, sizeof(PID_Controller_t));
    memset(&Speed_PID, 0, sizeof(PID_Controller_t));
    memset(&Inv_Park, 0, sizeof(InvPark_t));
    memset(&Motor, 0, sizeof(Motor_Parameter_t));
    memset(&Speed_Ramp, 0, sizeof(RampGenerator_t));

    STOP = 1;
    Protect_Flag = No_Protect;

    Motor.Pn = 5;
    Motor.Resolver_Pn = 1;
    Motor.Position_Scale = UINT16_MAX;
    Motor.inv_MotorPn = 1.0f / Motor.Pn; // 1/Pn
    Motor.Rs = 1.25f;
    Motor.Ld = 0.006f; // 6mH
    Motor.Lq = 0.009f; // 9mH
    Motor.Position_Offset = 13110.0f;
#ifdef Resolver_Position
    theta_factor = 2.0f * M_PI / ((Motor.Position_Scale + 1) * Motor.Resolver_Pn);
#endif
#ifdef Encoder_Position
    theta_factor = 2.0f * M_PI / (Motor.Position_Scale + 1);
#endif
    Speed_PID.Kp = 0.010f;
    Speed_PID.Ki = 0.005f;
    Speed_PID.Kd = 0.0f;
    Speed_PID.MaxOutput = 0.7 * I_Max; // Maximum Iq
    Speed_PID.MinOutput = -0.7 * I_Max;
    Speed_PID.IntegralLimit = 0.7 * I_Max;
    Speed_PID.previous_error = 0.0f;
    Speed_PID.integral = 0.0f;
    Speed_PID.output = 0.0f;
    Speed_PID.Ts = T_2K_HZ;

    Speed_Ramp.slope = 50.0f; // limit to 50 rpm/s
    Speed_Ramp.limit_min = -1800.0f;
    Speed_Ramp.limit_max = 1800.0f;
    Speed_Ramp.value = 0.0f;
    Speed_Ramp.target = 0.0f;
    Speed_Ramp.Ts = T_200_HZ;

    Id_PID.Kp = 7.5398223686f;
    Id_PID.Ki = 1570.796326794f;
    Id_PID.Kd = 0.0f;
    Id_PID.MaxOutput = 50.0f; // Maximum Udc/sqrt(3)
    Id_PID.MinOutput = -50.0f;
    Id_PID.IntegralLimit = 50.0f;
    Id_PID.previous_error = 0.0f;
    Id_PID.integral = 0.0f;
    Id_PID.output = 0.0f;
    Id_PID.Ts = T_2K_HZ;

    Iq_PID.Kp = 11.30973355f;
    Iq_PID.Ki = 1570.796326794f;
    Iq_PID.Kd = 0.0f;
    Iq_PID.MaxOutput = 50.0f;
    Iq_PID.MinOutput = -50.0f;
    Iq_PID.IntegralLimit = 50.0f;
    Iq_PID.previous_error = 0.0f;
    Iq_PID.integral = 0.0f;
    Iq_PID.output = 0.0f;
    Iq_PID.Ts = T_2K_HZ;

    VF.Vref_Ud = 0.0f;
    VF.Vref_Uq = 0.0f;
    VF.Freq = 0.0f;
    VF.Theta = 0.0f;

    IF.Id_ref = 0.0f;
    IF.Iq_ref = 0.0f;
    IF.IF_Freq = 0.0f;
    IF.Theta = 0.0f;
    IF.Sensor_State = DISABLE;

    FOC.PWM_ARR = PWM_ARR;
    FOC.SensorlessEnabled = 0; // 默认禁用无位置传感器
}
// !SECTION

/**
 * @brief 启动十二脉冲初始定位
 */
void FOC_Pulse_Start(void)
{
    /* 初始化十二脉冲算法 */
    TwelvePulse_Init(&Sensorless.pulse);

    /* 启动十二脉冲测试 */
    TwelvePulse_Start(&Sensorless.pulse);

    /* 切换到十二脉冲模式 */
    FOC.Mode = Pulse_Mode;

    /* 禁用保护，准备测试 */
    STOP = 0;

    /* 初始角度设为0 */
    FOC.Theta = 0.0f;
}

/**
 * @brief 无位置传感器初始化
 */
void FOC_Sensorless_Init(void)
{
    /* 初始化无位置传感器算法 */
    Sensorless_Init();

    /* 根据配置选择算法 */
#ifdef SENSORLESS_POSITION
    /* 设置为混合算法，自动根据速度切换 */
    Sensorless_SetAlgorithm(SENSORLESS_HYBRID);
    FOC.SensorlessEnabled = 1;
    Sensorless.enabled = 1;
#else
    FOC.SensorlessEnabled = 0;
    Sensorless.enabled = 0;
#endif
}

/**
 * @brief 无位置传感器更新
 */
void FOC_Sensorless_Update(void)
{
    if (!FOC.SensorlessEnabled)
    {
        return;
    }

    /* 更新无位置传感器算法 */
    extern float Ia, Ib, Ic;
    float ua = 0.0f, ub = 0.0f, uc = 0.0f; // 如果有电压反馈，在此处获取

    Sensorless_Update(Ia, Ib, Ic, ua, ub, uc);

    /* 获取估计的角度和速度 */
    Sensorless_Theta = Sensorless_GetTheta();
    Sensorless_Speed = Sensorless_GetSpeed();

    /* 状态机处理 */
    static MotorState_t last_state = MOTOR_STOP;
    MotorState_t current_state = MOTOR_STOP;

    if (STOP)
    {
        current_state = MOTOR_STOP;
    }
    else if (fabsf(Sensorless_Speed) < 50.0f)
    {
        current_state = MOTOR_LOW_SPEED;
    }
    else
    {
        current_state = MOTOR_HIGH_SPEED;
    }

    if (current_state != last_state)
    {
        Sensorless_StateTransition(current_state);
        last_state = current_state;
    }
}

// ...existing code...
void PID_Controller(float setpoint, float measured_value,
                    PID_Controller_t *PID_Controller)
{
    float difference = setpoint - measured_value;
    float integral = PID_Controller->integral;
    float derivative = difference - PID_Controller->previous_error;

    if (STOP == 1)
    {
        difference = 0.0f;
        integral = 0.0f;
        derivative = 0.0f;
    }
    // Proportional term
    float P = PID_Controller->Kp * difference;
    // Integral term
    integral += PID_Controller->Ki * difference * PID_Controller->Ts;
    // Derivative term
    float D = PID_Controller->Kd * derivative;
    // Calculate output
    float output_value = P + integral + D;
    // Clamp output to limits
    if (output_value > PID_Controller->MaxOutput)
    {
        output_value = PID_Controller->MaxOutput;
    }
    else if (output_value < PID_Controller->MinOutput)
    {
        output_value = PID_Controller->MinOutput;
    }
    if (integral > PID_Controller->IntegralLimit)
        integral = PID_Controller->IntegralLimit;
    else if (integral < -PID_Controller->IntegralLimit)
        integral = -PID_Controller->IntegralLimit;
    // Update integral and previous error for next iteration
    PID_Controller->integral = integral;
    PID_Controller->previous_error = difference;
    // Return the output value
    PID_Controller->output = output_value;
}

LowPassFilter_t Speed_Filter = {.a = 0.9685841f, .y_last = 0.0f};

// SECTION - Theta Process
static inline void Theta_Process(void)
{
#ifdef SENSORLESS_POSITION
    /* 无位置传感器模式，使用估计的角度 */
    if (FOC.SensorlessEnabled && Sensorless.enabled)
    {
        theta_elec = Sensorless_Theta;
        FOC.Theta = theta_elec;

        /* 从电角度反推机械角度 */
        theta_mech = theta_elec / Motor.Pn;

        /* 速度已经在无位置传感器算法中计算 */
        FOC.Speed = Sensorless_Speed;
        return;
    }
#endif

    /* 传统位置传感器模式 */
    int32_t pos_delta = Position_Data - Motor.Position_Offset;
    if (pos_delta < 0)
        pos_delta += (Motor.Position_Scale + 1);

    theta_mech = pos_delta * theta_factor;

    theta_elec = MathUtils_WrapAngle2Pi(theta_mech * Motor.Pn);
    FOC.Theta = theta_elec;

    static float last_theta = 0.0f;
    float delta_theta = theta_mech - last_theta;

    if (delta_theta > M_PI)
        delta_theta -= 2.0f * M_PI;
    else if (delta_theta < -M_PI)
        delta_theta += 2.0f * M_PI;

    last_theta = theta_mech;

    float Speed = delta_theta * F_2K_HZ * 60.0f / (2.0f * M_PI);
    View_Speed = Speed; // 更新全局速度变量
    FOC.Speed = LowPassFilter_Update(&Speed_Filter, Speed);
}
// !SECTION

// SECTION - Ramp Generator
static inline float RampGenerator(RampGenerator_t *ramp)
{
    if (STOP == 1)
    {
        ramp->value = 0.0f;
        ramp->target = 0.0f;
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
        ramp->value = ramp->limit_max;
    if (ramp->value < ramp->limit_min)
        ramp->value = ramp->limit_min;

    return ramp->value;
}
// !SECTION

void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t *out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
    out->Ialpha = Ia;
    out->Ibeta = 0.57735026919f * (Ia + 2.0f * Ib); // 0.57735026919f 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
    out->Ialpha = 0.66666666667f * Ia - 0.33333333333f * (Ib + Ic);
    out->Ibeta = 0.57735026919f * (Ib - Ic); // 0.57735026919f 1/√3
#endif
}

void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta, Park_t *out)
{
    float cos_theta = COS(theta);
    float sin_theta = SIN(theta);
    out->Id = Ialpha * cos_theta + Ibeta * sin_theta;
    out->Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void InvParkTransform(float_t Ud, float_t Uq, float_t theta, InvPark_t *out)
{
    float cos_theta = COS(theta);
    float sin_theta = SIN(theta);
    out->Ualpha = Ud * cos_theta - Uq * sin_theta;
    out->Ubeta = Ud * sin_theta + Uq * cos_theta;
}

void Set_PWM_Duty(float_t Ta, float_t Tb, float_t Tc, float_t pwm_arr)
{
#if (defined(GATE_POLARITY_HIGH_ACTIVE))
    TIMER_CH0CV(TIMER0) = (uint32_t)(pwm_arr * Ta);
    TIMER_CH1CV(TIMER0) = (uint32_t)(pwm_arr * Tb);
    TIMER_CH2CV(TIMER0) = (uint32_t)(pwm_arr * Tc);
#elif (defined(GATE_POLARITY_LOW_ACTIVE))
    TIMER_CH0CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Ta));
    TIMER_CH1CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Tb));
    TIMER_CH2CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Tc));
#endif
}

static inline float Get_Theta(float Freq, float Theta)
{
    // 电角度递推：θ += ω·Ts，ω = 2π·f
    Theta += 2.0f * M_PI * Freq * T_2K_HZ;
    if (Theta > 2.0f * M_PI)
        Theta -= 2.0f * M_PI;
    else if (Theta < 0.0f)
        Theta += 2.0f * M_PI;
    return Theta;
}

void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, float pwm_arr)
{
    uint8_t sector = 0;
    float Vref1 = Ubeta;
    float Vref2 = (+SQRT3 * Ualpha - Ubeta) * 0.5f;
    float Vref3 = (-SQRT3 * Ualpha - Ubeta) * 0.5f;

    // 判断扇区（1~6）
    if (Vref1 > 0)
        sector += 1;
    if (Vref2 > 0)
        sector += 2;
    if (Vref3 > 0)
        sector += 4;

    // Clarke to T1/T2 projection
    float X = SQRT3 * Ubeta * inv_Vdc;
    float Y = (+1.5f * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;
    float Z = (-1.5f * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;

    float T1 = 0.0f, T2 = 0.0f;

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
        T1 = 0.0f;
        T2 = 0.0f;
        break;
    }

    // 过调制处理
    float T_sum = T1 + T2;
    if (T_sum > 1.0f)
    {
        T1 /= T_sum;
        T2 /= T_sum;
    }

    // 中心对称调制时间计算
    float T0 = (1.0f - T1 - T2) * 0.5f;
    float Ta = T0;
    float Tb = T0 + T1;
    float Tc = Tb + T2;

    float duty_a, duty_b, duty_c;

    // 扇区映射到ABC占空比
    switch (sector)
    {
    case 1:
        duty_a = Tb;
        duty_b = Ta;
        duty_c = Tc;
        break;
    case 2:
        duty_a = Ta;
        duty_b = Tc;
        duty_c = Tb;
        break;
    case 3:
        duty_a = Ta;
        duty_b = Tb;
        duty_c = Tc;
        break;
    case 4:
        duty_a = Tc;
        duty_b = Tb;
        duty_c = Ta;
        break;
    case 5:
        duty_a = Tc;
        duty_b = Ta;
        duty_c = Tb;
        break;
    case 6:
        duty_a = Tb;
        duty_b = Tc;
        duty_c = Ta;
        break;
    default:
        duty_a = 0.5f;
        duty_b = 0.5f;
        duty_c = 0.5f;
        break;
    }

    // 输出PWM占空比（ARR值）
    Set_PWM_Duty(duty_a, duty_b, duty_c, pwm_arr);
}
