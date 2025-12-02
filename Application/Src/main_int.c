#include "main_int.h"
#include "Buffer.h"
#include "FlyingStart.h"
#include "Initialization.h"
#include "foc.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "leso.h"
#include "protect.h"
#include "reciprocal.h"
#include "sensorless_interface.h"
#include "transformation.h"

DeviceStateEnum_t MainInt_State        = RUNNING;
volatile bool     MainInt_UseRealTheta = true;
volatile bool     MainInt_Calculating  = false;
// volatile uint16_t MainInt_DataFlag     = 0x000U;

static inline void MainInt_Update_FocCurrent(void)
{
    Foc_IPhase = Peripheral_Get_PhaseCurrent();

    Foc_Iclark_Fdbk = ClarkTransform(Foc_IPhase);
    Foc_Iclark_Fdbk = Sensorless_FilterCurrent(Foc_Iclark_Fdbk);
    Sensorless_Set_Current(Foc_Iclark_Fdbk);

    if (FlyingStartEnabled)
    {
        FlyingStart_Update(&Fs_Hnd, Foc_Iclark_Fdbk);
    }
}

static inline void MainInt_Check_ProtectFlag(void)
{
    // 保护检测
    if (Foc_Get_Mode() == IDLE)
    {
        Stop = true;
    }
    Peripheral_Update_Break();
    Foc_Reset = Stop || FlyingStartEnabled;

    Sensorless_Reset = Stop;
}

static inline void MainInt_Update_BusVoltage(void)
{
    FloatWithInv_t bus_voltage = Peripheral_UpdateUdc();

    Foc_BusVoltage     = bus_voltage.val;
    Foc_BusVoltage_Inv = bus_voltage.inv;
}

static inline void MainInt_Update_Angle_and_Speed(void)
{
    MotorState_t res  = {0};
    MotorState_t est  = {0};
    MotorState_t real = {0};

    real = Peripheral_Update_Position();
    est  = Sensorless_Update_Position();

    Buffer_Put(real.speed, 4);
    Buffer_Put(Foc_Speed_Ramp, 5);
    Buffer_Put(est.speed, 6);

    Sensorless_Calculate_Err(real);

    if (MainInt_UseRealTheta)
    {
        res = real;
    }
    else if (Foc_Speed_Ramp >= Sensorless_Switch_Speed)
    {
        res = est;
    }
    else
    {
        res = real;
    }

    Foc_Speed_Fdbk = res.speed;
    Foc_Theta      = res.theta;

    Sensorless_SpeedRef  = Foc_Speed_Ramp;
    Sensorless_SpeedFdbk = real.speed;
}

static inline void MainInt_Initialization(void)
{
    Initialization_Modules();
    Peripheral_CalibrateADC();
    if (Foc_BusVoltage > 200.0F)
    {
        Peripheral_EnableHardwareProtect();
    }
    Peripheral_Reset_ProtectFlag();
    Foc_Set_Mode(IDLE);
}

static inline void MainInt_Startup(void)
{
    if (Sensorless_Method == SENSORLESS_START)
    {
        Foc_Set_Mode(STARTUP);
    }
    else
    {
        Foc_Set_Mode(SPEED);
    }
}

static inline void MainInt_Update_Sensorless(void)
{
    Clark_t voltage = {0};
    Park_t  induc   = Foc_Get_Inductor();
    Leso_Set_Inductor(induc);

    voltage = Foc_Get_Uclark_Ref();

    Sensorless_Set_Voltage(voltage);

    Sensorless_Calculate();

    Foc_Udq_Ref = Sensorless_Inject_Voltage(Foc_Udq_Ref);
}

static inline void MainInt_Run_Foc(void)
{
    Foc_Udq_Ref = Foc_Update_Main();
}

static inline void MainInt_Send_Data(void)
{
    Buffer_Send();
}

static inline void MainInt_Exit(void)
{
    Stop      = true;      // 停止所有操作
    Foc_Reset = true;      // 设置复位标志
    Foc_Mode  = IDLE;      // 切换到IDLE模式
    Protect_Reset_Flag();  // 重置保护标志
    Peripheral_DisableHardwareProtect();
}

static inline void MainInt_SVPWM(void)
{
    Phase_t tcm = Foc_Get_Tcm();  // 获取三相PWM时间
    if (FlyingStartEnabled)
    {
        tcm.a = 1.0F;
        tcm.b = 1.0F;
        tcm.c = 1.0F;
    }
    Peripheral_Set_PWMChangePoint(tcm);
}

/*!
    \brief      主中断函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Handler(void)
{
    MainInt_Calculating = !MainInt_Calculating;

    MainInt_Update_FocCurrent();
    MainInt_Update_Angle_and_Speed();
    MainInt_Update_BusVoltage();
    MainInt_Check_ProtectFlag();

    switch (MainInt_State)
    {
    case INIT:
    {
        MainInt_Initialization();
        MainInt_State = RUNNING;
        break;
    }

    case RUNNING:
    {
        // 运行状态：正常FOC控制
        MainInt_Run_Foc();
        MainInt_Update_Sensorless();
        Foc_Uclark_Ref = InvParkTransform(Foc_Udq_Ref, Foc_Theta);
        break;
    }

    case SENSORLESS:
    {
        MainInt_Startup();
        MainInt_Run_Foc();
        MainInt_Update_Sensorless();
        break;
    }

    case EXIT:
    {
        // 退出状态：进行必要的清理
        MainInt_Exit();
        break;
    }
    }

    MainInt_SVPWM();
    MainInt_Send_Data();

    Stop_Prev = Stop;

    if (MainInt_Calculating)
    {
        MainInt_Calculating = false;
    }
    else
    {
        MainInt_Calculating = true;
    }
}
