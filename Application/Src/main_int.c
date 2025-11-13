#include "main_int.h"
#include "Buffer.h"
#include "FlyingStart.h"
#include "Initialization.h"
#include "foc.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "leso.h"
#include "reciprocal.h"
#include "sensorless_interface.h"
#include "transformation.h"

static DeviceStateEnum_t MainInt_State        = RUNNING;
static volatile bool     MainInt_UseRealTheta = true;
//static volatile uint16_t MainInt_DataFlag     = 0x000U;

static inline void MainInt_Update_FocCurrent(void)
{
    Foc_IPhase = Peripheral_Get_PhaseCurrent();

    Foc_Iclark_Fdbk = ClarkTransform(Foc_IPhase);
    Foc_Iclark_Fdbk = Sensorless_FilterCurrent(Foc_Iclark_Fdbk);
    Sensorless_Set_Current(Foc_Iclark_Fdbk);
    Foc_Set_Iclark_Fdbk(Foc_Iclark_Fdbk);

    Buffer_Put(Foc_Iclark_Fdbk.a, 0);
    Buffer_Put(Foc_Iclark_Fdbk.b, 1);
}

static inline void MainInt_Check_ProtectFlag(void)
{
    if (FlyingStartEnabled)
    {
        FlyingStart_Update(&Fs_Hnd, Foc_Iclark_Fdbk);
    }

    // 保护检测
    bool stop = Peripheral_Update_Break();
    if (Foc_Get_Mode() == IDLE)
    {
        stop = true;
    }
    Foc_Set_ResetFlag(stop || FlyingStartEnabled);
    Peripheral_Set_Stop(stop);
    Sensorless_Reset = stop;
}

static inline void MainInt_Update_BusVoltage(void)
{
    FloatWithInv_t bus_voltage = Peripheral_UpdateUdc();
    Foc_Set_BusVoltage(bus_voltage.val);
    Foc_Set_BusVoltageInv(bus_voltage.inv);
}

static inline void MainInt_Update_Angle_and_Speed(void)
{
    MotorState_t res  = {0};
    MotorState_t est  = {0};
    MotorState_t real = {0};

    real = Peripheral_Update_Position();
    est  = Sensorless_Update_Position();

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
    if (Foc_Get_BusVoltage() > 200.0F)
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
    Park_t  ref     = {0};
    Park_t  induc   = Foc_Get_Inductor();
    Leso_Set_Inductor(induc);

    voltage = Foc_Get_Uclark_Ref();

    Sensorless_Set_Voltage(voltage);

    Sensorless_Calculate();

    ref = Foc_Get_Udq_Ref();
    ref = Sensorless_Inject_Voltage(ref);
    Foc_Set_Udq_Ref(ref);
}

static inline void MainInt_Run_Foc(void)
{
    Park_t vol_dq_ref = {0};

    vol_dq_ref = Foc_Update_Main();

    Foc_Set_Udq_Ref(vol_dq_ref);
}

static inline void MainInt_Send_Data(void)
{
    Buffer_Send();
}

static inline void MainInt_Exit(void)
{
    Peripheral_Set_Stop(true);       // 停止所有操作
    Foc_Set_ResetFlag(true);         // 设置复位标志
    Foc_Set_Mode(IDLE);              // 切换到IDLE模式
    Peripheral_Reset_ProtectFlag();  // 重置保护标志
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
}
