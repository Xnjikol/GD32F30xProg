#include "main_int.h"
#include "Initialization.h"
#include "foc.h"
#include "hardware_interface.h"
#include "reciprocal.h"
#include "sensorless_interface.h"
#include "transformation.h"

#include <stdlib.h>

static DeviceStateEnum_t MainInt_State        = RUNNING;
static bool              MainInt_UseRealTheta = true;

static inline void MainInt_Update_FocCurrent(void) {
    Phase_t current_phase = Peripheral_Get_PhaseCurrent();
    Clark_t current_clark = {0};
    ClarkeTransform(&current_phase, &current_clark);
    current_clark = Sensorless_FilterCurrent(current_clark);
    Foc_Set_Iclark_Fdbk(current_clark);
}

static inline void MainInt_Check_ProtectFlag(void) {
    bool stop = Peripheral_Update_Break();
    Foc_Set_ResetFlag(stop);
    stop = Foc_Get_ResetFlag();
    Peripheral_Set_Stop(stop);
}

static inline void MainInt_Update_BusVoltage(void) {
    FloatWithInv_t bus_voltage = Peripheral_UpdateUdc();
    Foc_Set_BusVoltage(bus_voltage.val);
    Foc_Set_BusVoltageInv(bus_voltage.inv);
}

static inline void MainInt_Update_Sensorless(void) {
    Clark_t voltage = {0};
    Clark_t current = {0};

    voltage = Foc_Get_Uclark_Ref();
    current = Foc_Get_Iclark_Fdbk();

    Sensorless_Set_Voltage(voltage);
    Sensorless_Set_Current(current);

    Sensorless_Calculate();
}

static inline void MainInt_Update_Angle_and_Speed(void) {
    AngleResult_t res       = {0};
    AngleResult_t est       = {0};
    AngleResult_t real      = {0};
    float         speed_ref = Foc_Get_SpeedRamp();

    real = Peripheral_UpdatePosition();
    est  = Sensorless_UpdatePosition();

    Sensorless_Update_Err(real);

    if (MainInt_UseRealTheta) {
        res = real;
    } else {
        res = est;
    }

    Foc_Set_Speed(res.speed);
    Foc_Set_Angle(res.theta);

    Sensorless_Set_SpeedRef(speed_ref);
    Sensorless_Set_SpeedFdbk(res.speed);
    Sensorless_Set_Angle(res.theta);
}

static inline void MainInt_Initialization(void) {
    Initialization_Modules();
    Peripheral_CalibrateADC();
    if (Foc_Get_BusVoltage() > 200.0F) {
        Peripheral_EnableHardwareProtect();
    }
    Peripheral_Reset_ProtectFlag();
    Foc_Set_Mode(IDLE);
}

static inline void MainInt_Run_Foc(void) {
    Park_t u_dq_ref = Foc_Update_Main();
    Foc_Set_Udq_Ref(u_dq_ref);
}

static inline void MainInt_Exit(void) {
    Peripheral_Set_Stop(true);       // 停止所有操作
    Foc_Set_ResetFlag(true);         // 设置复位标志
    Foc_Set_Mode(IDLE);              // 切换到IDLE模式
    Peripheral_Reset_ProtectFlag();  // 重置保护标志
    Peripheral_DisableHardwareProtect();
}

static inline void MainInt_SVPWM(void) {
    Phase_t tcm = Foc_Get_Tcm();  // 获取三相PWM时间
    Peripheral_Set_PWMChangePoint(tcm);
}

/*!
    \brief      主中断函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Main_Int_Handler(void) {
    MainInt_Update_FocCurrent();
    MainInt_Update_Angle_and_Speed();
    MainInt_Update_BusVoltage();
    MainInt_Check_ProtectFlag();

    switch (MainInt_State) {
    case INIT: {
        MainInt_Initialization();
        MainInt_State = RUNNING;
        break;
    }

    case RUNNING: {
        // 运行状态：正常FOC控制
        MainInt_Run_Foc();
        MainInt_Update_Sensorless();
        break;
    }

    case EXIT: {
        // 退出状态：进行必要的清理
        MainInt_Exit();
        break;
    }
    }

    MainInt_SVPWM();
}
