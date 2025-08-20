#include "main_int.h"
#include "Initialization.h"
#include "foc.h"
#include "hardware_interface.h"
#include "reciprocal.h"
#include "transformation.h"

#include <stdlib.h>

static DeviceStateEnum_t MainInt_State = RUNNING;

static inline void MainInt_Update_FocCurrent(void) {
    Phase_t current_phase = Peripheral_Get_PhaseCurrent();
    Clark_t current_clark = {0};
    ClarkeTransform(&current_phase, &current_clark);
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

static inline void MainInt_Update_Angle_and_Speed(void) {
    AngleResult_t angle_result = Peripheral_UpdatePosition();
    Foc_Set_Speed(angle_result.speed);
    Foc_Set_Angle(angle_result.theta);
}

static inline void MainInt_Initialization(void) {
    Initialization_Variables();
    Peripheral_CalibrateADC();
    if (Foc_Get_BusVoltage() > 200.0F) {
        Peripheral_EnableHardwareProtect();
    }
    Peripheral_Reset_ProtectFlag();
    Foc_Set_Mode(IDLE);
}

static inline void MainInt_Run_Foc(void) {
    Foc_Update_Main();
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
    MainInt_Check_ProtectFlag();
    MainInt_Update_BusVoltage();
    MainInt_Update_Angle_and_Speed();

    switch (MainInt_State) {
    case INIT: {
        MainInt_Initialization();
        break;
    }

    case RUNNING: {
        // 运行状态：正常FOC控制
        MainInt_Run_Foc();
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
