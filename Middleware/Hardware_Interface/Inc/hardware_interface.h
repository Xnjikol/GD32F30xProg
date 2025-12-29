#ifndef _PHERIPHERAL_INTERFACE_H_
#define _PHERIPHERAL_INTERFACE_H_

#include "com_frame.h"
#include "reciprocal.h"
#include "stdbool.h"
#include "stdint.h"
#include "theta_calc.h"
#include "transformation.h"

extern bool Stop;
extern bool Stop_Prev;

// 作用与Stop相同，但是防止Stop==true后程序不运行
extern volatile bool ShutFlag;

bool Peripheral_Get_SoftwareBrk(void);
bool Peripheral_Get_HardwareBrk(void);
bool Peripheral_Update_Break(void);
void Peripheral_Set_PWMChangePoint(Phase_t tcm);
void Peripheral_Set_BrkRatio(float ratio);

bool Peripheral_CANSend(const can_frame_t* frame);
bool Peripheral_CANReceive(can_frame_t* frame);
void Peripheral_SCISend(float* TxBuffer, uint8_t floatnum);
void Peripheral_SCISendCallback(void);

void           Peripheral_CalibrateADC(void);
FloatWithInv_t Peripheral_UpdateUdc(void);
Phase_t        Peripheral_Get_PhaseCurrent(void);
MotorState_t   Peripheral_Update_Position(void);
void           Peripheral_Update_Temperature(void);
void           Peripheral_Reset_ProtectFlag(void);
void           Peripheral_EnableHardwareProtect(void);
void           Peripheral_DisableHardwareProtect(void);

#endif /* _PHERIPHERAL_INTERFACE_H_ */
