#ifndef _PHERIPHERAL_INTERFACE_H_
#define _PHERIPHERAL_INTERFACE_H_

#include "com_frame.h"
#include "reciprocal.h"
#include "stdbool.h"
#include "stdint.h"
#include "theta_calc.h"
#include "transformation.h"

bool Peripheral_Get_SoftwareBrk(void);
bool Peripheral_Get_HardwareBrk(void);
void Peripheral_Set_Stop(bool stop);
bool Peripheral_Get_Stop(void);
bool Peripheral_Update_Break(void);
void Peripheral_Set_PWMChangePoint(Phase_t tcm);

bool Peripheral_CANSend(const can_frame_t* frame);
bool Peripheral_CANReceive(can_frame_t* frame);
void Peripheral_SCISend(float* TxBuffer, uint8_t floatnum);
void Peripheral_SCISendCallback(void);

void           Peripheral_CalibrateADC(void);
FloatWithInv_t Peripheral_UpdateUdc(void);
Phase_t        Peripheral_Get_PhaseCurrent(void);
AngleResult_t  Peripheral_Update_Position(void);
void           Peripheral_Update_Temperature(void);
void           Peripheral_Reset_ProtectFlag(void);
void           Peripheral_EnableHardwareProtect(void);
void           Peripheral_DisableHardwareProtect(void);

#endif /* _PHERIPHERAL_INTERFACE_H_ */
