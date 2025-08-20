#ifndef COM_H
#define COM_H
#include "stdint.h"

void Com_Initialization(void);
void COM_CANProtocol(void);
void COM_DAQProtocol(uint32_t systick_ms);
void COM_SCIProtocol(void);
#endif // COM_H
