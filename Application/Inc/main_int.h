#ifndef __MAIN_INT_H__
#define __MAIN_INT_H__

#include <stdbool.h>

#define MAIN_INT_BRK_VOLTAGE 620.0F

typedef enum
{
    INIT,        // 基础初始化：仅获取系统参数
    RUNNING,     // 运行：正常工作状态
    SENSORLESS,  // 传感器无位置模式
    EXIT
} DeviceStateEnum_t;

extern volatile bool MainInt_UseRealTheta;

void Main_Int_Handler(void);

#endif /* __MAIN_INT_H__ */
