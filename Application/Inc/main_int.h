#ifndef __MAIN_INT_H__
#define __MAIN_INT_H__

typedef enum {
    INIT,     // 基础初始化：仅获取系统参数
    RUNNING,  // 运行：正常工作状态
    EXIT
} DeviceStateEnum_t;

void Main_Int_Handler(void);

#endif /* __MAIN_INT_H__ */
