/*
 * File: FlyingStart.h
 * Path: /e:/Doc/Work/FRECON/Prj/GD32F30xProg/Application/Inc/FlyingStart.h
 *
 * 简要说明:
 *   带速重投模块（速度辅助起始/重启控制）接口头文件。
 *   该模块用于对电机/飞轮等执行带速度目标的启动、重投和状态管理。
 *
 * 该文件只包含接口声明，具体实现应放在 FlyingStart.c 中。
 */

#ifndef FLYINGSTART_H
#define FLYINGSTART_H

#include <stdbool.h>
#include <stdint.h>
#include "transformation.h"

#ifdef __cplusplus
extern "C"
{
#endif

    extern volatile bool FlyingStartEnabled;

    /* 状态枚举 */
    typedef enum
    {
        FS_STATE_IDLE = 0, /* 空闲 */
        FS_STATE_SHORT1,   /* 执行重投 */
        FS_STATE_SHORT2,   /* 执行重投 */
        FS_STATE_SHORT3,   /* 执行重投 */
        FS_STATE_RELEASE1, /* 执行释放 */
        FS_STATE_RELEASE2, /* 执行释放 */
        FS_STATE_Finish,   /* 执行释放 */
        FS_STATE_ERROR     /* 错误状态 */
    } FS_State_t;

    /* 结果码 */
    typedef struct
    {
        uint16_t ShortCnt;   /* 短路周期数 */
        uint16_t ReleaseCnt; /* 释放周期数 */
        uint16_t DeltaCnt;   /* 第二次释放周期数增量 */
        uint16_t ExeCnt;     /* 已执行周期数 */

        float Ts; /* 控制周期时间（秒）*/
        float ThetaI1;
        float ThetaI2;
        float ThetaI3;
        float Thetad3;
        float WeI3;
        float ThetaE;

        float SpeedErr;
        float ThetaErr;

        FS_State_t state;
    } FS_Handler_t;

    bool FlyingStart_Init(FS_Handler_t* hnd,
                          uint16_t      ShortCnt,
                          uint16_t      ReleaseCnt,
                          uint16_t      DeltaCnt,
                          float         Ts);

    extern FS_Handler_t Fs_Hnd;

    void FlyingStart_Update(FS_Handler_t* hnd, Clark_t current);

#ifdef __cplusplus
}
#endif

#endif /* FLYINGSTART_H */