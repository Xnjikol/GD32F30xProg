#include "FlyingStart.h"
#include "Buffer.h"
#include "foc.h"
#include "hardware_interface.h"
#include "main_int.h"
#include "motor.h"
#include "sensorless_interface.h"
#include "theta_calc.h"
#include "transformation.h"

volatile bool FlyingStartEnabled = false;
volatile bool RestartEnabled     = false;

FS_Handler_t Fs_Hnd = {0};

bool FlyingStart_Init(FS_Handler_t* hnd,
                      uint16_t      ShortCnt,
                      uint16_t      ReleaseCnt,
                      uint16_t      DeltaCnt,
                      float         Ts)
{
    if (hnd == NULL)
    {
        return false;
    }

    hnd->ShortCnt   = ShortCnt;
    hnd->ReleaseCnt = ReleaseCnt;
    hnd->DeltaCnt   = DeltaCnt;
    hnd->Ts         = Ts;
    hnd->state      = FS_STATE_IDLE;

    return true;
}

bool FlyingStart_Reset(FS_Handler_t* hnd)
{
    if (hnd == NULL)
    {
        return false;
    }

    hnd->ExeCnt  = 0;
    hnd->ThetaI1 = 0.0F;
    hnd->ThetaI2 = 0.0F;
    hnd->ThetaI3 = 0.0F;
    hnd->Thetad3 = 0.0F;
    hnd->WeI3    = 0.0F;
    hnd->ThetaE  = 0.0F;
    hnd->state   = FS_STATE_IDLE;

    return true;
}

void FlyingStart_Update(FS_Handler_t* hnd, Clark_t current)
{
    if (hnd == NULL)
    {
        return;
    }

    static float speedI3 = 0.0F;

    switch (hnd->state)
    {
    case FS_STATE_IDLE:
        Stop     = false;  // 防止程序直接停掉
        ShutFlag = true;

        if (current.a <= 0.1F && current.b <= 0.1F)
        {
            hnd->state  = FS_STATE_SHORT1;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_SHORT1:
        // 执行重投逻辑
        hnd->ExeCnt++;
        ShutFlag = false;
        if (hnd->ExeCnt >= hnd->ShortCnt)
        {
            hnd->state  = FS_STATE_RELEASE1;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_RELEASE1:
        // 执行释放逻辑
        if (hnd->ExeCnt == 0)
        {
            ATAN2(current.b, current.a, &hnd->ThetaI1);
        }
        hnd->ExeCnt++;
        ShutFlag = true;
        if (hnd->ExeCnt >= hnd->ReleaseCnt)
        {
            hnd->state  = FS_STATE_SHORT2;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_SHORT2:
        // 执行重投逻辑
        hnd->ExeCnt++;
        ShutFlag = false;
        if (hnd->ExeCnt >= hnd->ShortCnt)
        {
            hnd->state  = FS_STATE_RELEASE2;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_RELEASE2:
        // 执行释放逻辑
        if (hnd->ExeCnt == 0)
        {
            ATAN2(current.b, current.a, &hnd->ThetaI2);
        }
        hnd->ExeCnt++;
        ShutFlag = true;
        if (hnd->ExeCnt >= hnd->ReleaseCnt + hnd->DeltaCnt)
        {
            hnd->state  = FS_STATE_SHORT3;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_SHORT3:
        // 执行重投逻辑
        hnd->ExeCnt++;
        ShutFlag = false;
        if (hnd->ExeCnt >= hnd->ShortCnt)
        {
            hnd->state  = FS_STATE_Finish;
            hnd->ExeCnt = 0;
        }
        break;

    case FS_STATE_Finish:
        // 执行释放逻辑
        if (hnd->ExeCnt == 0)
        {
            ATAN2(current.b, current.a, &hnd->ThetaI3);
            float delta_theta = hnd->ThetaI1 + hnd->ThetaI3 - 2 * hnd->ThetaI2;
            if (delta_theta > M_PI)
            {
                delta_theta -= M_2PI;
            }
            else if (delta_theta < -M_PI)
            {
                delta_theta += M_2PI;
            }
            hnd->WeI3 = delta_theta / (hnd->Ts * hnd->DeltaCnt);
            speedI3   = radps2rpm(hnd->WeI3 * Motor_InvPn);

            float theta = (hnd->WeI3) * hnd->Ts * hnd->ShortCnt;
            float respd = Motor_Ld * SIN(theta);
            float respq = Motor_Lq * (1 - COS(theta));
            ATAN2(-respd, -respq, &hnd->Thetad3);
            hnd->ThetaE = wrap_theta_2pi(hnd->ThetaI3 - hnd->Thetad3);

            hnd->SpeedErr = Motor_Speed - speedI3;
            hnd->ThetaErr = Motor_ThetaElec - hnd->ThetaE;
            hnd->ThetaErr = wrap_theta_pi(hnd->ThetaErr);
            hnd->ThetaErr = rad2deg(hnd->ThetaErr);
        }
        hnd->ExeCnt++;
        ShutFlag = false;
        Stop     = true;
        if (hnd->ExeCnt < hnd->ReleaseCnt)
        {
            break;
        }
        hnd->ThetaE        = wrap_theta_2pi(hnd->ThetaE + hnd->WeI3 * hnd->Ts);
        hnd->state         = FS_STATE_IDLE;
        FlyingStartEnabled = false;
        hnd->ExeCnt        = 0;

        if (RestartEnabled)
        {
            Stop     = false;
            Foc_Mode = SPEED;

            // 设置转速
            Foc_Speed_Ref                = speedI3;
            Foc_Ramp_Speed_Handler.value = speedI3;

            // 设置电流
            Foc_Pid_CurQ_Handler.integral = hnd->WeI3 * Motor_Flux;

            MainInt_UseRealTheta       = false;
            Sensorless_Enabled         = true;
            Sensorless_SpeedEst        = speedI3;
            Sensorless_SpeedFilter2.x1 = speedI3;
            Sensorless_SpeedFilter2.x2 = speedI3;
            Sensorless_SpeedFilter2.y1 = speedI3;
            Sensorless_SpeedFilter2.y2 = speedI3;
            Sensorless_ThetaEst        = hnd->ThetaE;

            Sensorless_Theta_PID.integral = hnd->WeI3;
        }

        break;

    case FS_STATE_ERROR:
        ShutFlag = true;
        // 错误状态处理
        break;

    default:
        ShutFlag   = true;
        hnd->state = FS_STATE_ERROR;
        break;
    }
}