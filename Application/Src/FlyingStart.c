#include "FlyingStart.h"
#include "Buffer.h"
#include "hardware_interface.h"
#include "motor.h"
#include "theta_calc.h"
#include "transformation.h"

volatile bool FlyingStartEnabled = false;

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

    switch (hnd->state)
    {
    case FS_STATE_IDLE:
        Peripheral_Set_Stop(false);
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
            float delta_theta
                = hnd->ThetaI1 + hnd->ThetaI3 - 2 * hnd->ThetaI2;
            if (delta_theta > M_PI)
            {
                delta_theta -= M_2PI;
            }
            else if (delta_theta < -M_PI)
            {
                delta_theta += M_2PI;
            }
            hnd->WeI3     = delta_theta / (hnd->Ts * hnd->DeltaCnt);
            float speedI3 = radps2rpm(hnd->WeI3 * Motor_MotorPn_inv);

            Buffer_Put(speedI3, 6);

            float theta = (hnd->WeI3) * hnd->Ts * hnd->ShortCnt;
            float respd = Motor_Ld * SIN(theta);
            float respq = Motor_Lq * (1 - COS(theta));
            ATAN2(-respd, -respq, &hnd->Thetad3);
            hnd->ThetaE = wrap_theta_2pi(hnd->ThetaI3 - hnd->Thetad3);

            Buffer_Put(hnd->ThetaE, 8);

            hnd->SpeedErr = Motor_Get_Speed() - speedI3;
            hnd->ThetaErr = Motor_Get_ThetaElec() - hnd->ThetaE;
            hnd->ThetaErr = wrap_theta_pi(hnd->ThetaErr);
            hnd->ThetaErr = rad2deg(hnd->ThetaErr);
        }
        hnd->ExeCnt++;
        ShutFlag = false;
        Peripheral_Set_Stop(true);
        if (hnd->ExeCnt >= hnd->ReleaseCnt)
        {
            hnd->state         = FS_STATE_IDLE;
            FlyingStartEnabled = false;
            hnd->ExeCnt        = 0;
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