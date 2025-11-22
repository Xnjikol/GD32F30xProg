#include "svpwm.h"
#include "transformation.h"

Phase_t calculate_SVPWM_Tcm(Clark_t u_ref, float inv_Vdc)
{
    float   alpha  = u_ref.a;
    float   beta   = u_ref.b;
    uint8_t sector = 0;
    float   v_ref1 = beta;
    float   v_ref2 = (+SQRT3 * alpha - beta) * 0.5F;
    float   v_ref3 = (-SQRT3 * alpha - beta) * 0.5F;

    // 判断扇区（1~6）
    if (v_ref1 > 0)
    {
        sector += 1;
    }
    if (v_ref2 > 0)
    {
        sector += 2;
    }
    if (v_ref3 > 0)
    {
        sector += 4;
    }

    // Clarke to t1/t2 projection
    float X = SQRT3 * beta * inv_Vdc;
    float Y = (+1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;
    float Z = (-1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;

    float t1 = 0.0F, t2 = 0.0F;

    switch (sector)
    {
    case 1:
        t1 = Z;
        t2 = Y;
        break;
    case 2:
        t1 = Y;
        t2 = -X;
        break;
    case 3:
        t1 = -Z;
        t2 = X;
        break;
    case 4:
        t1 = -X;
        t2 = Z;
        break;
    case 5:
        t1 = X;
        t2 = -Y;
        break;
    case 6:
        t1 = -Y;
        t2 = -Z;
        break;
    default:
        t1 = 0.0F;
        t2 = 0.0F;
        break;
    }

    // 过调制处理
    float T_sum = t1 + t2;
    if (T_sum > 1.0F)
    {
        t1 /= T_sum;
        t2 /= T_sum;
    }

    // 中心对称调制时间计算
    float t0 = (1.0F - t1 - t2) * 0.5F;
    float ta = t0;
    float tb = t0 + t1;
    float tc = tb + t2;

    Phase_t tcm = {0.0F, 0.0F, 0.0F};

    // 扇区映射到ABC换相点
    switch (sector)
    {
    case 1:
        tcm.a = tb;
        tcm.b = ta;
        tcm.c = tc;
        break;
    case 2:
        tcm.a = ta;
        tcm.b = tc;
        tcm.c = tb;
        break;
    case 3:
        tcm.a = ta;
        tcm.b = tb;
        tcm.c = tc;
        break;
    case 4:
        tcm.a = tc;
        tcm.b = tb;
        tcm.c = ta;
        break;
    case 5:
        tcm.a = tc;
        tcm.b = ta;
        tcm.c = tb;
        break;
    case 6:
        tcm.a = tb;
        tcm.b = tc;
        tcm.c = ta;
        break;
    default:
        tcm.a = 0.5F;
        tcm.b = 0.5F;
        tcm.c = 0.5F;
        break;
    }

    return tcm;
}
