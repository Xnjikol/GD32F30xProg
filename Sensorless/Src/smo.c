#include "smo.h"
#include "motor.h"
#include "theta_calc.h"

SMO_Handle_t SmoHandle = {0};

void SMO_Initialization(SMO_Handle_t* h, const SMO_Param_t* p)
{
    if (h != NULL && p != NULL)
    {
        h->param = *p;
        SMO_Reset(h);

        IIR1stFilter_Init(
            &h->EmfFilterA, p->Emf_cutoffFreq, SampleFreq);
        IIR1stFilter_Init(
            &h->EmfFilterB, p->Emf_cutoffFreq, SampleFreq);
        IIR2ndFilter_Init(
            &h->OmegaFilter, p->Speed_cutoffFreq, Speed_Freq);
    }

    return;
}

void SMO_Reset(SMO_Handle_t* h)
{
    if (h != NULL)
    {
        h->state.Ihat.a   = 0.0F;
        h->state.Ihat.b   = 0.0F;
        h->state.Ehat.a   = 0.0F;
        h->state.Ehat.b   = 0.0F;
        h->state.Ierror.a = 0.0F;
        h->state.Ierror.b = 0.0F;
        h->state.theta    = 0.0F;
        h->state.speed    = 0.0F;
        h->state.pll_err  = 0.0F;
        h->sign.a         = 0.0F;
        h->sign.b         = 0.0F;
        h->integrator.a   = 0.0F;
        h->integrator.b   = 0.0F;
    }

    return;
}

void SMO_Update(SMO_Handle_t* h, Clark_t voltage, Clark_t current)
{
    if (h == NULL)
    {
        return;
    }

    // 计算电流误差
    h->state.Ierror.a = h->state.Ihat.a - current.a;
    h->state.Ierror.b = h->state.Ihat.b - current.b;

    // 计算滑模符号函数输出
    h->sign.a = (h->state.Ierror.a > 0.0F)
                    ? 1.0F
                    : ((h->state.Ierror.a < 0.0F) ? -1.0F : 0.0F);
    h->sign.b = (h->state.Ierror.b > 0.0F)
                    ? 1.0F
                    : ((h->state.Ierror.b < 0.0F) ? -1.0F : 0.0F);

    // 更新积分器状态
    // h->integrator.a += h->param.K_i * h->state.Ierror.a * h->param.Ts;
    // h->integrator.b += h->param.K_i * h->state.Ierror.b * h->param.Ts;
    h->state.Fdbk.a = h->param.Gain * h->sign.a;
    h->state.Fdbk.b = h->param.Gain * h->sign.b;

    // 更新电流估计
    h->state.Ihat.a
        += (voltage.a - Motor_Rs * current.a - h->state.Fdbk.a)
           * (SampleTime / Motor_Ld);

    h->state.Ihat.b
        += (voltage.b - Motor_Rs * current.b - h->state.Fdbk.b)
           * (SampleTime / Motor_Ld);

    // 更新反电动势估计
    h->state.Ehat.a
        = IIR1stFilter_Update(&h->EmfFilterA, h->state.Fdbk.a);
    h->state.Ehat.b
        = IIR1stFilter_Update(&h->EmfFilterB, h->state.Fdbk.b);

    h->state.pll_err = -(h->state.Ehat.a * COS(h->state.theta)
                         + h->state.Ehat.b * SIN(h->state.theta));

    return;
}