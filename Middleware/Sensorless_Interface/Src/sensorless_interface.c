/**
 * @file sensorless_interface.c
 * @brief 无传感器控制接口层实现
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 */

#include "sensorless_interface.h"

#include <stdbool.h>
#include "filter.h"
#include "hf_injection.h"
#include "reciprocal.h"
#include "smo.h"
#include "transformation.h"
#include <math.h>

static bool                Sensorless_Enabled    = {0};
static float               Sensorless_Switch_Hfi = {0};
static float               Sensorless_Switch_Smo = {0};
static float               Sensorless_Speed_Ref  = {0};
static float               Sensorless_Speed_Fdbk = {0};
static float               Sensorless_Theta      = {0};
static sensorless_method_t Sensorless_Method
    = SENSORLESS_METHOD_SM_OBSERVER;

bool Sensorless_Initialization(const Sensorless_Param_t* param) {
    if (param == NULL) {
        return false;
    }

    Sensorless_Switch_Hfi = param->switch_speed + param->hysteresis;
    Sensorless_Switch_Smo = param->switch_speed - param->hysteresis;

    return true;
}

bool Sensorless_Set_Enabled(bool enabled) {
    Sensorless_Enabled = enabled;
    return Sensorless_Enabled;
}

bool Sensorless_Get_Enabled(void) {
    return Sensorless_Enabled;
}

bool Sensorless_Set_Method(sensorless_method_t method, bool enable) {
    switch (method) {
    case SENSORLESS_METHOD_HF_INJECTION:
        Hfi_Set_Enabled(enable);
        break;

    case SENSORLESS_METHOD_SM_OBSERVER:
        Smo_Set_Enabled(enable);
        break;

    default:
        return false;
        break;
    }
    return true;
}

sensorless_method_t Sensorless_Get_Method(void) {
    return Sensorless_Method;
}

bool Sensorless_Set_Voltage(Clark_t voltage) {
    if (!Sensorless_Enabled) {
        return false;
    }

    if (Hfi_Get_Enabled()) {
    }

    if (Smo_Get_Enabled()) {
        Smo_Set_Voltage(voltage);
    }

    return true;
}

bool Sensorless_Set_Current(Clark_t current) {
    if (!Sensorless_Enabled) {
        // Hfi_Set_Current((Clark_t){0});
        Smo_Set_Current((Clark_t){0});
        return false;
    }

    // if (Hfi_Get_Enabled()) {
    //     Hfi_Set_Current(current);
    // }

    if (Smo_Get_Enabled()) {
        Smo_Set_Current(current);
    }

    return true;
}

void Sensorless_Set_SpeedFdbk(float fdbk) {
    Sensorless_Speed_Fdbk = fdbk;
}

void Sensorless_Set_SpeedRef(float ref) {
    Sensorless_Speed_Ref = ref;
}

void Sensorless_Set_Angle(float angle) {
    Sensorless_Theta = angle;
}

float Sensorless_Get_HfiResponse(void) {
    return Hfi_Get_Response();
}

float Sensorless_Get_HfiError(void) {
    return Hfi_Get_Error();
}

bool Sensorless_Update_Err(AngleResult_t result) {
    if (!Sensorless_Enabled) {
        return false;
    }

    float theta = result.theta;
    float speed = result.speed;

    Hfi_Set_Theta_Err(theta);
    Hfi_Set_Speed_Err(speed);

    Smo_Set_Theta_Err(theta);
    Smo_Set_Speed_Err(speed);

    return true;
}

AngleResult_t Sensorless_Update_Position(void) {
    if (Smo_Get_Enabled()) {
        return Smo_Get_Result();
    } else {
        return Hfi_Get_Result();
    }
}

static inline void enable_smo(bool enable) {
    if (enable) {
        if (!Smo_Get_Enabled()) {
            Smo_Set_Enabled(true);
        }
    } else {
        if (Smo_Get_Enabled()) {
            Smo_Set_Enabled(false);
        }
    }
}

static inline void enable_hfi(bool enable) {
    if (enable) {
        if (!Hfi_Get_Enabled()) {
            Hfi_Set_Enabled(true);
            float estimate, target, error;
            estimate = Hfi_Get_Result().theta;
            target   = Sensorless_Theta;
            error    = wrap_theta_2pi(target - estimate + PI) - PI;
            if (error >= M_PI_4 || error <= -M_PI_4) {
                Hfi_Set_InitialPosition(target);
            }
        }
    } else {
        if (Hfi_Get_Enabled()) {
            Hfi_Set_Enabled(false);
        }
    }
}

static inline void judge_strategy(float ref, float fdbk) {
    if (fabsf(ref) >= Sensorless_Switch_Smo) {
        if (fabsf(fdbk) >= Sensorless_Switch_Smo) {
            enable_smo(true);
        }
    } else {
        if (fabsf(fdbk) < Sensorless_Switch_Smo) {
            enable_smo(false);
        }
    }
    if (fabsf(ref) <= Sensorless_Switch_Hfi) {
        if (fabsf(fdbk) <= Sensorless_Switch_Hfi) {
            enable_hfi(true);
        }
    } else {
        if (fabsf(fdbk) > Sensorless_Switch_Hfi) {
            enable_hfi(false);
        }
    }
}

bool Sensorless_Calculate(void) {
    if (!Sensorless_Enabled) {
        return false;
    }

    judge_strategy(Sensorless_Speed_Ref, Sensorless_Speed_Fdbk);

    if (Hfi_Get_Enabled()) {
        Hfi_Update();
    }

    if (Smo_Get_Enabled()) {
        Smo_Update_EmfEst();
        Smo_Update_Angle();
    }

    return true;
}

Park_t Sensorless_Inject_Voltage(Park_t voltage) {
    if (!Sensorless_Enabled) {
        return voltage;
    }

    if (Hfi_Get_Enabled()) {
        Park_t inj = Hfi_Get_Inject_Voltage();
        voltage.d += inj.d;
        voltage.q += inj.q;
        return voltage;
    }

    return voltage;
}

Clark_t Sensorless_FilterCurrent(Clark_t current) {
    if (!Sensorless_Enabled) {
        return current;
    }

    if (!Hfi_Get_Enabled()) {
        return current;
    }

    return Hfi_Get_FilteredCurrent(current);
}
