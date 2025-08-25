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

static bool                Sensorless_Enabled         = {0};
static bool                Sensorless_Use_RealTheta   = {0};
static float               Sensorless_Speed_Threshold = {0};
static sensorless_method_t Sensorless_Method
    = SENSORLESS_METHOD_SM_OBSERVER;

bool Sensorless_Set_Enabled(bool enabled) {
    Sensorless_Enabled = enabled;
    return Sensorless_Enabled;
}

bool Sensorless_Get_Enabled(void) {
    return Sensorless_Enabled;
}

bool Sensorless_Set_Method(sensorless_method_t method, bool enable) {
    if (enable) {
        Sensorless_Method |= method;
    } else {
        Sensorless_Method &= ~method;
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

    if (Sensorless_Method &= SENSORLESS_METHOD_HF_INJECTION) {
    }

    if (Sensorless_Method &= SENSORLESS_METHOD_SM_OBSERVER) {
        Smo_Set_Voltage(voltage);
    }

    return true;
}

bool Sensorless_Set_Current(Clark_t current) {
    if (!Sensorless_Enabled) {
        return false;
    }

    if (Sensorless_Method &= SENSORLESS_METHOD_HF_INJECTION) {
        Hfi_Set_Current(current);
    }

    if (Sensorless_Method &= SENSORLESS_METHOD_SM_OBSERVER) {
        Smo_Set_Current(current);
    }

    return true;
}

bool Sensorless_Update(void) {
    if (!Sensorless_Enabled) {
        return false;
    }

    if (Sensorless_Method &= SENSORLESS_METHOD_HF_INJECTION) {
        Hfi_Update();
    }

    if (Sensorless_Method &= SENSORLESS_METHOD_SM_OBSERVER) {
        Smo_Update_EmfEst();
        Smo_Update_Angle();
    }

    return true;
}

/**
 * @brief 获取经过滤波处理的电流（Park坐标系）。
 *
 * 此函数用于在传感器无刷控制中获取经过滤波的电流值。仅当Sensorless_Enabled为真且采用高频注入（SENSORLESS_METHOD_HF_INJECTION）方法时，
 * 才会返回实际的滤波电流，否则返回零值。
 *
 * 注意：本函数必须在Hfi_Update函数执行之后调用，否则滤波电流参数未更新，可能导致获取到的值不正确。
 *
 * @return Park_t 经过滤波处理的电流（Park坐标系）。
 */
bool Sensorless_Get_FilteredCurrent(Park_t* filtered_current) {
    if (!Sensorless_Enabled) {
        return false;
    }

    if (!(Sensorless_Method & SENSORLESS_METHOD_HF_INJECTION)) {
        return false;
    }

    if (filtered_current == NULL) {
        return false;
    }

    *filtered_current = Hfi_Get_FilteredCurrent();
    return true;
}
