#include "flying.h"
#include <stdbool.h>
#include <stdint.h>

static uint16_t Flying_Count     = 0x0000U;
static bool     Flying_Completed = false;

static volatile uint16_t Startup_Delay = 0x0008U;

bool Flying_Set_Enabled(bool enabled) {
    if (enabled) {
        Flying_Count     = 0x0000U;
        Flying_Completed = false;
    }
    return true;
}

bool Flying_Is_Completed(void) {
    return Flying_Completed;
}

void Flying_Update(bool reset) {
    if (reset) {
        Flying_Count     = 0x0000U;
        Flying_Completed = false;
        return;
    }

    if (Flying_Completed) {
        return;
    }

    Flying_Count++;
    if (Flying_Count >= Startup_Delay) {
        Flying_Completed = true;
    }
}