#include "Initialization.h"
#include "foc_types.h"
#include "tim.h"

bool Initialization_Variables(void) {
    /* initialize Timer */
    TIM0_PWM_Init(MAIN_INT_TIMER_PRESCALER,
                  MAIN_INT_TIMER_PERIOD,
                  MAIN_INT_TIMER_DEADTIME_PERIOD);
    // Initialize variables here
    return true;
}
