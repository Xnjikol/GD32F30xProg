#include "can.h"
#include "foc.h"
#include "gd32f30x.h"
#include "gd32f30x_it.h"
#include "hardware_interface.h"
#include "justfloat.h"
#include "main.h"
#include "pid.h"
#include "systick.h"


/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#ifndef GATE_POLARITY_LOW_ACTIVE
#define GATE_POLARITY_LOW_ACTIVE /* Define here */
#endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) && !defined(GATE_POLARITY_LOW_ACTIVE)
#error "Please define GATE_POLARITY_HIGH_ACTIVE or GATE_POLARITY_LOW_ACTIVE in foc.h"
#endif

#ifndef Resolver_Position
#ifndef Encoder_Position
#define Encoder_Position /* Define here */
#endif
#endif

/* FOC parameters */
#define SPEED_LOOP_PRESCALER 10 /* Speed loop frequency division factor */

extern Sensor_Parameter_t Sensor;

static inline void Sensor_UpdatePosition(uint16_t Position)
{
  Sensor.Position_Scale = Position;   // Update position scale
  Sensor.Position_Offset = Position;  // Update zero position
}

void FOC_UpdateMainFrequency(float f, float Ts, float PWM_ARR);

void Main_Int_Handler(void);
