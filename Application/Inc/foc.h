#ifndef _FOC_H_
#define _FOC_H_

#include "filter.h"
#include "foc_types.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"


/*       Constants      */

#define T_2kHz 0.0005F  /* T 2kHz */
#define f_2kHz 2000.0F  /* f 2kHz */
#define T_1kHz 0.0001F  /* T 1kHz */
#define T_200Hz 0.005F  /* T 200Hz */
#define T_10kHz 0.0001F /* 10kHz sampling time */

#define TIME_2KHZ 0.0005F  /* T 2kHz */
#define FREQ_2KHZ 2000.0F  /* f 2kHz */
#define TIME_1KHZ 0.0001F  /* T 1kHz */
#define TIME_200HZ 0.005F  /* T 200Hz */
#define TIME_10KHZ 0.0001F /* 10kHz sampling time */

void FOC_Main(FOC_Parameter_t* foc, VF_Parameter_t* vf, IF_Parameter_t* if_p);

#endif /* _FOC_H_ */
