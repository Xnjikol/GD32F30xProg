#ifndef _FOC_H_
#define _FOC_H_

#include "filter.h"
#include "foc_types.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

void FOC_Main(FOC_Parameter_t* foc, VF_Parameter_t* vf, IF_Parameter_t* if_p);

#endif /* _FOC_H_ */
