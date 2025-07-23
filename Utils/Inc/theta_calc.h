#ifndef _THETA_CALC_H_
#define _THETA_CALC_H_

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "gd32f30x.h"

#define SQRT3 1.73205080757F
#define SQRT3_2 0.86602540378F        /* √3/2 */
#define M_2PI 6.28318530717958647692F /* 2π */

/*  DSP math function    */
#ifndef ARM_DSP
#define ARM_DSP
#endif

#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)

#else
#include <math.h>

#define COS(x) cosf(x)
#define SIN(x) sinf(x)

#endif

static inline float_t wrap_theta_2pi(float_t theta)
{
  theta = fmodf(theta, M_2PI);
  if (theta < 0.0F)
  {
    theta += M_2PI;
  }
  return theta;
}

#endif /* _THETA_CALC_H_ */