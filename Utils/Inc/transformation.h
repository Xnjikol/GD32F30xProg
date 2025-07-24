#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include <stdbool.h>
#include <stdint.h>

#include "gd32f30x.h"

/* Current sensing phase setting */
#ifndef TWO_PHASE_CURRENT_SENSING
#ifndef THREE_PHASE_CURRENT_SENSING
#define THREE_PHASE_CURRENT_SENSING /* Define here */
#endif
#endif

#if !defined(TWO_PHASE_CURRENT_SENSING) && !defined(THREE_PHASE_CURRENT_SENSING)
#error "Please define TWO_PHASE_CURRENT_SENSING or THREE_PHASE_CURRENT_SENSING in foc.h"
#endif

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

typedef struct
{
  float a;
  float b;
  float c;
} Phase_Data_t;

typedef struct
{
  float a;
  float b;
} Clarke_t;

// struct Park_Data_t is commented out as struct FOC_Parameter_t takes care of Id and Iq
typedef struct
{
  float d;
  float q;
} Park_Data_t;

static inline void ClarkeTransform(float Ia, float Ib, float Ic, Clarke_t* out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
  out->a = Ia;
  out->b = 0.57735026919F * (Ia + 2.0F * Ib);  // 0.57735026919F 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
  out->a = 0.66666666667F * Ia - 0.33333333333F * (Ib + Ic);
  out->b = 0.57735026919F * (Ib - Ic);  // 0.57735026919F 1/√3
#endif
}
static inline void ParkTransform(float alpha, float beta, float theta, Park_Data_t* out)
{
  float_t cos_theta = COS(theta);
  float_t sin_theta = SIN(theta);
  out->d = alpha * cos_theta + beta * sin_theta;
  out->q = -alpha * sin_theta + beta * cos_theta;
}
static inline void InvParkTransform(float d, float q, float theta, Clarke_t* out)
{
  float_t cos_theta = COS(theta);
  float_t sin_theta = SIN(theta);
  out->a = d * cos_theta - q * sin_theta;
  out->b = d * sin_theta + q * cos_theta;
}
static inline void InvClarkeTransform(float alpha, float beta, Phase_Data_t* out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
  // For two-phase current sensing, Ic is calculated as -(Ia + Ib)
  out->a = alpha;
  out->b = (-0.5F) * alpha + 0.86602540378F * beta;  // 0.86602540378F = sqrt(3)/2
  out->c = (-0.5F) * alpha - 0.86602540378F * beta;
#elif (defined(THREE_PHASE_CURRENT_SENSING))
  // For three-phase current sensing
  out->a = alpha;
  out->b = -0.5F * alpha + 0.86602540378F * beta;
  out->c = -0.5F * alpha - 0.86602540378F * beta;
#endif
}

#endif /* _TRANSFORMATION_H_ */