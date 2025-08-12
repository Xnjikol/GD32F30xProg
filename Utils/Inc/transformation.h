#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include <stdbool.h>
#include <stdint.h>

#include "gd32f30x.h"

/* Current sensing phase setting */
#ifndef TWO_PHASE_CURRENT_SENSING
#    ifndef THREE_PHASE_CURRENT_SENSING
#        define THREE_PHASE_CURRENT_SENSING /* Define here */
#    endif
#endif

#if !defined(TWO_PHASE_CURRENT_SENSING) \
    && !defined(THREE_PHASE_CURRENT_SENSING)
#    error \
        "Please define TWO_PHASE_CURRENT_SENSING or THREE_PHASE_CURRENT_SENSING in foc.h"
#endif

#ifndef SQRT3
#    define SQRT3 1.73205080757F
#endif

#ifndef SQRT3_2
#    define SQRT3_2 0.86602540378F /* √3/2 */
#endif

#ifndef M_PI
#    define M_PI 3.14159265358979323846F /* π */
#endif

#ifndef M_2PI
#    define M_2PI 6.28318530717958647692F /* 2π */
#endif

#ifndef M_PI_2
#    define M_PI_2 1.57079632679489661923F /* π/2 */
#endif

/* DSP math function    */
#ifndef ARM_DSP
#    define ARM_DSP
#endif

#ifdef ARM_DSP
#    include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export

#    define COS(x) arm_cos_f32(x)
#    define SIN(x) arm_sin_f32(x)

#else
#    include <math.h>

#    define COS(x) cosf(x)
#    define SIN(x) sinf(x)

#endif

typedef struct {
    float a;
    float b;
    float c;
} Phase_t;

typedef struct {
    float a;
    float b;
} Clark_t;

// struct Park_t is commented out as struct FOC_Parameter_t takes care of Id and Iq
typedef struct {
    float d;
    float q;
} Park_t;

static inline void ClarkeTransform(const Phase_t* in, Clark_t* out) {
#if (defined(TWO_PHASE_CURRENT_SENSING))
    out->a = in->a;
    out->b = 0.57735026919F
             * (in->a + 2.0F * in->b);  // 0.57735026919F 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
    out->a = 0.66666666667F * in->a - 0.33333333333F * (in->b + in->c);
    out->b = 0.57735026919F * (in->b - in->c);  // 0.57735026919F 1/√3
#endif
}
static inline void ParkTransform(const Clark_t* in,
                                 float          theta,
                                 Park_t*        out) {
    float_t cos_theta = COS(theta);
    float_t sin_theta = SIN(theta);
    out->d            = in->a * cos_theta + in->b * sin_theta;
    out->q            = -in->a * sin_theta + in->b * cos_theta;
}
static inline void InvParkTransform(const Park_t* in,
                                    float         theta,
                                    Clark_t*      out) {
    float_t cos_theta = COS(theta);
    float_t sin_theta = SIN(theta);
    out->a            = in->d * cos_theta - in->q * sin_theta;
    out->b            = in->d * sin_theta + in->q * cos_theta;
}
static inline void InvClarkeTransform(const Clark_t* in, Phase_t* out) {
#if (defined(TWO_PHASE_CURRENT_SENSING))
    // For two-phase current sensing, Ic is calculated as -(Ia + Ib)
    out->a = in->a;
    out->b = (-0.5F) * in->a
             + 0.86602540378F * in->b;  // 0.86602540378F = sqrt(3)/2
    out->c = (-0.5F) * in->a - 0.86602540378F * in->b;
#elif (defined(THREE_PHASE_CURRENT_SENSING))
    // For three-phase current sensing
    out->a = in->a;
    out->b = -0.5F * in->a + SQRT3_2 * in->b;
    out->c = -0.5F * in->a - SQRT3_2 * in->b;
#endif
}

#endif /* _TRANSFORMATION_H_ */