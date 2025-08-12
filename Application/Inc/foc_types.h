#ifndef _FOC_TYPES_H_
#define _FOC_TYPES_H_

#include "pid.h"
#include "signal.h"
#include "stdint.h"
#include "transformation.h"

typedef enum {
    Disable = 0,
    Enable  = !Disable
} EnableStatus,
    CommandStatus;

#endif /* _FOC_TYPES_H_ */
