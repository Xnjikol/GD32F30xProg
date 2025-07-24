#include "pid.h"

void PID_SetIntegral(PID_Handler_t* handler, bool Reset, float value)
{
  if (Reset)
  {
    return;
  }
  else
  {
    handler->integral = value;
    handler->Reset = false;
  }
}
