#ifndef _FOC_H_
#define _FOC_H_

#include "foc_types.h"

void FOC_Update(FOC_Parameter_t* foc);
void Foc_Set_SampleTime(SystemTimeConfig_t* config);
void Foc_Set_State(bool reset);
bool Foc_Get_State(void);
void Foc_Set_Angle(float angle);
void Foc_Set_Speed(float speed);
void Foc_Set_Speed_and_Angle(AngleResult_t* angle_speed);
void Foc_Set_Iclark_Fdbk(Clark_t* current);
void Foc_Set_Vf_Param(VF_Parameter_t* vf_param);
void Foc_Set_If_Param(IF_Parameter_t* if_param);

#endif /* _FOC_H_ */
