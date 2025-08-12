#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float Pn;
    float Resolver_Pn;
    float inv_MotorPn;
    float Position_Scale;
    float Position_Offset;  // Zero Position
    float theta_factor;
} Motor_Parameter_t;

bool  Motor_Set_Parameters(const Motor_Parameter_t* motor_params);
bool  Motor_Set_SpeedPrescaler(uint16_t prescaler);
bool  Motor_Set_Filter(float sample_freq, float cutoff_freq);
void  Motor_Set_Position(uint16_t position);
void  Motor_Set_Theta_Elec(float theta);
float Motor_Get_Theta_Elec(void);
void  Motor_Set_Theta_Mech(float theta);
float Motor_Get_Theta_Mech(void);
void  Motor_Set_Speed(float speed);
float Motor_Get_Speed(void);
