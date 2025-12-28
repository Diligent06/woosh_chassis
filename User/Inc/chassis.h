#ifndef CHASSIS_H
#define CHASSIS_H 

#include "hollysys.h"
#include "common.h"
#include "motorevo.h"

#define Steer_LF 0x05
#define Steer_RF 0x06
#define Steer_RB 0x07
#define Steer_LB 0x08

#define Drive_LF 0x01
#define Drive_RF 0x02
#define Drive_LB 0x03
#define Drive_RB 0x04

#define STEER_MOTOR_NUM 4
#define DRIVE_MOTOR_NUM 4

#define pi 3.1415926535897932384626433832795

extern s32 drive_motor_spd_cmd[DRIVE_MOTOR_NUM];
extern u8 drive_motor_id[DRIVE_MOTOR_NUM];
extern u8 steer_motor_id[STEER_MOTOR_NUM];
extern float steer_forward_pos[STEER_MOTOR_NUM];
extern float steer_motor_pos_cmd[STEER_MOTOR_NUM];



void Chassis_Init(void);
void Chassis_Set_drive_spd();
void Chassis_Drive_Info_Update();
void Chassis_Update(void);
void Chassis_Deinit(void);
void Chassis_Info_Query();
void Chassis_Steer_Info_Query();
void Chassis_Drive_Info_Query();
void Chassis_State_Update();
void Chassis_Set_steer_pos();
void Chassis_Set_steer_angle(float* angle);
void Chassis_Tidybot(float spd_x, float spd_y, float spd_w);



#endif