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


extern s32 drive_motor_spd_cmd[4];
extern u8 drive_motor_id[4];
extern u8 steer_motor_id[4];
extern float steer_forward_pos[4];

#define STEER_MOTOR_NUM 4
#define DRIVE_MOTOR_NUM 4

void Chassis_Init(void);
void Chassis_Set_drive_spd();
void Chassis_Drive_Info_Update();
void Chassis_Update(void);
void Chassis_Deinit(void);


#endif