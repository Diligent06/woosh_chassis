#ifndef HOLLYSYS_H
#define HOLLYSYS_H


#include "fdcan.h"
#include "string.h"
#include "common.h"



//canopen区域

#define PP_Mode  1
#define PV_Mode  3 
#define PT_Mode  4 


//SDO CMD
#define  SDO_W1   0x2F
#define  SDO_W2   0x2B
#define  SDO_W4   0x23
#define  SDO_RD   0x40


//Object dictionary of CANopen

#define  Control_word                  0x6040
#define  Status_word                   0x6041
#define  Modes_of_operation            0x6060
#define  Modes_0f_operation_display    0x6061
#define  Position_actual_value         0x6063
#define  Velocity_sensor_actual_value  0x6069
#define  Velocity_actual_value         0x606C
#define  Target_torque                 0x6071
#define  Target_position               0x607A
#define  Profile_velocity              0x6081
#define  Profile_accleration           0x6083
#define  Profile_deceleration          0x6084
#define  Torque_slope                  0x6087
#define  Position_factor               0x6093
#define  Target_velocity               0x60FF


extern FDCAN_RxHeaderTypeDef rxHeader_hollysys;

extern s32 hollysys_spd[4];
extern s32 hollysys_pos[4];


void Hollysys_GetData(u8 can_id);
void Hollysys_Update(void);
void Hollysys_Setspdcmd(u8 can_id, s32 spd);
void Hollysys_CANopen_PV_setspd(u8 can_id, s32 spd);
void Hollysys_CANopen_PV_setdec(u8 can_id, u32 dec);
void Hollysys_CANopen_PV_setacc(u8 can_id, u32 acc);
void Hollysys_CANopen_PV_Init(u8 can_id);
void Hollysys_disable_B(u8 CANopen_ID);
void Hollysys_disable_A(u8 CANopen_ID);
void Hollysys_enable_B(u8 CANopen_ID);
void Hollysys_enable_A(u8 CANopen_ID);
void Hollysys_closedown(u8 CANopen_ID);
void Hollysys_startup(u8 CANopen_ID);


#endif
