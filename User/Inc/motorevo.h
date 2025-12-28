#ifndef MOTOREVO_H
#define MOTOREVO_H


#include "fdcan.h"
#include "common.h"

#define MIN_range 0
#define MAX_range 1

#define Reset_status   0x00
#define Motor_servo    0x01
#define Motor_pos_tor  0x02
#define Motor_vel      0x03
#define Motor_tor      0x04
#define Motor_tor4     0x05

#define MOTOREVO_NUM   4
#define MOTOREVO_START_ID 0x05

struct Receive_servo
{
  u8 status;
  u8 pos_h;
  u8 pos_l;
  u8 vec_h;
  u8 vec_l_tor_h;
  u8 tor_l;
  u8 error;
  u8 temp;
};

struct Motor_state{
  u8 status;
  float pos;
  float vec;
  float tor;
  u8 error;
  u8 temp;
};

extern struct Receive_servo motorevo_rec[MOTOREVO_NUM];
extern struct Motor_state motorevo_state[MOTOREVO_NUM];

void Motorevo_SetPos_cmd(u8 can_id, float pose, float velo);
void Motorevo_init();
void Motorevo_Update();
void Motorevo_Update_State();
void Motorevo_Reset(u8 can_id);
void Motorevo_Activate(u8 can_id);
void Motorevo_SetZero(u8 can_id);
void Motorevo_Query(u8 can_id);
void Motorevo_SetPos(u8 can_id, float pos, float vel);



#endif