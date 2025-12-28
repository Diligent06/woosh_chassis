#include "motorevo.h"
#include "string.h"


struct Receive_servo motorevo_rec[MOTOREVO_NUM];
struct Motor_state motorevo_state[MOTOREVO_NUM];

u8 motorevo_sed_buf[8] = {0};

u8 motorevo_reset_buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
u8 motorevo_activate_buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
u8 motorevo_set_zero_buf[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};


u8 motorevo_id[MOTOREVO_NUM] = {0x05, 0x06, 0x07, 0x08};

float pos_range[2] = {-12.5, 12.5};
float vel_range[2] = {-30.0, 30.0};
float tor_range[2] = {-50.0, 50.0};
float pos_kp_range[2] = {0.0, 250.0};
float pos_kd_range[2] = {0.0, 50.0};
float vel_kp_range[2] = {0.0, 250.0};
float vel_kd_range[2] = {0.0, 50.0};
float vel_ki_range[2] = {0.0, 0.05};

float motorevo_pos_cmd[MOTOREVO_NUM] = {0};
float motorevo_vel_cmd[MOTOREVO_NUM] = {0};

float pos_min[MOTOREVO_NUM] = {0};
float pos_max[MOTOREVO_NUM] = {0};
float pos_front[MOTOREVO_NUM] = {0};


float motorevo_pos_kp = 15.0;
float motorevo_pos_kd = 4.5;
float motorevo_vel_kp = 50.0;
float motorevo_vel_kd = 0.0;
float motorevo_vel_ki = 0.001;

u8 pos_kp_command;
u8 pos_kd_command;
u8 vel_kp_command;
u8 vel_ki_command;
u8 vel_kd_command;

float map_u8_to_float(u8 x, float range_min, float range_max){
  return (float)x * (range_max - range_min) / 255.0 + range_min;
}
float map_u16_to_float(u16 x, float range_min, float range_max){
  return (float)x * (range_max - range_min) / 65535.0 + range_min;
}

float map_u12_to_float(u16 x, float range_min, float range_max){
  return (float)x * (range_max - range_min) / 4095.0 + range_min;
}

u8 map_float_to_u8(float x, float range_min, float range_max){
  if(x < range_min) return 0;
  else if(x > range_max) return 255;

  return (u8)((x - range_min) * 255.0 / (range_max - range_min));
}

u16 map_float_to_u16(float x, float range_min, float range_max){
  if(x < range_min) return 0;
  else if(x > range_max) return 65535;

  return (u16)((x - range_min) * 65535.0 / (range_max - range_min));
}


void Motorevo_init(){
  pos_kp_command = map_float_to_u8(motorevo_pos_kp, pos_kp_range[MIN_range], pos_kp_range[MAX_range]);
  pos_kd_command = map_float_to_u8(motorevo_pos_kd, pos_kd_range[MIN_range], pos_kd_range[MAX_range]);
  vel_kp_command = map_float_to_u8(motorevo_vel_kp, vel_kp_range[MIN_range], vel_kp_range[MAX_range]);
  vel_kd_command = map_float_to_u8(motorevo_vel_kd, vel_kd_range[MIN_range], vel_kd_range[MAX_range]);
  vel_ki_command = map_float_to_u8(motorevo_vel_ki, vel_ki_range[MIN_range], vel_ki_range[MAX_range]);
  motorevo_sed_buf[3] = pos_kp_command;
  motorevo_sed_buf[4] = pos_kd_command;
  motorevo_sed_buf[5] = vel_kp_command;
  motorevo_sed_buf[6] = vel_kd_command;
  motorevo_sed_buf[7] = vel_ki_command;
}

void Motorevo_CAN_Send(u16 can_id, u8* buf){
  txHeader.Identifier = can_id;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, buf);
}

void Motorevo_SetPos(u8 can_id, float pos, float vel){
  u16 pos_command = map_float_to_u16(pos, pos_range[MIN_range], pos_range[MAX_range]);
  u8 vel_command = map_float_to_u8(vel, vel_range[MIN_range], vel_range[MAX_range]);
  motorevo_sed_buf[0] = pos_command >> 8;
  motorevo_sed_buf[1] = pos_command & 0xFF;
  motorevo_sed_buf[2] = vel_command;
  Motorevo_CAN_Send(can_id, motorevo_sed_buf);
}
void Motorevo_Update(){
  for(u8 i = 0; i < sizeof(motorevo_id); i++){
    Motorevo_SetPos(motorevo_id[i], motorevo_pos_cmd[i], motorevo_vel_cmd[i]);
  }
}

void Motorevo_SetPos_cmd(u8 can_id, float pose, float velo){
  motorevo_pos_cmd[can_id - 0x05] = pose;
  motorevo_vel_cmd[can_id - 0x05] = velo;
}

void Motorevo_Update_State(){
  for(u8 i = 0; i < sizeof(motorevo_id); i++){
    motorevo_state[i].error = motorevo_rec[i].error;
    motorevo_state[i].temp = motorevo_rec[i].temp;
    motorevo_state[i].status = motorevo_rec[i].status;
    motorevo_state[i].pos = map_u16_to_float((u16)motorevo_rec[i].pos_h << 8 | motorevo_rec[i].pos_l, pos_range[MIN_range], pos_range[MAX_range]);
    motorevo_state[i].vec = map_u12_to_float((u16)motorevo_rec[i].vec_h << 4 | motorevo_rec[i].vec_l_tor_h >> 4, vel_range[MIN_range], vel_range[MAX_range]);
    motorevo_state[i].tor = map_u12_to_float((u16)(motorevo_rec[i].vec_l_tor_h & 0x0F) << 8 | motorevo_rec[i].tor_l, tor_range[MIN_range], tor_range[MAX_range]);
  }
}

void Motorevo_Reset(u8 can_id){
  Motorevo_CAN_Send(can_id, motorevo_reset_buf);
}
void Motorevo_Activate(u8 can_id){
  Motorevo_CAN_Send(can_id, motorevo_activate_buf);
}

void Motorevo_SetZero(u8 can_id){
  Motorevo_CAN_Send(can_id, motorevo_set_zero_buf);
}

void Motorevo_Query(u8 can_id){
  memset(motorevo_sed_buf, 0, sizeof(motorevo_sed_buf));
  Motorevo_CAN_Send(can_id, motorevo_sed_buf);
}



