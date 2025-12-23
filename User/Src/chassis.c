#include "chassis.h"



// orient front direction
// front left, front right, rear right, rear left
u8 steer_motor_id[4] = {0x05, 0x06, 0x07, 0x08};
// left front 0x01, right front 0x02, left rear 0x03, right rear 0x04
u8 drive_motor_id[4] = {0x01, 0x02, 0x03, 0x04};




s32 drive_motor_spd_cmd[4] = {0};
float steer_motor_pos_cmd[4] = {0};

float steer_motor_pos_max[4] = {0.0505455099, 0.0272755008, 0.0398641936, 0.0329976343};
// max -> min clock-wise
float steer_motor_pos_min[4] = {3.87140465, 3.84622717, 3.85271239, 3.85004187};

float steer_range_ = 3.8189516692;

float steer_forward_pos[4] = {1.8694208890999, 1.84615088, 1.8587395727999, 1.8518730134999};

void Chassis_Init(void){
  
  Hollysys_enable_A(0x03);
  Hollysys_enable_B(0x04);
  Hollysys_enable_A(0x01);
  Hollysys_enable_B(0x02);
  HAL_Delay(100);
  for(u8 i = 0; i < sizeof(drive_motor_id); i++){
    Hollysys_CANopen_PV_Init(drive_motor_id[i]);
  }
  HAL_Delay(100);
  for(u8 i = 0; i < sizeof(drive_motor_id); i++){
    Hollysys_startup(drive_motor_id[i]);
  }
  HAL_Delay(100);

  Motorevo_init();

  for(u8 i = 0; i < sizeof(steer_motor_id); i++){
    Motorevo_Reset(steer_motor_id[i]);
    Motorevo_Activate(steer_motor_id[i]);
  }

}

void Chassis_Set_drive_spd(){
  // for(u8 i = 0; i < 4; i++)
  Hollysys_Setspdcmd(drive_motor_id[0], drive_motor_spd_cmd[0]);
  Hollysys_Setspdcmd(drive_motor_id[1], -drive_motor_spd_cmd[1]);
  Hollysys_Setspdcmd(drive_motor_id[2], drive_motor_spd_cmd[2]);
  Hollysys_Setspdcmd(drive_motor_id[3], -drive_motor_spd_cmd[3]);
}

void Chassis_Drive_Info_Update(){
  for(u8 i = 0; i < sizeof(drive_motor_id); i++)
    Hollysys_GetData(drive_motor_id[i]);
}
void Chassis_Update(void){
  Hollysys_Update();
}

void Chassis_Deinit(void){
  HAL_Delay(50);
  for(u8 i = 0; i < sizeof(drive_motor_id); i++){
    Hollysys_closedown(drive_motor_id[i]);
  }
  Hollysys_disable_A(0x03);
  Hollysys_disable_B(0x04);
  Hollysys_disable_A(0x01);
  Hollysys_disable_B(0x02);
  HAL_Delay(50);

  for(u8 i = 0; i < sizeof(steer_motor_id); i++){
    Motorevo_Reset(steer_motor_id[i]);
  }
  HAL_Delay(50);
}


