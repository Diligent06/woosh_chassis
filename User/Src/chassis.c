#include "chassis.h"
#include <math.h>



// orient front direction
// front left, front right, rear right, rear left
u8 steer_motor_id[STEER_MOTOR_NUM] = {0x05, 0x06, 0x07, 0x08};
// left front 0x01, right front 0x02, left rear 0x03, right rear 0x04
u8 drive_motor_id[DRIVE_MOTOR_NUM] = {0x01, 0x02, 0x03, 0x04};



float steer_motor_pos_max[STEER_MOTOR_NUM] = {0.0505455099, 0.0272755008, 0.0398641936, 0.0329976343};
// max -> min clock-wise
float steer_motor_pos_min[STEER_MOTOR_NUM] = {3.87140465, 3.84622717, 3.85271239, 3.85004187};
float steer_range_ = 3.8189516692;
// float steer_forward_pos[STEER_MOTOR_NUM] = {1.8694208890999, -1.88467991, 1.8587395727999, -0.648317695};
float steer_forward_pos[STEER_MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0};
float steer_vel_limit = 5.0;
float steer_clip_angle = pi / 2.0;
float steer_front_angle = pi / 2.0;

// float steer_motor_pos_cmd[STEER_MOTOR_NUM] = {1.8694208890999, -1.88467991, 1.8587395727999, -0.648317695};
float steer_motor_pos_cmd[STEER_MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0};
float steer_motor_ang_cmd[STEER_MOTOR_NUM] = {pi / 2, pi / 2, pi / 2, pi / 2};

u8 steer_decrease_flag = 0;

s32 drive_motor_spd_cmd[DRIVE_MOTOR_NUM] = {0};

float hx = 0.3;    // width of chassis
float hy = 0.5;    // lenght of chassis
float bx = 0.0;    // offset of steer and motor center
float by = 0.0;   // offset of steer and motor center

float sign_hx[STEER_MOTOR_NUM] = {-1, 1, 1, -1};
float sign_hy[STEER_MOTOR_NUM] = {1, 1, -1, -1};

u8 steer_to_drive[STEER_MOTOR_NUM] = {0, 1, 3, 2};


void Chassis_Init(void){
  
  Hollysys_enable_A(0x03);
  Hollysys_enable_B(0x04);
  Hollysys_enable_A(0x01);
  Hollysys_enable_B(0x02);
  HAL_Delay(50);
  for(u8 i = 0; i < sizeof(drive_motor_id); i++){
    Hollysys_CANopen_PV_Init(drive_motor_id[i]);
  }
  HAL_Delay(50);
  for(u8 i = 0; i < sizeof(drive_motor_id); i++){
    Hollysys_startup(drive_motor_id[i]);
  }
  HAL_Delay(50);

  Motorevo_init();

  for(u8 i = 0; i < sizeof(steer_motor_id); i++){
    Motorevo_Reset(steer_motor_id[i]);
    Motorevo_Activate(steer_motor_id[i]);
  }

  HAL_Delay(50);
  Chassis_Info_Query();
  // Chassis_Steer_Info_Query();
  Motorevo_Update_State();
  HAL_Delay(50);
}

void Chassis_drive_spd_decrease(){
  for(u8 i = 0; i < STEER_MOTOR_NUM; i++){
    if(fabs(motorevo_state[i].pos - steer_motor_pos_cmd[i]) > 0.5){
      steer_decrease_flag = 1;
      break;
    }
  }
}
void Chassis_Set_drive_spd(){
  if(steer_decrease_flag){
    Hollysys_Setspdcmd(drive_motor_id[0], drive_motor_spd_cmd[0]*0.1);
    Hollysys_Setspdcmd(drive_motor_id[1], -drive_motor_spd_cmd[1]*0.1);
    Hollysys_Setspdcmd(drive_motor_id[2], drive_motor_spd_cmd[2]*0.1); 
    Hollysys_Setspdcmd(drive_motor_id[3], -drive_motor_spd_cmd[3]*0.1);
  }
  else{
    Hollysys_Setspdcmd(drive_motor_id[0], drive_motor_spd_cmd[0]);
    Hollysys_Setspdcmd(drive_motor_id[1], -drive_motor_spd_cmd[1]);
    Hollysys_Setspdcmd(drive_motor_id[2], drive_motor_spd_cmd[2]);
    Hollysys_Setspdcmd(drive_motor_id[3], -drive_motor_spd_cmd[3]);
  }
  steer_decrease_flag = 0;
}

float Chassis_Steer_Angle_Clip(float* angle, float* center_ang){
  if(*angle < *center_ang - steer_clip_angle){
    return *center_ang - steer_clip_angle;
  }
  else if (*angle > *center_ang + steer_clip_angle){
    return *center_ang + steer_clip_angle;
  }
  else{
    return *angle;
  }
}

/*        Set chassis steer angle in base coordinate (x is right and y is front)          */
// steer pos minus and motor rotate by inverse clock-wise direction
void Chassis_Set_steer_angle(float* angle){
  for(u8 i = 0; i < STEER_MOTOR_NUM; i++){
    steer_motor_ang_cmd[i] = Chassis_Steer_Angle_Clip(&angle[i], &steer_front_angle);
    steer_motor_pos_cmd[i] = -(steer_motor_ang_cmd[i] - steer_front_angle) + steer_forward_pos[i];
  }
}

/*        Set chassis steer angle by motor pos                */
void Chassis_Set_steer_pos(){ 
  Motorevo_SetPos_cmd(steer_motor_id[0], steer_motor_pos_cmd[0], steer_vel_limit);
  Motorevo_SetPos_cmd(steer_motor_id[1], steer_motor_pos_cmd[1], steer_vel_limit);
  Motorevo_SetPos_cmd(steer_motor_id[2], steer_motor_pos_cmd[2], steer_vel_limit);
  Motorevo_SetPos_cmd(steer_motor_id[3], steer_motor_pos_cmd[3], steer_vel_limit);
}

void Chassis_steer_pos_from_angle(){
  for(u8 i = 0; i < STEER_MOTOR_NUM; i++){
    steer_motor_pos_cmd[i] = -(steer_motor_ang_cmd[i] - steer_front_angle) + steer_forward_pos[i];
  }
}

void Chassis_State_Update(){
  Hollysys_Update_State();
  Motorevo_Update_State();
}


void Chassis_Info_Query(){
  Chassis_Drive_Info_Query();
  // Chassis_Steer_Info_Query();
}
void Chassis_Steer_Info_Query(){
  for(u8 i = 0; i < sizeof(steer_motor_id); i++)
    Motorevo_Query(steer_motor_id[i]);

}

void Chassis_Drive_Info_Query(){
  for(u8 i = 0; i < sizeof(drive_motor_id); i++)
    Hollysys_GetData(drive_motor_id[i]);
}

// Chassis motor output update
void Chassis_Update(void){

  Hollysys_Update();
  Motorevo_Update();
}

float Chassis_Convert_steer_pos_to_angle(u8 motor_no, float pos){
  return -(pos - steer_forward_pos[motor_no]) + steer_front_angle;
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

// spd_w inverse clock-wise direction is positive
void Chassis_Tidybot(float spd_x, float spd_y, float spd_w){
  for(u8 i = 0; i < sizeof(STEER_MOTOR_NUM); i++){
    float steer_angle = Chassis_Convert_steer_pos_to_angle(i, motorevo_state[i].pos); 
    float angle_sin = (float)sin(steer_angle);
    float angle_cos = (float)cos(steer_angle);
    float temp_x = sign_hx[i] * hx - bx * angle_cos + by * angle_sin;
    float temp_y = sign_hy[i] * hy - bx * angle_sin - by * angle_cos;

    float spd_w_x = -temp_y * spd_w;
    float spd_w_y = temp_x * spd_w;
    
    float final_x = spd_x + spd_w_x;
    float final_y = spd_y + spd_w_y;

    float total_spd = sqrt(final_x * final_x + final_y * final_y);
    float total_ang = atan2(final_y, final_x); // -pi, pi

    float final_total_spd = total_ang < 0 ? -total_spd : total_spd;
    float final_total_ang = total_ang < 0 ? total_ang + pi : total_ang;

    drive_motor_spd_cmd[steer_to_drive[i]] = (s32)final_total_spd;
    steer_motor_ang_cmd[i] = final_total_ang;
  }
  Chassis_steer_pos_from_angle();
}

