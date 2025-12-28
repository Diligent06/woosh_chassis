/********************************************************************************
  
	* @file     
  * @author  luys 
  * @version V3.0.0
  * @date    06-20-2017
  * @brief   
	
*********************************************************************************/ 
/********************************************************************************
  
	* @    
  * @ 
  * @ 
  * @ 
  * @  
 
*********************************************************************************/ 

#include "hollysys.h"
#include "fdcan.h"

//速度模式PV：
u32 PV_spd;
u32 PP_spd;
u32 PT_spd;

// CAN send buffer 
u8 CAN1Sedbuf[8];
// CAN receive buffer
u8 hollysys_spd_rxbuf[HOLLYSYS_NUM][8];
u8 hollysys_pos_rxbuf[HOLLYSYS_NUM][8];


u8 hollysys_motor_id[HOLLYSYS_NUM] = {0x01, 0x02, 0x03, 0x04};

u32 hollysys_acc[HOLLYSYS_NUM] = {1000, 1000, 1000, 1000};
u32 hollysys_dec[HOLLYSYS_NUM] = {1000, 1000, 1000, 1000};
s32 hollysys_spd_cmd[HOLLYSYS_NUM] = {0, 0, 0, 0};

s32 hollysys_spd[HOLLYSYS_NUM];
s32 hollysys_pos[HOLLYSYS_NUM];

u8 need_pos = 0;

void Hollysys_Update_State(){
	for(u8 i = 0; i < HOLLYSYS_NUM; i++){
		hollysys_spd[i] = ((s32)hollysys_spd_rxbuf[i][7] << 24) | ((s32)hollysys_spd_rxbuf[i][6] << 16) | 
											((s32)hollysys_spd_rxbuf[i][5] << 8) | (s32)hollysys_spd_rxbuf[i][4];
		if(need_pos){
			hollysys_pos[i] = ((s32)hollysys_pos_rxbuf[i][7] << 24) | ((s32)hollysys_pos_rxbuf[i][6] << 16) | 
												((s32)hollysys_pos_rxbuf[i][5] << 8) | (s32)hollysys_pos_rxbuf[i][4];
		}
		
	}
}
u8 Hollysys_CAN_Send(u16 Id){
	txHeader.Identifier = Id;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, CAN1Sedbuf);
}

u16 Hollysys_Cal_F_Index(u8 x, u8 y, u8 z){
	return 0x2000 + 0x1000*x + 0x100*y + z;
}

u16 Hollysys_Cal_P_Index(u8 y, u8 z){
	return 0x4000 + 0x100*y + z;
}

u16 Hollysys_Cal_D_Index(u8 x, u8 y){
	return 0x5000 + 0x100*x + y;
}

void Hollysys_startup(u8 CANopen_ID){
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x40;
	CAN1Sedbuf[2] = 0x60;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x06;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
	CAN1Sedbuf[4] = 0x07;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
	CAN1Sedbuf[4] = 0x0f;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}

void Hollysys_closedown(u8 CANopen_ID){
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x40;
	CAN1Sedbuf[2] = 0x60;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x07;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}


// enable controller
void Hollysys_enable_A(u8 CANopen_ID){
	// axis 1
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x00;
	CAN1Sedbuf[2] = 0x21;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x01;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}

void Hollysys_enable_B(u8 CANopen_ID){
	// axis 2
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x00;
	CAN1Sedbuf[2] = 0x31;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x01;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}

// disable controller
void Hollysys_disable_A(u8 CANopen_ID){
	// axis 1
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x00;
	CAN1Sedbuf[2] = 0X21;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x00;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}

void Hollysys_disable_B(u8 CANopen_ID){
	// axis 2
	CAN1Sedbuf[0] = 0x2b;
	CAN1Sedbuf[1] = 0x00;
	CAN1Sedbuf[2] = 0X31;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = 0x00;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	Hollysys_CAN_Send(0x600 + CANopen_ID);
}


/*模式设置*/
u8 Hollysys_Contol_Mode_SET(u8 CANopen_ID,u8 CANopen_mode){
	 
	 CAN1Sedbuf[0]=SDO_W1;
	 CAN1Sedbuf[1]=0x60;
   CAN1Sedbuf[2]=0x60;
	 CAN1Sedbuf[3]=0x00;
	 CAN1Sedbuf[4]=CANopen_mode;
	 CAN1Sedbuf[5]=0x00;
	 CAN1Sedbuf[6]=0x00;
	 CAN1Sedbuf[7]=0x00;	
	 Hollysys_CAN_Send(0x600 + CANopen_ID);
	 return(1);
} 

/*激活节点*/
u8 Hollysys_CANopen_Activate(u8 CANopen_ID){
	 
	 CAN1Sedbuf[0]=0x01;
	 CAN1Sedbuf[1]=CANopen_ID;	
	 Hollysys_CAN_Send(0x000);
	 return(1);
} 

u8 Hollysys_CAN_lifeguard(u8 CANopen_ID){
	memset(CAN1Sedbuf, 0, sizeof(CAN1Sedbuf));
	Hollysys_CAN_Send(0x700 + CANopen_ID);
}

u8 Hollysys_SDO_Write_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA){
   CAN1Sedbuf[0]=CMD;
	 CAN1Sedbuf[1]=(u8)(Index    & 0xFF);
   CAN1Sedbuf[2]=(u8)(Index>>8 & 0xFF);
	 CAN1Sedbuf[3]=SubIndex;
	 CAN1Sedbuf[4]=(u8)(DATA     & 0xFF);
	 CAN1Sedbuf[5]=(u8)(DATA>>8  & 0xFF);
	 CAN1Sedbuf[6]=(u8)(DATA>>16 & 0xFF);
	 CAN1Sedbuf[7]=(u8)(DATA>>24 & 0xFF);	
	 Hollysys_CAN_Send(0x600 + CANopen_ID);
   return(1);
}

u8 Hollysys_SDO_Read_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex){
  CAN1Sedbuf[0] = CMD;
	CAN1Sedbuf[1] = Index & 0xff;
	CAN1Sedbuf[2] = (Index >> 8) & 0xff;
	CAN1Sedbuf[3] = SubIndex;
	memset(&CAN1Sedbuf[4], 0, 4);
	Hollysys_CAN_Send(0x600 + CANopen_ID);
  return(1);
}

//PV canopen设置
void Hollysys_CANopen_PV_Init(u8 can_id){
	
  Hollysys_CANopen_Activate(can_id);
	 
	Hollysys_Contol_Mode_SET(can_id,  PV_Mode );
	 
	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x6083,0x00,hollysys_acc[can_id - 1]);
	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x6084,0x00,hollysys_dec[can_id - 1]);

	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x60FF,0x00,hollysys_spd_cmd[can_id - 1]);
}

void Hollysys_CANopen_PV_setacc(u8 can_id, u32 acc){
	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x6083,0x00,acc);
}

void Hollysys_CANopen_PV_setdec(u8 can_id, u32 dec){
	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x6084,0x00,dec);
}
void Hollysys_CANopen_PV_setspd(u8 can_id, s32 spd){
	Hollysys_SDO_Write_OD(can_id, SDO_W4, 0x60FF,0x00,spd);
}

void Hollysys_Setspdcmd(u8 can_id, s32 spd){
	hollysys_spd_cmd[can_id - 1] = spd;
}

void Hollysys_Update(void){
	for(u8 i = 0; i < sizeof(hollysys_motor_id); i++){
		Hollysys_CANopen_PV_setspd(hollysys_motor_id[i], hollysys_spd_cmd[i]);
	}
}

void Hollysys_GetData(u8 can_id){
	CAN1Sedbuf[0] = 0x40;
	CAN1Sedbuf[1] = 0x69;
	CAN1Sedbuf[2] = 0x60;
	CAN1Sedbuf[3] = 0x00;
	memset(&CAN1Sedbuf[4], 0, 4);
	Hollysys_CAN_Send(0x600 + can_id); // get speed
	if(need_pos){
		CAN1Sedbuf[1] = 0x63;
		Hollysys_CAN_Send(0x600 + can_id); // get position
	}
}












































// //PP canopen设置
// void CANopen_PP_Init(void){
	
// //STEP1:激活节点1、节点2
// 	 CANopen_Activate(1);
// 	 CANopen_Activate(2);
	 
// //STEP2:设置位置模式	6060H写为1 
// 	 Contol_Mode_SET(Left_Wheel_ID,PP_Mode);
// 	 Contol_Mode_SET(Right_Wheel_ID,PP_Mode);
	 
// //STEP3:设置目标脉冲	写607AH	10000,常规电机转1圈	
	
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x607A,0x00,0);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x607A,0x00,0);

// //STEP4:设置目标转速为10rpm	写6081H	 
	
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6081,0x00,0);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6081,0x00,0);

// //STEP5:设置加减速	写6083H和6084H	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6083,0x00,0x03E8);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6083,0x00,0x03E8);
	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6084,0x00,0x03E8);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6084,0x00,0x03E8);
	 
// //STEP6:设置电子齿轮比 分子分母 	写6093H的sub1和sub2	 
//    SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6093,0x01,1);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6093,0x01,1);
	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6093,0x02,1);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6093,0x02,1);	 
 
// }

// void CANopen_PP_Set(s32 TargetPosition,u32 ProfileVelocity){
	
// //STEP1:设置目标脉冲	写607AH	TargetPosition 	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x607A,0x00,TargetPosition);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x607A,0x00,TargetPosition);

// //STEP2:设置目标转速为	写6081H	 ProfileVelocity
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6081,0x00,ProfileVelocity);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6081,0x00,ProfileVelocity);

// }

// void Motor_PP_Trigger(void){
	
//    //STEP1:6040 bits4清0 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6040,0x00,0x0F);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6040,0x00,0x0F);
	 
// 	  //STEP2:置bits4为1，电机运动 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6040,0x00,0x5F);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6040,0x00,0x5F);
	 
// 	  //STEP3:6040 bits4清0 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6040,0x00,0x4F);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6040,0x00,0x4F);

// }



// //PT canopen设置
// void CANopen_PT_Init(void){
	
// //STEP1:激活节点1、节点2
// 	 CANopen_Activate(1);
// 	 CANopen_Activate(2);
	 
// //STEP2:设置转矩模式	6060H写为4 
//    Contol_Mode_SET(Left_Wheel_ID,PT_Mode);
// 	 Contol_Mode_SET(Right_Wheel_ID,PT_Mode);
	 
// //STEP3:设置加减速	写6087H	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6087,0x00,0x03e8);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6087,0x00,0x03e8);
	 
// //STEP4:设置目标转矩为0	写6071H	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6071,0x00,0);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6071,0x00,0);
 
// }

// void CANopen_PT_Set(s16 TargetTorque){
// 	//STEP1:设置目标转矩为0	写6071H	 
// 	 SDO_Write_OD(Left_Wheel_ID,SDO_W4,0x6071,0x00,TargetTorque);
// 	 SDO_Write_OD(Right_Wheel_ID,SDO_W4,0x6071,0x00,TargetTorque);

// }
