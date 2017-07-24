#ifndef ROBOT_7DOF_FB__H
#define ROBOT_7DOF_FB__H


#include "Matrix.h"
#include "modbus.h"




#define DEBUG 1

#if (DEBUG)
	#define DBGMSG(x)  printf x;
#else
    #define DBGMSG(x)
#endif

//==========
//==MAX AXIS
//==========
#define MAX_AXIS_NUM  7
#define DEF_RIGHT_HAND 1
#define DEF_LEFT_HAND 2


//==========
//==Axis frame
//==========
#define DEF_X 0
#define DEF_Y 1
#define DEF_Z 2

#define DEF_ALPHA 0
#define DEF_BETA 1
#define DEF_GAMMA 2
//===========================
//==Axis index,ID,NO mapping
//==========================
//enum{
//	ID_AXIS1=1,
//	ID_AXIS2,
//	ID_AXIS3,
//	ID_AXIS4,
//	ID_AXIS5,
//	ID_AXIS6,
//	ID_AXIS7
//};

//enum{
//	NO_AXIS1=1,
//	NO_AXIS2,
//	NO_AXIS3,
//	NO_AXIS4,
//	NO_AXIS5,
//	NO_AXIS6,
//	NO_AXIS7
//};
//
//
//static const unsigned char gMapAxisNO[MAX_AXIS_NUM]=
//{
//	NO_AXIS1,
//	NO_AXIS2,
//	NO_AXIS3,
//	NO_AXIS4,
//	NO_AXIS5,
//	NO_AXIS6,
//	NO_AXIS7
//};
//static const unsigned char gMapAxisID[MAX_AXIS_NUM]=
//{
//	ID_AXIS1,
//	ID_AXIS2,
//	ID_AXIS3,
//	ID_AXIS4,
//	ID_AXIS5,
//	ID_AXIS6,
//	ID_AXIS7
//};

enum{
	Index_AXIS1=0,
	Index_AXIS2,
	Index_AXIS3,
	Index_AXIS4,
	Index_AXIS5,
	Index_AXIS6,
	Index_AXIS7,
};

enum{
	ID_RAXIS1=1,
	ID_RAXIS2,
	ID_RAXIS3,
	ID_RAXIS4,
	ID_RAXIS5,
	ID_RAXIS6,
	ID_RAXIS7,
	ID_LAXIS1,
	ID_LAXIS2,
	ID_LAXIS3,
	ID_LAXIS4,
	ID_LAXIS5,
	ID_LAXIS6,
	ID_LAXIS7
};

enum{
	NO_AXIS1=1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};


static const unsigned char gMapAxisNO[MAX_AXIS_NUM]=
{
	NO_AXIS1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};


static const unsigned char gMapRAxisID[MAX_AXIS_NUM]=
{
	ID_RAXIS1,
	ID_RAXIS2,
	ID_RAXIS3,
	ID_RAXIS4,
	ID_RAXIS5,
	ID_RAXIS6,
	ID_RAXIS7
};

static const unsigned char gMapLAxisID[MAX_AXIS_NUM]=
{
	ID_LAXIS1,
	ID_LAXIS2,
	ID_LAXIS3,
	ID_LAXIS4,
	ID_LAXIS5,
	ID_LAXIS6,
	ID_LAXIS7
};


//================
//==Unit transform
//=================
#define DEF_PI (3.1415926F)
#define DEF_RATIO_PUS_TO_DEG (0.0879F)		//360/4096
#define DEF_RATIO_PUS_TO_RAD (0.0015F)		//2pi/4096   0.00153398078788564122971808758949
#define DEF_RATIO_DEG_TO_PUS (11.3778F)		//4096/360
#define DEF_RATIO_DEG_TO_RAD (DEF_PI/180)		//pi/180	0.01745329251994329576923690768489 (0.0175F)
#define DEF_RATIO_RAD_TO_DEG (180/DEF_PI)	
#define DEF_RATIO_RAD_TO_PUS (651.8986F)	//4096/2pi	651.89864690440329530934789477382
#define DEF_RATIO_VEL_PUS_TO_DEG (0.684F)  //moving speed in register(0~1024) to deg/s
#define DEF_RATIO_VEL_DEG_TO_PUS (1.462)

//for read pos unit select
enum{
	DEF_UNIT_RAD=1,
	DEF_UNIT_DEG,
	DEF_UNIT_PUS
};

//=====================================
//==robot hard ware dependent parameter
//=====================================
//==robot to Motor offset==//  //robot pos=motor position -M2R_OFFSET
//right
#define AXISR1_R2M_OFFSET_DEG 180
#define AXISR2_R2M_OFFSET_DEG 270
#define AXISR3_R2M_OFFSET_DEG 180
#define AXISR4_R2M_OFFSET_DEG 180
#define AXISR5_R2M_OFFSET_DEG 180
#define AXISR6_R2M_OFFSET_DEG 180
#define AXISR7_R2M_OFFSET_DEG 180
//left
#define AXISL1_R2M_OFFSET_DEG 180
#define AXISL2_R2M_OFFSET_DEG 90
#define AXISL3_R2M_OFFSET_DEG 180
#define AXISL4_R2M_OFFSET_DEG 90
#define AXISL5_R2M_OFFSET_DEG 90
#define AXISL6_R2M_OFFSET_DEG 180
#define AXISL7_R2M_OFFSET_DEG 180

//==robot angle limit==//
//right
#define AXISR1_ROBOT_LIM_DEG_L (-80)
#define AXISR1_ROBOT_LIM_DEG_H 170
#define AXISR2_ROBOT_LIM_DEG_L (-180)
#define AXISR2_ROBOT_LIM_DEG_H 10
#define AXISR3_ROBOT_LIM_DEG_L (-105)
#define AXISR3_ROBOT_LIM_DEG_H 170
#define AXISR4_ROBOT_LIM_DEG_L 0	
#define AXISR4_ROBOT_LIM_DEG_H 170	
#define AXISR5_ROBOT_LIM_DEG_L (-100)
#define AXISR5_ROBOT_LIM_DEG_H 90
#define AXISR6_ROBOT_LIM_DEG_L (-21)
#define AXISR6_ROBOT_LIM_DEG_H 110
#define AXISR7_ROBOT_LIM_DEG_L (-170)
#define AXISR7_ROBOT_LIM_DEG_H 170
//left
#define AXISL1_ROBOT_LIM_DEG_L (-170)
#define AXISL1_ROBOT_LIM_DEG_H 80
#define AXISL2_ROBOT_LIM_DEG_L (-10)
#define AXISL2_ROBOT_LIM_DEG_H 180
#define AXISL3_ROBOT_LIM_DEG_L (-170)
#define AXISL3_ROBOT_LIM_DEG_H 105
#define AXISL4_ROBOT_LIM_DEG_L 0	
#define AXISL4_ROBOT_LIM_DEG_H 170	
#define AXISL5_ROBOT_LIM_DEG_L (-90)
#define AXISL5_ROBOT_LIM_DEG_H 100
#define AXISL6_ROBOT_LIM_DEG_L (-21)
#define AXISL6_ROBOT_LIM_DEG_H 110
#define AXISL7_ROBOT_LIM_DEG_L (-170)
#define AXISL7_ROBOT_LIM_DEG_H 170

//==robot TORQUE limit==//  0~1023
//right
#define AXISR1_MAX_TORQUE 928	//90%
#define AXISR2_MAX_TORQUE 430   //42%	
#define AXISR3_MAX_TORQUE 923	//90%
#define AXISR4_MAX_TORQUE 926	//90%
#define AXISR5_MAX_TORQUE 514	//50%
#define AXISR6_MAX_TORQUE 519	//50%
#define AXISR7_MAX_TORQUE 517	//50%
//left
#define AXISL1_MAX_TORQUE 928	//90%
#define AXISL2_MAX_TORQUE 430   //42%	
#define AXISL3_MAX_TORQUE 923	//90%
#define AXISL4_MAX_TORQUE 926	//90%
#define AXISL5_MAX_TORQUE 514	//50%
#define AXISL6_MAX_TORQUE 519	//50%
#define AXISL7_MAX_TORQUE 517	//50%

//==PID P gain==//
#define AXISR1_P_GAIN	40		
#define AXISR2_P_GAIN	32
#define AXISR3_P_GAIN	32
#define AXISR4_P_GAIN	32
#define AXISR5_P_GAIN	32
#define AXISR6_P_GAIN	32
#define AXISR7_P_GAIN	32

#define AXISL1_P_GAIN	40		
#define AXISL2_P_GAIN	32
#define AXISL3_P_GAIN	32
#define AXISL4_P_GAIN	32
#define AXISL5_P_GAIN	32
#define AXISL6_P_GAIN	32
#define AXISL7_P_GAIN	32

//==PID I gain==//
#define AXISR1_I_GAIN	20		
#define AXISR2_I_GAIN	15
#define AXISR3_I_GAIN	15
#define AXISR4_I_GAIN	15
#define AXISR5_I_GAIN	15
#define AXISR6_I_GAIN	15
#define AXISR7_I_GAIN	15

#define AXISL1_I_GAIN	20	
#define AXISL2_I_GAIN	15
#define AXISL3_I_GAIN	15
#define AXISL4_I_GAIN	15
#define AXISL5_I_GAIN	15
#define AXISL6_I_GAIN	15
#define AXISL7_I_GAIN	15

//==PID D gain==//
#define AXISR1_D_GAIN	10		
#define AXISR2_D_GAIN	10
#define AXISR3_D_GAIN	10
#define AXISR4_D_GAIN	10
#define AXISR5_D_GAIN	10
#define AXISR6_D_GAIN	10
#define AXISR7_D_GAIN	10

#define AXISL1_D_GAIN	10		
#define AXISL2_D_GAIN	10
#define AXISL3_D_GAIN	10
#define AXISL4_D_GAIN	10
#define AXISL5_D_GAIN	10
#define AXISL6_D_GAIN	10
#define AXISL7_D_GAIN	10

//==define right hand or left hand==//
#define DEF_RIGHT_HAND	1
#define DEF_LEFT_HAND	2

static const unsigned short int gr2m_offset_pulse_R[MAX_AXIS_NUM]=
{
	(unsigned short int)(AXISR1_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR2_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR3_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR4_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR5_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR6_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR7_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS)
};

static const unsigned short int gr2m_offset_pulse_L[MAX_AXIS_NUM]=
{
	(unsigned short int)(AXISL1_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL2_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL3_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL4_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL5_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL6_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL7_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS)
};

//right hand
static const float grobot_lim_rad_R_Low[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_rad_R_High[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_pus_R_Low[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS
};

static const float grobot_lim_pus_R_High[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS
};

//left hand
static const float grobot_lim_rad_L_Low[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_rad_L_High[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_pus_L_Low[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS
};

static const float grobot_lim_pus_L_High[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS
};


//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0
//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0

//===================
//==ROBOT link length
//====================
#define L0 225    	//head to shoulder
#define L1 250    	//L-type linker long side
#define L2 25   	//L-type linker short side
#define L3 25     	//L-type linker short side
#define L4 230     	//L-type linker long side
#define L5 180     	//length from wrist to end effector
#define X_BASE 0  	//膀非翴常砞0
#define Y_BASE 0
#define Z_BASE 0

#define DEF_VERY_SMALL (1.e-4)//秖耞0ㄏノ

//==========
//Function
//==========
unsigned char getMapAxisNO(unsigned char index); //index 0~ (MAX_AXIS_NUM-1)
unsigned char getMapAxisID(unsigned char index);
int ROM_Setting_Dual();
void PID_Setting_Dual();
int Read_pos(int RLHand,float *pos,unsigned char unit);
int Output_to_Dynamixel(int RLHand,const float *Ang_rad,const unsigned short int *velocity) ;
int Output_to_Dynamixel_Dual(const float *Ang_rad_R,const unsigned short int *velocity_R,const float *Ang_rad_L,const unsigned short int *velocity_L);
int Output_to_Dynamixel_pulse(const unsigned short int *Ang_pulse,const unsigned short int *velocity);

Matrix R_z1x2y3(float alpha,float beta,float gamma);
float norm(const Matrix& v);
Matrix Rogridues(float theta,const Matrix& V_A);
int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta);
int IK_7DOF_FB7roll(int RLHand,const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta);
bool AngleOverConstrain(int RLHand,const float theta[MAX_AXIS_NUM],int *OverIndex);
int MoveToPoint(int RLHand,float Pend[3],float Pose_deg[3],float redant_alpha_deg,float vel_deg);
int MoveToPoint_Dual(float Pend_R[3],float Pose_deg_R[3],float Rednt_alpha_deg_R,float vel_deg_R,float Pend_L[3],float Pose_deg_L[3],float Rednt_alpha_deg_L,float vel_deg_L);  //莱赣璶Τ硉把计
int IsMoving(int RLHand,bool *stillmoving);
void QPDelay_ms(int t_ms);

//dynamixel use
int syncWrite_x86(unsigned short int start_addr, unsigned short int data_length, unsigned short int *param, unsigned short int param_length); // WORD(16bit) syncwrite() for DXL
int setPosition_x86(int ServoID, int Position, int Speed);
int DXL_Initial_x86();
int DXL_Terminate_x86();


//Modbus control gripper
#ifdef MODBUS_GRIPPER
int Initial_Modbus();
void Terminate_Modbus();
int GripperHold(int RLHand,bool Hold);
#endif

int Gripper_LattePanda_Initial();
void Gripper_LattePanda_Close();
int Gripper_LattePanda_Hold(int RLHand,bool Hold);
#endif    /* ROBOT_7DOF_FB__H */