#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "pid.h"
#include "Moto.h"


//typedef enum
//{
//	GIMBAL_EMPTY = 0,
//	GIMBAL_NO_FORCE,
//  GIMBAL_RC,
//	GIMBAL_KEYBOARD,
//} gimbal_mode_e;

//typedef struct
//{
//	//�ٶȻ�pid
//  pid_type_def gimbal_roll_pid;
//  //�ǶȻ�pid
//  pid_type_def gimbal_roll_angelpid;
//	fp32 gimbal_roll_speed;
//	fp32 gimbal_roll_speed_set;
//	fp32 gimbal_roll_angle;
//	fp32 gimbal_roll_angle_set;
//	
//	fp32 rc_gimbal_roll;
//	fp32 gimbal_roll_current;
//	fp32 rc_gimbal_pitch;
//	
//	
//	fp32 gimbal_roll_angle_mileage;
//	fp32 gimbal_roll_mileage;
//	int32_t gimbal_roll_limit_flag;
//	fp32 gimbal_roll_limit_max;
//	fp32 gimbal_roll_interval;
////	fp32 gimbal_roll_
////	fp32 gimbal_roll_
////	fp32 
//	
//} gimbal_move_t;


//typedef struct
//{
//	
//	fp32 gimbal_roll_;
////	fp32 gimbal_roll_
////	fp32 
//	
//} gimbal_mo_t;
////ǰ��
//#define GIMBAL_2006_P				5000.0f	//����3508
//#define GIMBAL_2006_I				.0f
//#define GIMBAL_2006_D				0.0f
////����
//#define GIMBAL_2006_ANGEL_PID_KP				22.0f	//����pitch 3508 �ٶȻ�
//#define GIMBAL_2006_ANGEL_PID_KI				0.0f
//#define GIMBAL_2006_ANGEL_PID_KD				0.3f
//#define M2006_MAX_OUT		10000.0f
//#define M2006_MAX_IOUT	5000.0f



//#define GIMBAL_RC_MODE_S2 0
//#define GIMBAL_RC_MODE_S1 1

//#define GIMBAL_RC_M2006 2
//#define GIMBAL_RC_DEADLINE 25

//#define M2006_MOTOR_RPM_TO_ACTION				0.000415809748903494517209f	//m3508ת���ɵ����ٶ�(m/s)�ı���
//#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_ACTION

extern void GimbalControlTask_Loop(void);
extern void GimbalControlTask_Setup(void);

#endif
