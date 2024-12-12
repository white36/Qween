#include "MotionTask.h"
#include "Gimbal_Task.h"
#include "remote_control.h"
#include "Servo.h"
#include "tim.h"
#include "pid.h"

//volatile float gimbal_pitch_angle = 90;		//ͼ��pitch��Ƕ�

////Servo_HandlerTypedef
////	servo_rescue_right,
////	servo_rescue_left;
//gimbal_move_t gimbal_move;
//gimbal_mode_e gimbal_mode = GIMBAL_EMPTY;

//Servo_HandlerTypedef pitch_servo;

//////��������
//static void gimbal_mode_set(void);
//static void gimbal_update(void);
//static void gimbal_control_set(void);
//static void gimbal_can_tx(void);
//static void gimbal_rc_controldown();
//static void gimbal_keyboard_control(void);
//static void gimbal_pitch_control(void);

chassis_mode_e chassis_mde;
Servo_HandlerTypedef
	servo_gimbal_pitch;
//////������ѯ
void GimbalControlTask_Loop(void)
{
//	gimbal_mode_set();		//ң����ѡ��ģʽ
//	gimbal_update();			//���ݸ���
//	gimbal_control_set();//����������8

//	gimbal_can_tx();			//can����
//	Servo_Set_Angle(&servo_rescue_left,gimbal_pitch_angle);
	
	osDelay(1);

}
////fp32 init_angle;
//////���̳�ʼ��
void GimbalControlTask_Setup(void)
{
//	//PID��ʼ��
//	//�ٶȻ�pid����
//	PID_Value_Typedef gimbal_roll_pidval = {GIMBAL_2006_P,GIMBAL_2006_I,GIMBAL_2006_D};									//��צPID����
////	//�ǶȻ�pid����
////  PID_Value_Typedef gimbal_roll_angelpidval = {GIMBAL_2006_ANGEL_PID_KP, GIMBAL_2006_ANGEL_PID_KI, GIMBAL_2006_ANGEL_PID_KD};

//	
//	
//	//�ٶȻ���ʼ��
//	PID_init(&gimbal_move.gimbal_roll_pid,PID_POSITION,&gimbal_roll_pidval,M2006_MAX_OUT,M2006_MAX_IOUT);//6020

////	//�ǶȻ���ʼ��
////	PID_init(&gimbal_move.gimbal_roll_angelpid,PID_POSITION,&gimbal_roll_angelpidval,M2006_MAX_OUT,M2006_MAX_IOUT);//6020
//////  


////	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
////	//�����ʼ��
////	Servo_Init(&servo_rescue_left,&htim8,TIM_CHANNEL_1);
////	Servo_Init(&servo_rescue_right,&htim8,TIM_CHANNEL_2);
////		//�������
////	Servo_Set_Angle(&servo_rescue_left,	126);
//////	Servo_Set_Angle(&servo_rescue_left,	150);
////	Servo_Set_Angle(&servo_rescue_right,54);
//////	Servo_Set_Angle(&servo_rescue_right,30);
	Servo_Init(&servo_gimbal_pitch,&htim8,TIM_CHANNEL_1);

}

//static void gimbal_mode_set(void)
//{
//	if(switch_is_up(rc_ctrl.rc.s[GIMBAL_RC_MODE_S2]))
//		gimbal_mode = GIMBAL_NO_FORCE;
//	if(switch_is_mid(rc_ctrl.rc.s[GIMBAL_RC_MODE_S2]))
//		gimbal_mode = GIMBAL_RC;
//	if(switch_is_down(rc_ctrl.rc.s[GIMBAL_RC_MODE_S2]))
//		gimbal_mode = GIMBAL_KEYBOARD;
//}

//static void gimbal_update(void)
//{
//	
//	gimbal_move.gimbal_roll_speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[0].speed;
//	gimbal_move.gimbal_roll_angle = Moto_Data2[0].angle_sum*360.0/8192.0/36.0;
//	
//}
//static void gimbal_control_set(void)
//{
//		if(gimbal_mode == GIMBAL_RC)//ң��������
//	{
//		if(switch_is_down(rc_ctrl.rc.s[GIMBAL_RC_MODE_S1]))
//		gimbal_rc_controldown();
//		
//	}
//	
//	if(gimbal_mode == GIMBAL_KEYBOARD)
//	{
//		gimbal_keyboard_control();
//	}
//	
//		gimbal_move.gimbal_roll_current = PID_calc(&gimbal_move.gimbal_roll_pid,gimbal_move.gimbal_roll_speed,gimbal_move.gimbal_roll_speed_set);

//}

//static void gimbal_rc_controldown(void)
//{
//	//ң��������
//	rc_deadband_limit((fp32)rc_ctrl.rc.ch[GIMBAL_RC_M2006],gimbal_move.rc_gimbal_roll,GIMBAL_RC_DEADLINE);
//	//�����ٶ�����
////	action_forward.forward_speedset[0] = action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
////	action_forward.forward_speedset[1] = -action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
//	gimbal_move.gimbal_roll_speed_set = gimbal_move.rc_gimbal_roll/660*3.0f;
//	

//	
//	
//	
//	
//	
//	
//	

//	
//	//176 474

//}


//static void gimbal_pitch_control(void)
//{
//	
//	
//	gimbal_move.gimbal_roll_angle_mileage = Moto_Data2[0].angle_sum*360.0/36.0/8192.0; //�Ƕ���� �Ƕ���
//	gimbal_move.gimbal_roll_mileage = Moto_Data2[0].angle_sum/19.0/8192.0*100*3.14; //������� ��λmm
//	
//	//�Ҳ�����ǰ��ת��λ
//	if(Moto_Data2[0].current<-2000)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
//	{
//		gimbal_move.gimbal_roll_limit_flag = 1;//��־λΪ1
//		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//	}
//	else
//	{
//		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//ȷ�����ֵ Ϊ��תλ��
//	}

//	if(gimbal_move.gimbal_roll_limit_flag == 1)//ͨ����־Ϊ���˶�������λ
//	{
//		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//�����˶�����
//    if(gimbal_move.gimbal_roll_interval>520.0f)//�Ҳ���������Сֵ
//	  {
//		  gimbal_move.gimbal_roll_interval = 520.0f;
//		  if(gimbal_move.gimbal_roll_speed_set>0)//����������Сֵʱ�ٶȴ����� �������������ٶ�
//		  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0f;
//		  }
//	  }
//    if(gimbal_move.gimbal_roll_interval<0.6f)//�Ҳ����������ֵ
//	  {
//	  	gimbal_move.gimbal_roll_interval = 0.6f;
//	  	if(gimbal_move.gimbal_roll_speed_set<0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
//	  	{
//	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
//	  	}
//	  }	

//		
//  }
//	
//	
//	
//		if(Moto_Data2[0].current>2000)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
//	{
//		gimbal_move.gimbal_roll_limit_flag = 2;//��־λΪ1
//		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//	}
//	else
//	{
//		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//ȷ�����ֵ Ϊ��תλ��
//	}

//	if(gimbal_move.gimbal_roll_limit_flag == 2)//ͨ����־Ϊ���˶�������λ
//	{
//		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//�����˶�����
//    if(gimbal_move.gimbal_roll_interval<-520.0f)//�Ҳ���������Сֵ
//	  {
//		  gimbal_move.gimbal_roll_interval = -520.0f;
//		  if(gimbal_move.gimbal_roll_speed_set<0)//����������Сֵʱ�ٶȴ����� �������������ٶ�
//		  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0f;
//		  }
//	  }
//    if(gimbal_move.gimbal_roll_interval>0.6f)//�Ҳ����������ֵ
//	  {
//	  	gimbal_move.gimbal_roll_interval = 0.6f;
//	  	if(gimbal_move.gimbal_roll_speed_set>0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
//	  	{
//	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
//	  	}
//	  }	

//		
//  }
//}


//static void gimbal_keyboard_control(void)
//{
//	gimbal_move.gimbal_roll_speed_set = (fp32)rc_ctrl.mouse.x * 0.01f;//TTT
//	
//	if(gimbal_move.gimbal_roll_speed_set>2.0)
//	{
//		gimbal_move.gimbal_roll_speed_set = 2.0;
//	}
//	if(gimbal_move.gimbal_roll_speed_set<-2.0)
//	{
//		gimbal_move.gimbal_roll_speed_set = -2.0;
//	}
//	
//	gimbal_pitch_control();
//	
//	
//}

//static void gimbal_can_tx(void)
//{
////		CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_LOW,
////							 (int16_t)gimbal_move.gimbal_roll_current,
////							 0,

////							 0,
////							 0);

//}

