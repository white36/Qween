/***********************************************************
�ļ�����Servo.c
������
		�������ʵ��
From:Qi-Q@Rjgawuie
***********************************************************/
#include "Servo.h"

/***********************************************************
��������Servo_Init
���ܣ����PWM��ʼ����ʹ��ǰ����
������servohandler 	������ƾ����ַ
			htimhandler 	��ʱ�������ַ
			Chn 					ͨ����
����ֵ: ��ʼ���Ƿ�ɹ�
***********************************************************/
HAL_StatusTypeDef Servo_Init(Servo_HandlerTypedef *servohandler,TIM_HandleTypeDef *htimhandler, uint32_t Chn)
{
	servohandler->timhandler = htimhandler;
	servohandler->Chn = Chn;
	return HAL_TIM_PWM_Start(htimhandler,Chn);
}

/***********************************************************
��������Servo_Init
���ܣ����PWM��ʼ����ʹ��ǰ����
������htimhandler 	��ʱ�������ַ
			Chn 					ͨ����
����ֵ: ��ʼ���Ƿ�ɹ�
***********************************************************/
void Servo_Set_Angle(Servo_HandlerTypedef *servohandler,float angle)
{
	uint16_t Cmp_v;
	Cmp_v = 500 + (uint16_t)(11.111111f * angle);
	__HAL_TIM_SetCompare(servohandler->timhandler,servohandler->Chn,Cmp_v);
}
