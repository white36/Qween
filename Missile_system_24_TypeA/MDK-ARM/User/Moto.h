/**
  ******************************************************************************
  * File Name          : Moto.h
  * Description        : C620,C610_FOC Driver ���(���)����
  ******************************************************************************
 **/
 
#ifndef __Moto_H
#define __Moto_H

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1

#include "stm32f4xx_hal.h"

/* CAN send and receive ID */
#define    CAN_YAW_ALL_ID  0x200
#define    CAN_SHOOT_ALL_ID  0x1FF
#define    CAN_PULL_SPRING_ID  0x205
#define    CAN_RELOAD_ID  0x206
#define    CAN_MISSILE_SHOOT_MOTOR_ID  0x207
#define    CAN_YAW_MOTOR_ID  0x208

/**********************************
������ݽṹ��
*************************************/
typedef struct
{
	uint16_t	angle;//��е��
	int16_t		speed; //���ת��
	int16_t		current; //ʵ�ʵ���
	uint8_t		temperate; //����¶�
	uint8_t		empty;  //��
	uint16_t	angle_last;//��һ�λ�е��
	int32_t		angle_sum;//�ǶȻ���
} Moto_DataTypedef;

typedef struct
{
	float POS_GAOL;//Ŀ��λ��
	float POS_ABS;//����λ��0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}ANGLE_TypeDef;

 __weak CAN_HandleTypeDef hcan2;          //�����壬��ֹCAN2����ʱ����
 extern CAN_HandleTypeDef hcan1,hcan2;    //HAL��CAN�ṹ������

extern Moto_DataTypedef Moto_Data1[8];
extern Moto_DataTypedef Moto_Data2[8];

void CAN_CMD_MOTO(CAN_HandleTypeDef *CAN_ID,uint32_t MOTO_ID,int16_t data0, int16_t data1, int16_t data2, int16_t data3);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Filter_Init(CAN_HandleTypeDef *h_can);

#endif
