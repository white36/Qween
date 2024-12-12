
#ifndef __MOTIONTASK_H
#define __MOTIONTASK_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "Moto.h"
#include "pid.h"
#include "user_lib.h"
#include "math.h"
//typedef __packed struct
//{
//    fp32 input;        //��������
//    fp32 out;          //�˲����������
//    fp32 num[1];       //�˲�����
//    fp32 frame_period; //�˲���ʱ���� ��λ s
//} first_order_filter_type_t;
typedef struct
{
	int16_t *buffer;
	volatile uint16_t size;
	volatile uint16_t poi;
}average_type;
////����
//#define CHASSIS_3508_P				15000.0f	//����3508
//#define CHASSIS_3508_I				10.0f
//#define CHASSIS_3508_D				0.0f
#define CHASSIS_3508_P				5000.0f	//����3508 15000
#define CHASSIS_3508_I				0.0f
#define CHASSIS_3508_D				700.0f

//�ٶȽ���
#define CHASSIS_VX_MAX									4.0f	//�����ٶ����ֵ
#define CHASSIS_VY_MAX									4.0f
#define CHASSIS_WZ_MAX									10.0f
#define CHASSIS_VUP_MAX					        4.0f	
#define CHASSIS_VTRAN_MAX               3.0f
#define CHASSIS_VSY_MAX                 3.0F
#define CHASSIS_WZ_SET_SCALE						0.1f	//����
#define MOTOR_DISTANCE_TO_CENTER				0.2f	//�����ľ�
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX	0.25f	//����ת����
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY	0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ	0.25f
#define M3508_MOTOR_RPM_TO_VECTOR				0.000415809748903494517209f	//m3508ת���ɵ����ٶ�(m/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//ң����
#define CHASSIS_RC_DEADLINE			 25.0f
#define CHASSIS_RC_X_CH					 3
#define CHASSIS_RC_Y_CH					 2
#define CHASSIS_RC_Z_CH					 0

#define CHASSIS_RC_MODE_S2			 0
#define CHASSIS_RC_MODE_S1			 1
//���
#define MOTOR_CHASSIS_FR	Moto_Data1[0]//����ǰ��
#define MOTOR_CHASSIS_FL	Moto_Data1[1]
#define MOTOR_CHASSIS_BR	Moto_Data1[2]
#define MOTOR_CHASSIS_BL	Moto_Data1[3]

#define OBSTACLE_ENABLE_IO GPIO_PIN_SET
#define OBSTACLE_DISABLE_IO GPIO_PIN_RESET

#define OBSTACLE_ENABLE							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,OBSTACLE_ENABLE_IO)//���ǰ��
#define OBSTACLE_DISABLE							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,OBSTACLE_DISABLE_IO)


typedef enum
{
	CHASSIS_EMPTY,
	CHASSIS_NO_FORCE,
  CHASSIS_RC,
	CHASSIS_KEYBOARD,
	
} chassis_mode_e;

typedef enum
{
	OBSTACLE_EMPTY,
	OBSTACLE_DOWN,
	OBSTACLE_MOVE,

} obstacle_mode_e;

typedef enum
{
	CHASSIS_MOVE,//�ϲ��ƶ�
	CHASSIS_ACTION,//�²��ƶ�
	CHASSIS_BUG,//����bug
	CHASSIS_GOLD,
	CHASSIS_SILVER,
	CHASSIS_CATCH,
} chassis_key_mode_e;


typedef struct
{
  const Moto_DataTypedef *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;


typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def chassis_pid[4];             //motor speed PID.���̵���ٶ�pid

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s

//���							
  fp32 chassis_speed[4];		    //���̵�� 3508 can1 id 1��4
  fp32 chassis_speedset[4];
  fp32 chassis_give_current[4];
  fp32 direction;

  ramp_function_source_t wz_set_ramp_source_type;

} chassis_move_t;



typedef struct
{
  fp32 speed_up;
  fp32 speed_down;
  fp32 speed_rate;
	fp32 time_run;
	
	
	
	int32_t mode_switch_flag_C;
	int32_t control_Cc;
	int32_t control_Ca;
	
	
	int32_t control_gold_C;
	int32_t control_silver_C;
	int32_t control_catch_C;
	int32_t fast_switch_flag_C;
	
	
	int32_t help_C;
	int32_t help_switch_flag_C;
	int32_t help_ready_flag_C;
	
	
} chassis_keyboard;



typedef struct
{

	fp32 servo_left_cail;
	fp32 servo_right_cail;
//	fp32 
//	fp32 
  int32_t obstacle_steer_flag;
  int32_t obstacle_down;

} obstacle_steer;

extern void MotionControlTask_Setup(void);
extern void MotionControlTask_Loop(void);
////һ���˲���ʼ��
//extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
////һ���˲�����
//extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

#endif
