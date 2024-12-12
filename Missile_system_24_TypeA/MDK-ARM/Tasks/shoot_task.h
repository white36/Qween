/**
 * @file shoot_task.h
 * @version 0.1
 * @date 2023-05-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H


#include "struct_typedef.h"
#include "pid.h"
#include "Moto.h"
#include "remote_control.h"
#include "user_lib.h"

//���ң��������������Ʒ�ʽ��ͨ��
#define SHOOT_CONTROL_CHANNEL 1
#define SHOOT_TASK_INIT_TIME 201
//����������ʱʱ�� 1ms
#define SHOOT_TASK_DELAY_TIME 1

// ��������ʱ��ת�� ��ת����
#define SHOOT_TASK_S_TO_MS(x) ((int32_t)((x * 1000.0f) / (SHOOT_TASK_DELAY_TIME)))

//�����������ʱ�䣬����Ϊ��λ 20 s
#define SHOOT_TASK_MAX_INIT_TIME 10

#define SHOOT_CONTROL_TIME  0.02

#define RC_S_LONG_TIME 2000

#define PRESS_LONG_TIME 400

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

#define MISSILE_READY_ANGLE_PID_MAX_OUT 30000.0f
#define MISSILE_READY_ANGLE_PID_MAX_IOUT 2500.0f
#define MISSILE_READY_SPEED_PID_MAX_OUT 30000.0f
#define MISSILE_READY_SPEED_PID_MAX_IOUT 2500.0f

#define SPRING_READY_ANGLE_PID_MAX_OUT 30000.0f
#define SPRING_READY_ANGLE_PID_MAX_IOUT 2500.0f
#define SPRING_READY_SPEED_PID_MAX_OUT 30000.0f
#define SPRING_READY_SPEED_PID_MAX_IOUT 2500.0f

#define RELOAD_READY_ANGLE_PID_MAX_OUT 30000.0f
#define RELOAD_READY_ANGLE_PID_MAX_IOUT 2500.0f
#define RELOAD_READY_SPEED_PID_MAX_OUT 30000.0f
#define RELOAD_READY_SPEED_PID_MAX_IOUT 2500.0f

#define MYAW_READY_ANGLE_PID_MAX_OUT 30000.0f
#define MYAW_READY_ANGLE_PID_MAX_IOUT 2500.0f
#define MYAW_READY_SPEED_PID_MAX_OUT 30000.0f
#define MYAW_READY_SPEED_PID_MAX_IOUT 2500.0f

#define MISSILE_ANGLE_PID_MAX_OUT 30000.0f
#define MISSILE_ANGLE_PID_MAX_IOUT 2000.0f
#define MISSILE_SPEED_PID_MAX_OUT 30000.0f
#define MISSILE_SPEED_PID_MAX_IOUT 2000.0f

#define SPRING_ANGLE_PID_MAX_OUT 30000.0f
#define SPRING_ANGLE_PID_MAX_IOUT 2000.0f
#define SPRING_SPEED_PID_MAX_OUT 30000.0f
#define SPRING_SPEED_PID_MAX_IOUT 2000.0f

#define RELOAD_ANGLE_PID_MAX_OUT 10000.0f
#define RELOAD_ANGLE_PID_MAX_IOUT 2000.0f
#define RELOAD_SPEED_PID_MAX_OUT 10000.0f
#define RELOAD_SPEED_PID_MAX_IOUT 2000.0f

#define MYAW_ANGLE_PID_MAX_OUT 30000.0f
#define MYAW_ANGLE_PID_MAX_IOUT 2000.0f
#define MYAW_SPEED_PID_MAX_OUT 30000.0f
#define MYAW_SPEED_PID_MAX_IOUT 2000.0f

#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0



typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_BULLET_ONE,
    SHOOT_DONE,
    SHOOT_INIT, //��ʼ��ģʽ
} shoot_mode_e;

//�������ģʽ
typedef enum
{
    SHOOT_MOTOR_RUN,  // �������
    SHOOT_MOTOR_STOP, // ���ֹͣ
} shoot_motor_control_mode_e;


typedef enum
{
    SHOOT_LAST_TWO,    		 //����ǰ��վģʽ
    SHOOT_RC_CONTROL,      //ң��������ģʽ
		SHOOT_FIRST_TWO,       //�������ģʽ
    SHOOT_INIT_CONTROL,    //��ʼ������ģʽ
    SHOOT_STOP_CONTROL,    //ֹͣ����ģʽ
}shoot_control_mode_e;

typedef struct
{
    //PID�ṹ��

    PidTypeDef motor_pid;
		PidTypeDef motor_pid_angle;
		pid_type_def motor_Pid;
		pid_type_def motor_Pid_angle;
		Moto_DataTypedef *shoot_motor_measure;
		ANGLE_TypeDef ANGLE_rev;
    fp32 speed;
    fp32 set_speed;
    fp32 angle;
		fp32 given_angle;
    int8_t ecd_count;
    fp32 set_angle;
		int32 angle_sum;//�ǶȻ���
    int16_t give_current;
		int16_t current_cal;
		int32 angle_ref;
		int32 last_angle_ref;
		int32 reload_angle_ref;
		uint32_t blocking_time;
		int16_t block_flag;
    fp32 shoot_CAN_Set_Current;
    fp32 blocking_angle_set;
    fp32 blocking_angle_current;
    int8_t blocking_ecd_count;
} Shoot_Motor_t;

typedef struct
{
    const Moto_DataTypedef *missile_shoot_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 set_speed;
    int16_t give_current;
    uint16_t rc_key_time;
} missile_shoot_Motor_t;


//��ʼ��״̬
typedef enum
{
    SHOOT_INIT_FINISH,   // ��ʼ�����
    SHOOT_INIT_UNFINISH, // ��ʼ��δ���
} shoot_init_state_e;

typedef struct
{
    const RC_ctrl_t *shoot_rc;            // ң����

    shoot_mode_e missile_shoot_mode;               // ����ģʽ
    shoot_mode_e last_missile_shoot_mode;          // ��һ�εķ���ģʽ
    fp32 missile_shoot_CAN_Set_Current[2];         // can�������

    first_order_filter_type_t missile_shoot1_cmd_slow_set_speed; // һ�׵�ͨ
    first_order_filter_type_t missile_shoot2_cmd_slow_set_speed; // һ�׵�ͨ
		first_order_filter_type_t missile_shoot3_cmd_slow_set_speed; // һ�׵�ͨ
		first_order_filter_type_t missile_shoot4_cmd_slow_set_speed; // һ�׵�ͨ

    fp32 angle[2];
    int16_t ecd_count[2];
    int16_t given_current[2];
    fp32 set_angle[2];
    fp32 speed[2];
    fp32 set_speed[2];
    fp32 current_set[2];
    bool_t move_flag;

    fp32 min_speed;
    fp32 max_speed;
    
} missile_shoot_move_t;

extern void shoot_init(void);
extern void shoot_control_loop(void);



/**
 * @brief ���������
 * 
 * @param pvParameters 
 */
void shoot_task(void const *pvParameters);



#endif
