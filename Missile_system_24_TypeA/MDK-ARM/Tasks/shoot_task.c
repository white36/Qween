/**
 * @file shoot_task.c
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "referee.h"
#include "Moto.h"
#include "pid.h"
#include "stm32.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
/*----------------------------------宏定义---------------------------*/
#define Linear_Actuator(x) HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET) // 电推杆IO
/*----------------------------------内部函数---------------------------*/
/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void);

/**
 * @brief          发射控制量计算
 * @param[in]      void
 * @retval         none
 */
static void shoot_set_control(void);

/**
 * @brief        	 射击位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_angle_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief        	 弹簧位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_spring_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 换弹位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_reload_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief        	 yaw位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_yaw_angle_control_loop(Shoot_Motor_t *missile_move_control_loop);

/**
 * @brief          计算位置返回值
 * @param[in]      motor_angle_calc：要计算位置的结构体
 * @retval         none
 */
static void Motor_Angle_Cal(Shoot_Motor_t *motor_angle_calc);

/**
 * @brief          计算速度返回值
 * @param[in]      motor_speed_clac：要计算速度的结构体
 * @retval         none
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac);

/**
 * @brief          计算电流返回值
 * @param[in]      motor_current_calc：要计算电流的结构体
 * @retval         none
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc);

/**
 * @brief          判断电机堵转
 * @param[in]      motor_current_calc：要判断堵转的结构体
 * @retval         none
 */
static void Motor_Block(Shoot_Motor_t*blocking_motor);

/**
 * @brief          判断微动开关是否触发
 * @param[in]      void
 * @retval         none
 */
static void Micro_switch_feedback(void);

/**
 * @brief          设置发射控制模式
 * @param[in]      void
 * @retval         none
 */
static void shoot_set_mode(missile_shoot_move_t *missile_shoot_set_control);

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         none
 */
void shoot_init(void);

/**
 * @brief          舵机控制
 * @param[in]      void
 * @retval         none
 */
void SERIO_Control(void);
/*----------------------------------内部变量---------------------------*/
uint8_t Pull_force = 0;
fp32 missile_shoot;
int shoot_step = 0;
int shoot_step12 = 0;
fp32 motor_last_angle = 0;
int turnback_flag = 0;
int shoot_ready_flag = 0;
#define SERVO_MIN_Pulse   500
#define SERVO_MAX_Pulse   2000
int Pulse=1000;
int turn_shoot_flag = 0;
int turn_spring_flag = 0;
int turn_reload_flag = 0;
int turn_yaw_flag = 0;
int micro_switch_on = 0;
int32 last_reload_ref = 0;
int missile_shoot_cnt = 0;
int reload_next = 1;
int reload_back_flag = 1;
double time_flag = 0;
double last_time_flag = 0;
int shoot_finish_flag = 1;
int start_control_flag = 0;
WeightData_t weight_data;//重量数据
/*----------------------------------结构体------------------------------*/
Shoot_Motor_t missile_shoot_motor;
Shoot_Motor_t pull_spring_motor; 
Shoot_Motor_t reload_motor; 
Shoot_Motor_t yaw_motor;
missile_shoot_move_t missile_shoot_move;       // 发射控制
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t;
/*---------------------------------------------------------------------*/
// 控制模式
shoot_mode_e shoot_mode = SHOOT_STOP;                             // 此次射击模式
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // 上次射击模式
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // 射击控制模式
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // 射击初始化枚举体
shoot_motor_control_mode_e missile_shoot_motor_mode = SHOOT_MOTOR_STOP;    

/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */ 
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    shoot_init();
    while (1)
    {
		//扳机舵机控制
		SERIO_Control();
        //串口将信号量+1触发解码函数
        if (xSemaphoreTake(weight_decode_semaphore, portMAX_DELAY) == pdTRUE) {
            // 直接使用中断中记录的缓冲区，并解码到weight_data里
            decode_weight_data(current_decode_buf, &weight_data);
        }
            // 设置发射模式
            Shoot_Set_Mode();
            // 发射数据更新
            shoot_set_control();
            // 发射控制循环
            shoot_control_loop();
            // 发送控制电流
            //		CAN_CMD_MOTO(&hcan1, CAN_SHOOT_ALL_ID, 0, 0, 0, 0);
            CAN_CMD_MOTO(&hcan1, CAN_SHOOT_ALL_ID, pull_spring_motor.give_current, reload_motor.give_current,
                         missile_shoot_motor.give_current, yaw_motor.give_current);
            vTaskDelay(SHOOT_TASK_DELAY_TIME);
        }
}

/**
 * @brief          舵机控制
 * @param[in]      void
 * @retval         none
 */
void SERIO_Control(void)
{
		if(shoot_control_mode == SHOOT_RC_CONTROL)
		{
		if(missile_shoot_move.shoot_rc->rc.ch[3] >= 531)
		{
			Pulse = missile_shoot_move.shoot_rc->rc.ch[3]*1.9;
		}
		else
		{
			Pulse = 1000;
		}
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,Pulse);
		if(Pulse == 1000)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		}
		}
		if(shoot_control_mode == SHOOT_LAST_TWO)
		{
			if(shoot_step == 0)
			{
			Pulse = 1320;
			}
			if(shoot_step == 3)
			{
			Pulse = 1000;
			}
			if(shoot_step == 5)
			{
			Pulse = 1320;
				if(shoot_finish_flag == 1)
				{
				last_time_flag = time_flag;
					shoot_finish_flag = 0;
				}
			}
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,Pulse);
		}
		if(shoot_control_mode == SHOOT_FIRST_TWO)
		{
			if(shoot_step12 == 3)
			{
			Pulse = 1320;
			}
			if(shoot_step12 == 6)
			{
			Pulse = 1000;
			}
			if(shoot_step12 == 2 || shoot_step12 == 8)
			{
			Pulse = 1320;
				if(shoot_finish_flag == 1)
				{
				last_time_flag = time_flag;
					shoot_finish_flag = 0;
				}
			}
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,Pulse);
		}
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         none
 */
void shoot_init(void)
{
    static const fp32 missile_shoot_speed_pid[3] = {80, 0, 20};
		static const fp32 missile_shoot_angle_pid[3] = {3, 0, 0.1};
		static const fp32 pull_spring_speed_pid[3] = {15, 0, 2};
		static const fp32 pull_spring_angle_pid[3] = {10, 0, 0.5};
		static const fp32 reload_speed_pid[3] = {60, 0, 20};
		static const fp32 reload_angle_pid[3] = {50, 0, 20};
		static const fp32 yaw_speed_pid[3] = {20, 0, 1};
		static const fp32 yaw_angle_pid[4] = {10, 0, 1};
		PID_init(&missile_shoot_motor.motor_Pid_angle, PID_POSITION, missile_shoot_angle_pid, MISSILE_READY_ANGLE_PID_MAX_OUT, MISSILE_READY_ANGLE_PID_MAX_IOUT);
    	PID_init(&missile_shoot_motor.motor_Pid, PID_POSITION, missile_shoot_speed_pid, MISSILE_READY_SPEED_PID_MAX_OUT, MISSILE_READY_SPEED_PID_MAX_IOUT);
		PID_init(&pull_spring_motor.motor_Pid_angle, PID_POSITION, pull_spring_angle_pid, SPRING_READY_ANGLE_PID_MAX_OUT, SPRING_READY_ANGLE_PID_MAX_IOUT);
		PID_init(&pull_spring_motor.motor_Pid, PID_POSITION, pull_spring_speed_pid, SPRING_READY_SPEED_PID_MAX_OUT, SPRING_READY_SPEED_PID_MAX_IOUT);
		PID_init(&reload_motor.motor_Pid_angle, PID_POSITION, reload_angle_pid, RELOAD_READY_ANGLE_PID_MAX_OUT, RELOAD_READY_ANGLE_PID_MAX_IOUT);
		PID_init(&reload_motor.motor_Pid, PID_POSITION, reload_speed_pid, RELOAD_READY_SPEED_PID_MAX_OUT, RELOAD_READY_SPEED_PID_MAX_IOUT);
		PID_init(&yaw_motor.motor_Pid_angle, PID_POSITION, yaw_angle_pid, MYAW_READY_ANGLE_PID_MAX_OUT, MYAW_READY_ANGLE_PID_MAX_IOUT);
		PID_init(&yaw_motor.motor_Pid, PID_POSITION, yaw_speed_pid, MYAW_READY_SPEED_PID_MAX_OUT, MYAW_READY_SPEED_PID_MAX_IOUT);
    // 数据指针获取
        // weight_data = get_weight_data_point();//解码在串口中断时启用
        missile_shoot_move.shoot_rc = get_remote_control_point();
        reload_motor.shoot_motor_measure = &Moto_Data1[5];
    	reload_motor.blocking_angle_set = 0;
		reload_motor.set_angle = 0;
    	pull_spring_motor.shoot_motor_measure = &Moto_Data1[4];
		pull_spring_motor.blocking_angle_set = 0;
		pull_spring_motor.set_angle = 0;
   		missile_shoot_motor.shoot_motor_measure = &Moto_Data1[6];
		missile_shoot_motor.blocking_angle_set = 0;
		missile_shoot_motor.set_angle = 0;
		yaw_motor.shoot_motor_measure = &Moto_Data1[7];
		yaw_motor.blocking_angle_set = 0;
		yaw_motor.set_angle = 0;	
		 
    shoot_set_control();
}

/**
 * @brief          发射控制量计算
 * @param[in]      void
 * @retval         none
 */
static void shoot_set_control(void)
{    
		Motor_Speed_Cal(&missile_shoot_motor);
		Motor_Angle_Cal(&missile_shoot_motor);
		Motor_Current_Cal(&missile_shoot_motor);
		Motor_Block(&missile_shoot_motor);
	
		Motor_Speed_Cal(&reload_motor);
		Motor_Angle_Cal(&reload_motor);
		Motor_Current_Cal(&reload_motor);
		Motor_Block(&reload_motor);
		
		Motor_Speed_Cal(&pull_spring_motor);
		Motor_Angle_Cal(&pull_spring_motor);
		Motor_Current_Cal(&pull_spring_motor);
		Motor_Block(&pull_spring_motor);
		
		Motor_Speed_Cal(&yaw_motor);
		Motor_Angle_Cal(&yaw_motor);
		Motor_Current_Cal(&yaw_motor);
		Motor_Block(&yaw_motor);
	
		Micro_switch_feedback();
		time_flag+=0.01;
}

/**
 * @brief          计算速度返回值
 * @param[in]      motor_speed_clac：要计算速度的结构体
 * @retval         none
 */
static void Motor_Speed_Cal(Shoot_Motor_t *motor_speed_clac)
{
	motor_speed_clac->speed = motor_speed_clac->shoot_motor_measure->speed * Motor_RMP_TO_SPEED * 10;
}

/**
 * @brief          计算位置返回值
 * @param[in]      motor_angle_calc：要计算位置的结构体
 * @retval         none
 */
static void Motor_Angle_Cal(Shoot_Motor_t *motor_angle_calc)
{
		motor_angle_calc->ANGLE_rev.eer = motor_angle_calc->shoot_motor_measure->angle -motor_angle_calc->shoot_motor_measure->angle_last;
		if(motor_angle_calc->ANGLE_rev.eer < -4096)
		{
			motor_angle_calc->ANGLE_rev.eer += 8192;
		}
		else if(motor_angle_calc->ANGLE_rev.eer > 4096)
		{
			motor_angle_calc->ANGLE_rev.eer -= 8192;
		}
		motor_angle_calc->angle_sum += motor_angle_calc->ANGLE_rev.eer;
		motor_angle_calc->angle_ref = motor_angle_calc->angle_sum*360.0/19.0/8192.0/10;
		motor_angle_calc->reload_angle_ref = motor_angle_calc->angle_sum*360.0/8192.0/10;
}

/**
 * @brief          计算电流返回值
 * @param[in]      motor_current_calc：要计算电流的结构体
 * @retval         none
 */
static void Motor_Current_Cal(Shoot_Motor_t *motor_current_calc)
{
	motor_current_calc->current_cal = motor_current_calc->shoot_motor_measure->current;
}

/**
 * @brief          判断电机堵转
 * @param[in]      motor_current_calc：要判断堵转的结构体
 * @retval         none
 */
static void Motor_Block(Shoot_Motor_t*blocking_motor)
{
	if(abs(blocking_motor->give_current) == 10000)
	{
		blocking_motor->blocking_time++;
	}
	else
	{
		blocking_motor->blocking_time = 0;
	}
	if(blocking_motor->blocking_time >=1000)
	{
		blocking_motor->block_flag = 1;
	}
	else
	{
		blocking_motor->block_flag = 0;
	}
}

/**
 * @brief          判断微动开关是否触发
 * @param[in]      void
 * @retval         none
 */
static void Micro_switch_feedback()
{
if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
{
	micro_switch_on = 0;
}
else
{
	micro_switch_on = 1;
}
}

/**
 * @brief          设置发射控制模式
 * @param[in]      void
 * @retval         none
 */
static void shoot_set_mode(missile_shoot_move_t *missile_shoot_set_control)
{

    // 运行模式

    // 判断初始化是否开启
    // if (shoot_control_mode == SHOOT_INIT_CONTROL)
    // {
    //     static uint32_t init_time = 0;
    //     // 判断拨杆是否拨到下档
    //     if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    //     {
    //         // 拨到下档停止初始化
    //         init_time = 0;
    //     }
    //     else
    //     {
    //         // 判断是否初始化完成
    //         if (shoot_init_state == SHOOT_INIT_UNFINISH)
    //         {
    //             // 初始化未完成

    //             // 判断初始化时间是否过长
    //             if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
    //             {
    //                 // 初始化时间过长不进行初始化，进入其他模式
    //                 init_time = 0;
    //             }
    //             else
    //             {
    //                     // 初始化模式保持原状，初始化时间增加
    //                     init_time++;
    //                     return;
	// 							}
	// 						}
    //         else
    //         {
    //             // 进入其他模式
    //             init_time = 0;
    //         }
    //     }
    // }

    //开电时遥控器拨杆状态保护
    if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]) && switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[0])
        && start_control_flag == 0)
    {
        start_control_flag = 1;
    }
    if(start_control_flag == 1)
    {
        // 根据遥控器开关设置发射控制模式
        if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 发射后两发
            shoot_control_mode = SHOOT_LAST_TWO;
        }
        else if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 遥控器控制模式
            shoot_control_mode = SHOOT_RC_CONTROL;
        }
        else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 发射前两发
            shoot_control_mode = SHOOT_FIRST_TWO;
        }
        else
        {
            shoot_control_mode = SHOOT_STOP_CONTROL;
        }
            
        if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[0])&&switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            Linear_Actuator(0);
        }
        else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[0])&&switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            Linear_Actuator(1);
        }
    }

    
    // 判断进入初始化模式(弃置)
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // 进入初始化模式
        //        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}
/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void)
{

    // 设置发射控制模式
    shoot_set_mode(&missile_shoot_move);
	
	// 保存为上次射击模式，以便下个循环读取
    last_shoot_mode = shoot_mode;
	
    // 更新当前射击模式
	
	
	
}
/**
 * @brief          发射控制循环
 * @param[in]      void
 * @retval         返回无
 */
void shoot_control_loop(void)
{
	if(shoot_control_mode == SHOOT_LAST_TWO)//装填
	{
		if(shoot_step == 0)
		{
			missile_shoot_motor.set_angle += 0.8f;
				if(micro_switch_on == 1)
				{
					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
					shoot_step++;
				}
		}		
		if(shoot_step == 1)
		{
			reload_motor.set_angle += 0.05f;
				if(reload_motor.block_flag == 1)
				{
						reload_motor.set_angle = reload_motor.reload_angle_ref;
						shoot_step++;
				}
		}
		if(shoot_step == 2 && Pulse == 1320)
			{
				missile_shoot_motor.set_angle -= 0.8f;
					if(missile_shoot_motor.set_angle <= 0)
					{
						missile_shoot_motor.set_angle = 0;
						shoot_step++;
					}	
			}
		if(missile_shoot_motor.set_angle <= 0)
		{
			missile_shoot_motor.set_angle = 0;
		}
		else if(missile_shoot_motor.set_angle >= 1700)
		{
			missile_shoot_motor.set_angle = 1700;
		}	
//		if(shoot_step == 0)//后两发（装填并发射）
//		{
//			missile_shoot_motor.set_angle += 0.8f;
//				if(micro_switch_on == 1)
//				{
//					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
//					shoot_step++;
//				}
//		}		
//		if(shoot_step == 1)
//		{
//			reload_motor.set_angle += 0.05f;
//				if(reload_motor.block_flag == 1)
//				{
//						reload_motor.set_angle = reload_motor.reload_angle_ref;
//						shoot_step++;
//				}
//		}
//		if(shoot_step == 2 && Pulse == 1320)
//			{
//				missile_shoot_motor.set_angle -= 0.8f;
//					if(missile_shoot_motor.set_angle <= 880)
//					{
//						missile_shoot_motor.set_angle = 880;
//						shoot_step++;
//					}	
//			}
//		if(shoot_step == 3 && Pulse == 1000)
//			{
//				missile_shoot_motor.set_angle += 0.8f;
//					if(micro_switch_on == 1)
//				{
//					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
//					shoot_step++;
//				}
//			}
//		if(shoot_step == 4)
//			{
//				missile_shoot_motor.set_angle -= 0.8f;
//					if(missile_shoot_motor.set_angle <= 0)
//					{
//						missile_shoot_motor.set_angle = 0;
//						shoot_finish_flag = 1;
//						shoot_step++;
//					}	
//			}
//			if(shoot_step == 5&& Pulse == 1320 && time_flag - last_time_flag >= 20)
//			{
//				if(reload_next == 1)
//				{
//				last_reload_ref = reload_motor.reload_angle_ref;
//					Linear_Actuator(1);
//					reload_next = 0;
//				}
//				if(reload_back_flag == 1)
//				{
//				reload_motor.set_angle = last_reload_ref - 3;
//				reload_back_flag = 0;
//				}
//				if(reload_motor.reload_angle_ref <= last_reload_ref - 3)
//				{
//				reload_motor.set_angle = last_reload_ref + 10;
//				}
//				if(reload_motor.reload_angle_ref >= last_reload_ref + 10)
//				{
//						Linear_Actuator(0);
//						shoot_step = 0;
//						missile_shoot_cnt++;
//					  reload_next = 1;
//					reload_back_flag = 1;
//				}
//			}
//			if(missile_shoot_cnt == 2)
//			{
//				shoot_step = 6;
//			}
//			
//		if(missile_shoot_motor.set_angle <= 0)
//		{
//			missile_shoot_motor.set_angle = 0;
//		}
//		else if(missile_shoot_motor.set_angle >= 1700)
//		{
//			missile_shoot_motor.set_angle = 1700;
//		}
		// pid计算
    missile_angle_control_loop(&missile_shoot_motor); // 发射控制
		missile_spring_angle_control_loop(&pull_spring_motor); // 弹簧控制
		missile_reload_angle_control_loop(&reload_motor); // 换弹盘控制
		missile_yaw_angle_control_loop(&yaw_motor); // yaw轴控制
	}
	
	if(shoot_control_mode == SHOOT_RC_CONTROL)//遥控器控制
	{
		shoot_step = 0;	
		shoot_step12 = 0;
		missile_shoot_cnt = 0;		
		    if (missile_shoot_move.shoot_rc->rc.ch[4] == 660 && turn_shoot_flag == 0)
    {
					missile_shoot_motor.set_angle -= 0.8f;
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[4] >= 1000 && turn_shoot_flag == 0)
    {
					missile_shoot_motor.set_angle += 0.8f;
		}
		if(missile_shoot_motor.set_angle <= 0)
		{
			missile_shoot_motor.set_angle = 0;
		}
		
		if(micro_switch_on == 1)
		{
			missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
			turn_shoot_flag = 1;
		}

				if (missile_shoot_move.shoot_rc->rc.ch[2] ==660 && turn_spring_flag == 0)
    {
					pull_spring_motor.set_angle += 0.03f;	
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[2] == -660 && turn_spring_flag == 0)
    {
					pull_spring_motor.set_angle -= 0.03f;		
    }

				if (missile_shoot_move.shoot_rc->rc.ch[1] > 400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle += 0.08f;

    }
		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -400 && turn_reload_flag == 0)
    {
					reload_motor.set_angle -= 0.08f;	
    }
		
		if(reload_motor.block_flag == 1)
		{
				reload_motor.set_angle = reload_motor.reload_angle_ref;
				turn_reload_flag = 1;
		}
		
				if (missile_shoot_move.shoot_rc->rc.ch[0] > 400 && turn_yaw_flag == 0)
    {
					yaw_motor.set_angle -= 0.01f;

    }
		else if (missile_shoot_move.shoot_rc->rc.ch[0] < -400 && turn_yaw_flag == 0)
    {
					yaw_motor.set_angle += 0.01f;
	
    }

		if (missile_shoot_move.shoot_rc->rc.ch[4] == 0)
		{		
			missile_shoot_motor.set_speed = 0;
			missile_shoot_motor.set_angle += 0;
			turn_shoot_flag = 0;
		}
		
		if (missile_shoot_move.shoot_rc->rc.ch[1] == 0)
		{		
			reload_motor.set_speed = 0;
			reload_motor.set_angle += 0;
			turn_reload_flag = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[2] == 0)
		{		
			pull_spring_motor.set_speed = 0;
			pull_spring_motor.set_angle += 0;
			turn_spring_flag = 0;
		}
		
		if (missile_shoot_move.shoot_rc->rc.ch[0] == 0)
		{		
			yaw_motor.set_speed = 0;
			yaw_motor.set_angle += 0;
			turn_yaw_flag = 0;
		}

    // pid计算
    missile_angle_control_loop(&missile_shoot_motor); // 发射控制
		missile_spring_angle_control_loop(&pull_spring_motor); // 弹簧控制
		missile_reload_angle_control_loop(&reload_motor); // 换弹盘控制
		missile_yaw_angle_control_loop(&yaw_motor); // yaw轴控制
	}
	
	if(shoot_control_mode == SHOOT_FIRST_TWO)//发射两发
	{
		if(shoot_step12 == 0)
		{
			missile_shoot_motor.set_angle += 0.8f;
				if(micro_switch_on == 1)
				{
					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
					shoot_step12++;
				}
		}
		if(shoot_step12 == 1)
			{
				missile_shoot_motor.set_angle -= 0.8f;
					if(missile_shoot_motor.set_angle <= 0)
					{
						missile_shoot_motor.set_angle = 0;
						shoot_finish_flag = 1;
						shoot_step12++;
					}	
			}
			if(shoot_step12 == 2&& Pulse == 1320 && time_flag - last_time_flag >= 20)
			{
				if(reload_next == 1)
				{
				last_reload_ref = reload_motor.reload_angle_ref;
					Linear_Actuator(1);
					reload_next = 0;
				}
				if(reload_back_flag == 1)
				{
				reload_motor.set_angle = last_reload_ref - 3;
				reload_back_flag = 0;
				}
				if(reload_motor.reload_angle_ref <= last_reload_ref - 3)
				{
				reload_motor.set_angle = last_reload_ref + 10;
				}
				if(reload_motor.reload_angle_ref >= last_reload_ref + 10)
				{
						Linear_Actuator(0);
						shoot_step12++;
						missile_shoot_cnt++;
					  reload_next = 1;
					reload_back_flag = 1;
				}
			}
			if(shoot_step12 == 3)
		{
			missile_shoot_motor.set_angle += 0.8f;
				if(micro_switch_on == 1)
				{
					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
					shoot_step12++;
				}
		}		
		if(shoot_step12 == 4)
		{
			reload_motor.set_angle += 0.05f;
				if(reload_motor.block_flag == 1)
				{
						reload_motor.set_angle = reload_motor.reload_angle_ref;
						shoot_step12++;
				}
		}
		if(shoot_step12 == 5 && Pulse == 1320)
			{
				missile_shoot_motor.set_angle -= 0.8f;
					if(missile_shoot_motor.set_angle <= 880)
					{
						missile_shoot_motor.set_angle = 880;
						shoot_step12++;
					}	
			}
		if(shoot_step12 == 6 && Pulse == 1000)
			{
				missile_shoot_motor.set_angle += 0.8f;
					if(micro_switch_on == 1)
				{
					missile_shoot_motor.set_angle = missile_shoot_motor.angle_ref;
					shoot_step12++;
				}
			}
		if(shoot_step12 == 7)
			{
				missile_shoot_motor.set_angle -= 0.8f;
					if(missile_shoot_motor.set_angle <= 0)
					{
						missile_shoot_motor.set_angle = 0;
						shoot_finish_flag = 1;
						shoot_step12++;
					}	
			}
			if(shoot_step12 == 8&& Pulse == 1320 && time_flag - last_time_flag >= 20)
			{
				if(reload_next == 1)
				{
				last_reload_ref = reload_motor.reload_angle_ref;
					Linear_Actuator(1);
					reload_next = 0;
				}
				if(reload_back_flag == 1)
				{
				reload_motor.set_angle = last_reload_ref - 3;
				reload_back_flag = 0;
				}
				if(reload_motor.reload_angle_ref <= last_reload_ref - 3)
				{
				reload_motor.set_angle = last_reload_ref + 10;
				}
				if(reload_motor.reload_angle_ref >= last_reload_ref + 8)
				{
						Linear_Actuator(0);
						shoot_step12++;
						missile_shoot_cnt++;
					  reload_next = 1;
					reload_back_flag = 1;
				}
			}
			
			if(missile_shoot_cnt == 2)
			{
				shoot_step12 = 9;
			}
			if(missile_shoot_motor.set_angle <= 0)
			{
				missile_shoot_motor.set_angle = 0;
			}
			else if(missile_shoot_motor.set_angle >= 1700)
			{
				missile_shoot_motor.set_angle = 1700;
			}
				// pid计算
    missile_angle_control_loop(&missile_shoot_motor); // 发射控制
		missile_spring_angle_control_loop(&pull_spring_motor); // 弹簧控制
		missile_reload_angle_control_loop(&reload_motor); // 换弹盘控制
		missile_yaw_angle_control_loop(&yaw_motor); // yaw轴控制
	}
}

/**
 * @brief          发射位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_Pid.max_out = MISSILE_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid.max_iout = MISSILE_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_Pid_angle.max_out = MISSILE_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid_angle.max_iout = MISSILE_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_calc(&missile_move_control_loop->motor_Pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_calc(&missile_move_control_loop->motor_Pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_Pid.out);
}

/**
 * @brief          弹簧位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_spring_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_Pid.max_out = SPRING_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid.max_iout = SPRING_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_Pid_angle.max_out = SPRING_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid_angle.max_iout = SPRING_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_calc(&missile_move_control_loop->motor_Pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_calc(&missile_move_control_loop->motor_Pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_Pid.out);
}

/**
 * @brief          换弹位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none
 */
static void missile_reload_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_Pid.max_out = RELOAD_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid.max_iout = RELOAD_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_Pid_angle.max_out = RELOAD_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid_angle.max_iout = RELOAD_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_calc(&missile_move_control_loop->motor_Pid_angle, missile_move_control_loop->reload_angle_ref, missile_move_control_loop->set_angle);
		PID_calc(&missile_move_control_loop->motor_Pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_Pid.out);
}

/**
 * @brief          yaw位置控制循环
 * @param[in]      trigger_move_control_loop：要计算位置的结构体
 * @retval         none  
 */
static void missile_yaw_angle_control_loop(Shoot_Motor_t *missile_move_control_loop)
{
    missile_move_control_loop->motor_Pid.max_out = MYAW_SPEED_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid.max_iout = MYAW_SPEED_PID_MAX_IOUT;	
		missile_move_control_loop->motor_Pid_angle.max_out = MYAW_ANGLE_PID_MAX_OUT;
    missile_move_control_loop->motor_Pid_angle.max_iout = MYAW_ANGLE_PID_MAX_IOUT;	
		missile_move_control_loop->given_angle = PID_calc(&missile_move_control_loop->motor_Pid_angle, missile_move_control_loop->angle_ref, missile_move_control_loop->set_angle);
		PID_calc(&missile_move_control_loop->motor_Pid, missile_move_control_loop->speed, missile_move_control_loop->given_angle);
    missile_move_control_loop->give_current = (missile_move_control_loop->motor_Pid.out);
}



