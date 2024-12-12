
#include "ActionTask.h"
#include "remote_control.h"
#include "Servo.h"
#include "tim.h"
#include "pid.h"
//#include "Gimbal_Task.h"


action_mode_e action_mode = ACTION_EMPTY;
action_key_mode_e key_mode_A = ACTION_CHASSIS;

pid_init_struct action_pid;
forward_struct action_forward;
uplift_struct action_uplift;
pitch_struct action_pitch;
roll_struct action_roll;

gimbal_move_t gimbal_move;


action_keyboard Key_A;
//gimbal_mode_e gimbal_mode = GIMBAL_EMPTY;

bool_t microswitch_uplift_bottoml;

bool_t microswitch_uplift_bottomr;
//��������
static void action_mode_set(void);//ģʽ����
static void action_update(void);//��ȡ����
static void action_control_set(void);//�ٶ� �Ƕ��趨
static void action_can_tx(void);//canͨ�ŷ���
static void calibration(void);//У׼����
static void action_rc_controlup(void);//ң�����ϲ��˿���
static void action_rc_controlmid(void);//ң�����в��˿���
static void action_rc_controldown(void);//ң�����²��˿���
static void action_keyboard_control(void);//���̲��˿���
static void forward_control(void);//ǰ����λ����
static void uplift_control(void);//̧����λ����
static void pitch_control(void);//pitch����λ����
static void roll_control(void);//roll����λ����
static void gimbal_roll_control(void);//��̨roll����λ����

//������ѯ
void ActionControlTask_Loop(void)
{
	action_mode_set();		//ң����ѡ��ģʽ
	action_update();			//���ݸ���
	action_control_set();//����������8

	action_can_tx();			//can����

	osDelay(1);
	
	
}

//���̳�ʼ��
void ActionControlTask_Setup(void)
{
	//PID��ʼ��
	//�ٶȻ�pid����
	PID_Value_Typedef forward_pidval = {FORWARD_3508_P,FORWARD_3508_I,FORWARD_3508_D};									//��צPID����
	PID_Value_Typedef uplift_pidval_up = {UPLIFT_3508_P,UPLIFT_3508_I,UPLIFT_3508_D};	//��ת����PID����
	PID_Value_Typedef sucker_pitch_pidval = {SUCKER_PITCH_3508_P,SUCKER_PITCH_3508_I,SUCKER_PITCH_3508_D};			//������PID����
	PID_Value_Typedef sucker_roll_pidval = {SUCKER_ROLL_P,SUCKER_ROLL_I,SUCKER_ROLL_D};			//������PID����
	PID_Value_Typedef gimbal_roll_pidval = {GIMBAL_2006_P,GIMBAL_2006_I,GIMBAL_2006_D};									//��̨pitch��PID����
	//�ǶȻ�pid����
  PID_Value_Typedef sucker_roll_angelpidval = {SUCKER_ROLL_ANGEL_PID_KP, SUCKER_ROLL_ANGEL_PID_KI, SUCKER_ROLL_ANGEL_PID_KD};
	PID_Value_Typedef sucker_pitch_angelpidval = {SUCKER_PITCH_ANGEL_PID_KP, SUCKER_PITCH_ANGEL_PID_KI, SUCKER_PITCH_ANGEL_PID_KD};
	PID_Value_Typedef uplift_pidval_anglepidval_up = {UPLIFT_ANGEL_PID_KP,UPLIFT_ANGEL_PID_KI,UPLIFT_ANGEL_PID_KD};
	action_roll.sucker_roll_angleset = -6.85f;//��ʼ��6020����Ƕȼ����䴦���ض�����ֵ

	PUMP_OPEN;//������
	PUMP_OFF;//������
  BUTTEN_TRIG_PIN1;//΢�����س�ʼ��
	BUTTEN_TRIG_PIN2;//
	
	//�ٶȻ���ʼ��
	PID_init(&gimbal_move.gimbal_roll_pid,PID_POSITION,&gimbal_roll_pidval,M2006_MAX_OUT,M2006_MAX_IOUT);//6020
	
	for(int i=0;i<2;i++)
		{
			PID_init(&action_pid.forward_pid[i],PID_POSITION,&forward_pidval,5000,2500);//3508
			PID_init(&action_pid.uplift_pid_up[i],PID_POSITION,&uplift_pidval_up,20000,2000);//3508		
	    PID_init(&action_pid.sucker_pitch_pid[i],PID_POSITION,&sucker_pitch_pidval,14000.0f,10000.0f);//3508	
		}
		  PID_init(&action_pid.sucker_roll_pid,PID_POSITION,&sucker_roll_pidval,20000.0f,5000.0f);//6020

	//�ǶȻ���ʼ��
	for(int i=0;i<2;i++)
	{
  PID_init(&action_pid.sucker_pitch_angelpid[i],PID_POSITION,&sucker_pitch_angelpidval,2200,3000);
	PID_init(&action_pid.uplift_angelpid_up[i],PID_POSITION,&uplift_pidval_anglepidval_up,2200.0f,2000.0f);//3508			
  }
	PID_init(&action_pid.sucker_roll_angelpid,PID_POSITION,&sucker_roll_angelpidval,8000.0f,0.4f);//6020
  

}
fp32 asd,sdf,qwrw;
static void action_update(void)
{
	//�����ȡ����
	action_roll.sucker_roll_speed = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[4].speed;	//roll���ٶȶ�ȡ
	action_roll.sucker_roll_angle = Moto_Data2[4].angle;	//roll��Ƕȶ�ȡ
	
	action_uplift.uplift_speed[0] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[1].speed;//���̧���ٶȶ�ȡ
	action_uplift.uplift_speed[1] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[3].speed;//�Ҳ�̧���ٶȶ�ȡ
	action_uplift.uplift_last_angleset[0] = action_uplift.uplift_angleset[0];//���̧����һ�νǶȶ�ȡ
	action_uplift.uplift_last_angleset[1] = action_uplift.uplift_angleset[1];//���̧����һ�νǶȶ�ȡ
	action_uplift.uplift_angle[0] = Moto_Data2[1].angle;//���̧���Ƕȶ�ȡ
	action_uplift.uplift_angle[1] = Moto_Data2[3].angle;//���̧���Ƕȶ�ȡ

	action_forward.forward_speed[0] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data1[4].speed;//���ǰ���ٶȶ�ȡ
	action_forward.forward_speed[1] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data1[5].speed;//�Ҳ�ǰ���ٶȶ�ȡ

	action_forward.forward_last_angle[0] = action_forward.forward_angle[0];//���ǰ����һ�νǶȶ�ȡ
	action_forward.forward_last_angle[1] = action_forward.forward_angle[1];//�Ҳ�ǰ����һ�νǶȶ�ȡ

	action_forward.forward_angle[0] = Moto_Data1[4].angle;//���ǰ��Ƕȶ�ȡ
	action_forward.forward_angle[1] = Moto_Data1[5].angle;//�Ҳ�ǰ��Ƕȶ�ȡ
	action_forward.forward_angle_sum[0] = (action_forward.forward_angle[0] - action_forward.forward_last_angle[0]);
	
	action_forward.forward_angle_sum[1] = (action_forward.forward_angle[1] - action_forward.forward_last_angle[1]);

	action_pitch.sucker_pitch_speed = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[2].speed;	//pitch���ٶȶ�ȡ
	action_pitch.sucker_pitch_last_angle = Moto_Data2[2].angle_last;//pitch����һ�νǶȶ�ȡ
	action_pitch.sucker_pitch_angle = Moto_Data2[2].angle;//pitch���ϽǶȶ�ȡ	
	action_pitch.sucker_pitch_angle_sum = (action_pitch.sucker_pitch_angle - action_pitch.sucker_pitch_last_angle);//����Ƕȱ仯��
	action_pitch.sucker_pitch_angle_add1+=action_pitch.sucker_pitch_angle_sum;//����Ƕȱ仯��
	action_pitch.sucker_pitch_last_angleset = action_pitch.sucker_pitch_angleset;

	sdf+=action_forward.forward_angle_sum[1];
	asd+=action_forward.forward_angle_sum[0];

	microswitch_uplift_bottoml = BUTTEN_TRIG_PIN1;
	microswitch_uplift_bottomr = BUTTEN_TRIG_PIN2;

	
	
	
	gimbal_move.gimbal_roll_speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[0].speed;
	gimbal_move.gimbal_roll_angle = Moto_Data2[0].angle_sum*360.0/8192.0/36.0;

}
static void action_mode_set(void)
{
	if(switch_is_up(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_NO_FORCE;
	if(switch_is_mid(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_RC;
	if(switch_is_down(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_KEYBOARD;
}
fp32 highl,highr,highl_set,highr_set,b,c,d,error_l,error_r;
int miubl_cail,miubr_cail,miub_sum;
fp32 awdp,awd3,eifuweifu;
static void action_control_set(void)
{
	
	if(action_mode == ACTION_RC)//ң��������
	{
		if(switch_is_up(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controlup();
	  if(switch_is_mid(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controlmid();
		if(switch_is_down(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controldown();
		
	}
	
	if(action_mode == ACTION_KEYBOARD)
	{
		action_keyboard_control();
	}
		
	

	//ǰ��PID
	action_forward.forward_give_current[0] = PID_calc(&action_pid.forward_pid[0],action_forward.forward_speed[0],action_forward.forward_speedset[0]);//
	action_forward.forward_give_current[1] = PID_calc(&action_pid.forward_pid[1],action_forward.forward_speed[1],action_forward.forward_speedset[1]);
	
	//̧���ǶȻ�PID
//	if((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH]>0)
//	{
	action_uplift.uplift_give_angle[0] = PID_calc(&action_pid.uplift_angelpid_up[0],Moto_Data2[1].angle_sum*360.0/19.0/8192.0,action_uplift.uplift_angleset[0]);
  action_uplift.uplift_give_angle[1] = PID_calc(&action_pid.uplift_angelpid_up[1],Moto_Data2[3].angle_sum*360.0/19.0/8192.0,action_uplift.uplift_angleset[1]);
	action_uplift.uplift_give_current[0] = PID_calc(&action_pid.uplift_pid_up[0],action_uplift.uplift_speed[0],action_uplift.uplift_give_angle[0]);
	action_uplift.uplift_give_current[1] = PID_calc(&action_pid.uplift_pid_up[1],action_uplift.uplift_speed[1],action_uplift.uplift_give_angle[1]);
//	}
//	if((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH]<0)
//	{	
//  uplift_give_angle[0] = PID_calc(&uplift_angelpid_up[0],Moto_Data2[1].angle_sum*360.0/19.0/8192.0,uplift_angleset[0]);
//  uplift_give_angle[1] = PID_calc(&uplift_angelpid_up[1],Moto_Data2[3].angle_sum*360.0/19.0/8192.0,uplift_angleset[1]);
//	uplift_give_current[0] = PID_calc(&uplift_pid_up[0],uplift_speed[0],uplift_give_angle[0]);
//	uplift_give_current[1] = PID_calc(&uplift_pid_up[1],uplift_speed[1],uplift_give_angle[1]);
//	}
  //pitch��ǶȻ�PID
	action_pitch.sucker_pitch_give_angle = PID_calc(&action_pid.sucker_pitch_angelpid[1],Moto_Data2[2].angle_sum*360.0/19.0/8192.0,action_pitch.sucker_pitch_angleset);
	action_pitch.sucker_pitch_give_current = PID_calc(&action_pid.sucker_pitch_pid[1],action_pitch.sucker_pitch_speed,action_pitch.sucker_pitch_give_angle);
//first_order_filter_cali(&action_pitchone.chassis_cmd_slow_set_vx,action_pitchone.sucker_pitchone_give_current[1]);
//action_pitchone.sucker_pitchone_give_current[1] = action_pitchone.chassis_cmd_slow_set_vx.out;
  //roll��ǶȻ�PID
  action_roll.sucker_roll_give_angle = PID_calc(&action_pid.sucker_roll_angelpid,Moto_Data2[4].angle_sum*360.0/19.0/8192.0,action_roll.sucker_roll_angleset);
	action_roll.sucker_roll_give_current = PID_calc(&action_pid.sucker_roll_pid,action_roll.sucker_roll_speed,action_roll.sucker_roll_give_angle);

	//��������Ϊ0
	
 gimbal_move.gimbal_roll_current = PID_calc(&gimbal_move.gimbal_roll_pid,gimbal_move.gimbal_roll_speed,gimbal_move.gimbal_roll_speed_set);

	if(action_mode == ACTION_NO_FORCE)
	{
		action_forward.forward_speedset[0] = 0.0f;
		action_forward.forward_speedset[1] = 0.0f;
		action_uplift.uplift_speed[0] = 0.0f;
		
		action_uplift.uplift_speed[1] = 0.0f;
	}

}
int uwygfuywf,whgbwber;
static void calibration(void)
{
		//ǰ��ң�����ٶ�����


	action_forward.forward_speedset[0] = -1.0;
	action_forward.forward_speedset[1] = 1.0;
//		whgbwber++;
//  if(Moto_Data1[4].current<-4800)
//	{
//		action_forward.limit_fla1g[0] = 2;
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//		uwygfuywf++;
//	}
//	else 	action_forward.limit_fla1g[0] = 0;

//	//���������ת��λ
//		if(Moto_Data1[5].current>4800)
//	{
//		action_forward.limit_fla1g[1] = 2;
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//		whgbwber++;
//	}
//	else 	action_forward.limit_fla1g[1] = 0;


//	if(action_forward.limit_fla1g[0] == 2)
//	{
//		action_forward.forward_speedset[0] = 0;
////		whgbwber++;
//	}
//	if(action_forward.limit_fla1g[1] == 2)
//	{
//	  action_forward.forward_speedset[1] = 0;
//  }
	forward_control();
		
	
//  if(action_uplift.uplift_limit_flag[0] == 0||action_uplift.uplift_limit_max[1] == 0)
//	{
	  action_uplift.uplift_angleset[0] -= 0.2f;
	  action_uplift.uplift_angleset[1] += 0.2f;
//	}
	  uplift_control();
	  if(action_uplift.limit_init_ready_u[0] == 1&&action_uplift.limit_init_ready_u[1] == 1)
	 {
		  action_uplift.uplift_angleset[0] -= 0.0f;
	    action_uplift.uplift_angleset[1] += 0.0f;
	 }
	
	
	action_pitch.sucker_pitch_angleset += 0.1f;
	pitch_control();	
//  gimbal_move.gimbal_roll_speed_set = 
}
int wueinfiwef;
static void action_rc_controlup(void)
{
	//ң��������
	fp32 rc_r;
	//ң��������
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[4],rc_r,20);
	
	//�����ٶ�����
  if(rc_r>40)
	{
		wueinfiwef = 1;
	}
	else if(rc_r<-40) wueinfiwef = 0;
	
	if(wueinfiwef == 1)
	{
		calibration();
	
  }
	if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.pitch_limit_flag==1)
  {
		wueinfiwef = 0;//�˳���ʼ��ģʽ��־λ
	}
}

fp32 asd,sdf;
fp32 fjvghsif;
static void action_rc_controlmid(void)
{

	//ң��������

	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_CLAW_CH],action_forward.rc_forward,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH],action_uplift.rc_uplift,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_SUCKERYAW_CH],action_pitch.rc_pitch,ACTION_RC_DEADLINE);
	//�����ٶ�����
	//ǰ��ң�����ٶ�����
	action_forward.forward_speedset[0] = action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
	action_forward.forward_speedset[1] = -action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
	//ǰ����λ����
  forward_control();
	
	//����pitch��ң�����Ƕ�����
	action_pitch.sucker_pitch_angleset += action_pitch.rc_pitch/5000.0f*ACTION_VSY_MAX;
	//����pitch����λ����	
	pitch_control();
	
	if(action_uplift.init_u == 0)
	{
    action_uplift.uplift_angleset[0]+=0.2f;
  	if(action_uplift.uplift_angleset[0]>14.0f)
  	{
		  action_uplift.init_u = 1;
  	}
	}
	action_uplift.uplift_angleset[0] += action_uplift.rc_uplift/2000.0f*ACTION_VTRAN_MAX;//
	action_uplift.uplift_angleset[1] -= action_uplift.rc_uplift/2000.0f*ACTION_VTRAN_MAX;

	uplift_control();

}


fp32 ewgeqg;
int uyegfyuwe;
static void action_rc_controldown(void)
{
	fp32 rc_pump,rc_sucker_roll;
	//ң��������

	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_CLAW_CH],rc_pump,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[0],rc_sucker_roll,ACTION_RC_DEADLINE);
	
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[3],ewgeqg,ACTION_RC_DEADLINE);
	
	
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[2],gimbal_move.rc_gimbal_roll,ACTION_RC_DEADLINE);
	//�����ٶ�����

	if(rc_sucker_roll>30)
		{action_roll.sucker_roll_angleset += 0.005f;}
	else if(rc_sucker_roll<-30)
		{action_roll.sucker_roll_angleset -= 0.005f;}
	else
		{action_roll.sucker_roll_angleset += 0.0f;}

		roll_control();

	
	if(rc_pump>400)
	{
		PUMP_OPEN;
		
	}
	if(rc_pump<-400)
	{
		PUMP_OFF;		
		
	}
	
	
	gimbal_move.gimbal_roll_speed_set = gimbal_move.rc_gimbal_roll/660*3.0f;
	gimbal_roll_control();
	if(gimbal_move.gimbal_roll_limit_flag == 1)
	{
		uyegfyuwe = 2;
		if(ewgeqg>30)
		{
			uyegfyuwe = 1;
//			if(gimbal_move.gimbal_roll_interval<176.0)
//			{
//			  gimbal_move.gimbal_roll_speed_set = 0.8;
//				if(gimbal_move.gimbal_roll_interval>176.0)
//			  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0;
//				}
//			}
//			if(gimbal_move.gimbal_roll_interval>176.0)
//			{
//			  gimbal_move.gimbal_roll_speed_set = -0.8;
//				if(gimbal_move.gimbal_roll_interval<176.0)
//			  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0;
//				}
//			}
			if(gimbal_move.gimbal_roll_interval<160.0f)
			{
			  gimbal_move.gimbal_roll_speed_set = 1.2f;
				if(gimbal_move.gimbal_roll_interval>160.0f)
			  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
				}
			}
			if(gimbal_move.gimbal_roll_interval>192.0f)
			{
			  gimbal_move.gimbal_roll_speed_set = -1.2f;
				if(gimbal_move.gimbal_roll_interval<192.0f)
			  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
				}
			}
		}
		else if(ewgeqg<-30)
		{
			gimbal_move.gimbal_roll_speed_set = 1.2f;
			if(gimbal_move.gimbal_roll_interval>458.0f)
			{
				gimbal_move.gimbal_roll_speed_set = 0;
			}
		}
	}

}

static void uplift_control(void)
{


	action_uplift.uplift_angleset_sum[0] = (action_uplift.uplift_angleset[0] - action_uplift.uplift_last_angleset[0]);
	awdp+=action_uplift.uplift_angleset_sum[0];
	action_uplift.uplift_angleset_sum[1] = (action_uplift.uplift_angleset[1] - action_uplift.uplift_last_angleset[1]);
	awd3+=action_uplift.uplift_angleset_sum[1];
	action_uplift.uplift_angle_mileage[0] = Moto_Data2[1].angle_sum*360.0f/19.0f/8192.0f; //�Ƕ���� �Ƕ���
	action_uplift.uplift_angle_mileage[1] = Moto_Data2[3].angle_sum*360.0f/19.0f/8192.0f; //�Ƕ���� �Ƕ���
	action_uplift.uplift_mileage[0] = Moto_Data2[1].angle_sum/19.0f/8192.0f*38.0f*3.14f; //������� ��λmm
	action_uplift.uplift_mileage[1] = Moto_Data2[3].angle_sum/19.0f/8192.0f*38.0f*3.14f; //������� ��λmm
	asd = Moto_Data2[3].angle_sum/19.0/8192.0*38*3.14/360.0;
	
	if(Moto_Data2[1].current<-3000)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
	{
		action_uplift.uplift_limit_flag[0] = 1;//��־λΪ1
		action_uplift.uplift_limit_max[0] = action_uplift.uplift_mileage[0];//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_uplift.uplift_limit_max[0] = action_uplift.uplift_limit_max[0];//ȷ�����ֵ Ϊ��תλ��
	}
	//�������ǰ��ת��λ
		if(Moto_Data2[3].current>3000)//ͨ������ֵ�ж�����Ƿ��ת
	{
		action_uplift.uplift_limit_flag[1] = 1;
		action_uplift.uplift_limit_max[1] = action_uplift.uplift_mileage[1];//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_uplift.uplift_limit_max[1] = action_uplift.uplift_limit_max[1];//ȷ�����ֵ Ϊ��תλ��
	}
	
	if(action_uplift.uplift_limit_flag[0] == 1)//ͨ����־Ϊ���˶�������λ
	{
		action_uplift.uplift_interval[0] = action_uplift.uplift_mileage[0] - action_uplift.uplift_limit_max[0];//�����˶�����
    if(action_uplift.uplift_interval[0]<3.0f)//�Ҳ���������Сֵ
	  {
		  action_uplift.uplift_interval[0] = 3.0f;
		  if(action_uplift.uplift_angleset[0]<14.0f)//����������Сֵʱ�ٶȴ����� �������������ٶ�
		  {
			  action_uplift.uplift_angleset[0] -= 0;
			  action_uplift.uplift_angleset[0] = 14.0f;
				action_uplift.limit_init_ready_u[0] = 1;
		  }
	  }
		else
				action_uplift.limit_init_ready_u[0] = 0;
		if(action_uplift.uplift_interval[0]>240.0f)
	  {
		  action_uplift.uplift_interval[0] = 240.0f;
		  if(action_uplift.uplift_angleset[0]>748.0f)
		  {
		  	action_uplift.uplift_angleset[0] = 748.0f;
		  }
	  }
		if(action_forward.limit_flag[0] == 2)
		{
			if(action_forward.forward_interval[0]<90.0f)
			{
				if(action_uplift.uplift_angleset[0]>404.0f)
				{
					action_uplift.uplift_angleset[0] = 404.0f;
					
				}
				
			}
			
		}
	}
	if(action_uplift.uplift_limit_flag[1] == 1)
	{
		action_uplift.uplift_interval[1] = action_uplift.uplift_mileage[1] - action_uplift.uplift_limit_max[1];
    if(action_uplift.uplift_interval[1]>-1.1f)//�Ҳ����������ֵ
	  {
	  	action_uplift.uplift_interval[1] = -1.1f;
	  	if(action_uplift.uplift_angleset[1]>0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
	  	{
			  action_uplift.uplift_angleset[1] += 0;
			  action_uplift.uplift_angleset[1] = 0;
				action_uplift.limit_init_ready_u[1] = 1;
	  	}
	  }	
			else
				action_uplift.limit_init_ready_u[1] = 0;
//		
		if(action_uplift.uplift_interval[1]<-240.0f)
	  {
		  action_uplift.uplift_interval[1] = -240.0f;
		  if(action_uplift.uplift_angleset[1]<-748.0f)
		  {
		  	action_uplift.uplift_angleset[1] = -748.0f;
		  }
	  }
//    if(action_uplift.uplift_interval[1]<0.5f)
//	  {
//	  	action_uplift.uplift_interval[1] = 0.5f;
//	  	if(action_uplift.uplift_angleset[1]<0)
//	  	{
//	  		action_uplift.uplift_angleset[1] = 0;
//	  	}
//	  }	
		if(action_forward.limit_flag[1] == 2)
		{
			if(action_forward.forward_interval[1]>-90.0f)
			{
				if(action_uplift.uplift_angleset[1]<-390.0f)
				{
					action_uplift.uplift_interval[1] = -390.0f;
					
				}
				
			}
			
		}		
		
  }
}
static void forward_control(void)
	


{

	action_forward.forward_angle_mileage[0] = Moto_Data1[4].angle_sum*360.0f/19.0f/8192.0f; //�Ƕ���� �Ƕ���
	action_forward.forward_angle_mileage[1] = Moto_Data1[5].angle_sum*360.0f/19.0f/8192.0f; //�Ƕ���� �Ƕ���
	action_forward.forward_mileage[0] = Moto_Data1[4].angle_sum/19.0f/8192.0f*38.0f*3.14f; //������� ��λmm
	action_forward.forward_mileage[1] = Moto_Data1[5].angle_sum/19.0f/8192.0f*38.0f*3.14f; //������� ��λmm
	
//	action_forward.forward_init_time++;
//	if(action_forward.forward_init_time<1000)
//	{
//		action_forward.forward_speedset[0] = action_forward.forward_speedset[1] = 0;
//		init_angle = Moto_Data1[4].angle;
//	}
//	else
//	{
//		init_angle = init_angle;
//		action_forward.forward_init_time = 1000;
//		
//	}
//	action_forward.forward_last_angle[0] = action_forward.forward_angle[0];
//	action_forward.forward_angle_sum[0] = (action_forward.forward_angle[0] - action_forward.forward_last_angle[0]);
//	asd+=action_forward.forward_angle_sum[0];
//	
//	action_forward.forward_last_angle[1] = action_forward.forward_angle[1];
//	action_forward.forward_angle_sum[1] = (action_forward.forward_angle[1] - action_forward.forward_last_angle[1]);
//	sdf+=action_forward.forward_angle_sum[1];//	asd+=action_forward.forward_angle_sum[0];
	//�Ҳ�����ǰ��ת��λ
	if(Moto_Data1[4].current>4900)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
	{
		action_forward.limit_flag[0] = 1;//��־λΪ1
		action_forward.forward_limit_max[0] = action_forward.forward_mileage[0];//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_max[0] = action_forward.forward_limit_max[0];//ȷ�����ֵ Ϊ��תλ��
	}
	//�������ǰ��ת��λ
		if(Moto_Data1[5].current<-4900)//ͨ������ֵ�ж�����Ƿ��ת
	{
		action_forward.limit_flag[1] = 1;
		action_forward.forward_limit_max[1] = action_forward.forward_mileage[1];//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_max[1] = action_forward.forward_limit_max[1];//ȷ�����ֵ Ϊ��תλ��
	}
	
	if(action_forward.limit_flag[1] == 1&&action_forward.limit_flag[0] == 1)//ͨ����־Ϊ���˶�������λ
	{
		action_forward.forward_interval[0] = action_forward.forward_mileage[0] - action_forward.forward_limit_max[0];//�����˶�����
		action_forward.forward_interval[1] = action_forward.forward_mileage[1] - action_forward.forward_limit_max[1];
    if(action_forward.forward_interval[0]<-438.0f)//�Ҳ���������Сֵ
	  {
		  action_forward.forward_interval[0] = -438.0f;
		  if(action_forward.forward_speedset[0]<0)//����������Сֵʱ�ٶȴ����� �������������ٶ�
		  {
			  action_forward.forward_speedset[0] = 0;
		  }
	  }
    if(action_forward.forward_interval[0]>-0.5f)//�Ҳ����������ֵ
	  {
	  	action_forward.forward_interval[0] = -0.5f;
	  	if(action_forward.forward_speedset[0]>0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
	  	{
	  		action_forward.forward_speedset[0] = 0;
	  	}
	  }	
		
		if(action_forward.forward_interval[1]>438.0f)
	  {
		  action_forward.forward_interval[1] = 438.0f;
		  if(action_forward.forward_speedset[1]>0)
		  {
		  	action_forward.forward_speedset[1] = 0;
		  }
	  }
    if(action_forward.forward_interval[1]<0.5f)
	  {
	  	action_forward.forward_interval[1] = 0.5f;
	  	if(action_forward.forward_speedset[1]<0)
	  	{
	  		action_forward.forward_speedset[1] = 0;
	  	}
	  }			
		
  }

	//�Ҳ�������ת��λ
	if(Moto_Data1[4].current<-4800)
	{
		action_forward.limit_flag[0] = 2;
		action_forward.forward_limit_min[0] = action_forward.forward_mileage[0];
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_min[0] = action_forward.forward_limit_min[0];
	}
	//���������ת��λ
		if(Moto_Data1[5].current>4800)
	{
		action_forward.limit_flag[1] = 2;
		action_forward.forward_limit_min[1] = action_forward.forward_mileage[1];
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_min[1] = action_forward.forward_limit_min[1];
	}
	if(action_forward.limit_flag[1] == 2&&action_forward.limit_flag[0] == 2)
	{
		action_forward.forward_interval[0] = action_forward.forward_mileage[0] - action_forward.forward_limit_min[0];
		action_forward.forward_interval[1] = action_forward.forward_mileage[1] - action_forward.forward_limit_min[1];
    if(action_forward.forward_interval[0]>438.0f)
	  {
		  action_forward.forward_interval[0] = 438.0f;
		  if(action_forward.forward_speedset[0]>0)
		  {
			  action_forward.forward_speedset[0] = 0;
		  }
	  }
    if(action_forward.forward_interval[0]<0.5f)
	  {
	  	action_forward.forward_interval[0] = 0.5f;
			action_forward.limit_init_ready_f[0] = 1;//��λ������Сλ�ú�������ʼ��λ��
	  	if(action_forward.forward_speedset[0]<0)
	  	{
	  		action_forward.forward_speedset[0] = 0;
	  	}
	  }	
		else action_forward.limit_init_ready_f[0] = 0;//δ�����ʼ��λ��
		
		if(action_forward.forward_interval[1]<-438.0f)
	  {
		  action_forward.forward_interval[1] = -438.0f;
		  if(action_forward.forward_speedset[1]<0)
		  {
		  	action_forward.forward_speedset[1] = 0;
		  }
	  }
    if(action_forward.forward_interval[1]>-0.5f)
	  {
	  	action_forward.forward_interval[1] = -0.5f;
			action_forward.limit_init_ready_f[1] = 1;//��λ������Сλ�ú�������ʼ��λ��
	  	if(action_forward.forward_speedset[1]>0)
	  	{
	  		action_forward.forward_speedset[1] = 0;
	  	}
	  }
		else action_forward.limit_init_ready_f[1] = 0;//δ�����ʼ��λ��
		
  }
		
	
	
	
}



static void pitch_control(void)
{
	
//	if(Moto_Data2[2].current>6200)
//	{
//		action_pitch.sucker_pitch_angleset = action_pitch.sucker_pitch_angleset;
//	}
	
//	action_pitch.sucker_pitch_last_angle
	action_pitch.pitch_angle_mileage = Moto_Data2[2].angle_sum*360.0f/19.0f/8192.0f; //�Ƕ���� �Ƕ���
	
	
	action_pitch.sucker_pitch_angleset_sum = (action_pitch.sucker_pitch_angleset - action_pitch.sucker_pitch_last_angleset);
	
	action_pitch.sucker_pitch_angle_add+=action_pitch.sucker_pitch_angleset_sum;
  if(action_pitch.limit_init_p == 0)
	{
	  if(action_pitch.sucker_pitch_angle_add - action_pitch.pitch_angle_mileage>15||action_pitch.sucker_pitch_angle_add - action_pitch.pitch_angle_mileage<-15)
  	{
	  	action_pitch.pitch_limit_flag = 1;
	  	action_pitch.pitch_limit_max = action_pitch.pitch_angle_mileage;
	  	action_pitch.pitch_limit_min = action_pitch.pitch_limit_max - 278.0f;
//		action_pitch.pitch_limit_min = action_pitch.sucker_pitch_angleset;
//		action_pitch.pitch_limit_min = action_pitch.pitch_limit_max - 278.0f;

	  	
  		action_pitch.limit_init_p = 1;
    
  	}
  	else		
  	{
	  	action_pitch.pitch_limit_flag = 0;
//		action_pitch.pitch_limit_min = action_pitch.pitch_limit_min;
    }
    

  }
	if(action_pitch.pitch_limit_flag == 1)
	  {
	 		action_pitch.pitch_max_angleset = action_pitch.sucker_pitch_angleset;
	  	if(action_pitch.sucker_pitch_angle_add > action_pitch.pitch_angle_mileage)
	  	{
	  		action_pitch.sucker_pitch_angle_add = action_pitch.pitch_angle_mileage;
//	  		action_pitch.sucker_pitch_angleset = action_pitch.pitch_angle_mileage;
				action_pitch.limit_init_ready_p = 1;
	  	}
	  }
		  if(action_pitch.pitch_angle_mileage<action_pitch.pitch_limit_max-5.0f)
	  {
	  	action_pitch.limit_init_ready_p = 0;
	  }
	if(action_pitch.limit_init_p == 1)
	{
	if(action_pitch.sucker_pitch_angleset<action_pitch.pitch_limit_min)
	{
		action_pitch.sucker_pitch_angleset-=0;
		action_pitch.sucker_pitch_angleset = action_pitch.pitch_limit_min;
	}
	if(action_pitch.sucker_pitch_angleset>action_pitch.pitch_limit_max)
	{
		action_pitch.sucker_pitch_angleset+=0;
		action_pitch.sucker_pitch_angleset = action_pitch.pitch_limit_max;
	}
  }
}



static void gimbal_roll_control(void)
{
	
	
	gimbal_move.gimbal_roll_angle_mileage = Moto_Data2[0].angle_sum*360.0f/36.0f/8192.0f; //�Ƕ���� �Ƕ���
	gimbal_move.gimbal_roll_mileage = Moto_Data2[0].angle_sum/19.0f/8192.0f*100.0f*3.14f; //������� ��λmm
	
	//�Ҳ�����ǰ��ת��λ
	if(Moto_Data2[0].current<-3000)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
	{
		gimbal_move.gimbal_roll_limit_flag = 1;//��־λΪ1
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//ȷ�����ֵ Ϊ��תλ��
	}

	if(gimbal_move.gimbal_roll_limit_flag == 1)//ͨ����־Ϊ���˶�������λ
	{
		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//�����˶�����
    if(gimbal_move.gimbal_roll_interval>520.0f)//�Ҳ���������Сֵ
	  {
		  gimbal_move.gimbal_roll_interval = 520.0f;
		  if(gimbal_move.gimbal_roll_speed_set>0)//����������Сֵʱ�ٶȴ����� �������������ٶ�
		  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  }
	  }
    if(gimbal_move.gimbal_roll_interval<0.6f)//�Ҳ����������ֵ
	  {
	  	gimbal_move.gimbal_roll_interval = 0.6f;
	  	if(gimbal_move.gimbal_roll_speed_set<0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
	  	{
	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
	  	}
	  }	

		
  }
	
	
	
		if(Moto_Data2[0].current>3000)//ͨ������ֵ�ж��Ҳ��Ƿ��ת
	{
		gimbal_move.gimbal_roll_limit_flag = 2;//��־λΪ1
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//��ʱ���ֵΪ������ֵ ��Ϊ��ֵ
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//ȷ�����ֵ Ϊ��תλ��
	}

	if(gimbal_move.gimbal_roll_limit_flag == 2)//ͨ����־Ϊ���˶�������λ
	{
		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//�����˶�����
    if(gimbal_move.gimbal_roll_interval<-520.0f)//�Ҳ���������Сֵ
	  {
		  gimbal_move.gimbal_roll_interval = -520.0f;
		  if(gimbal_move.gimbal_roll_speed_set<0)//����������Сֵʱ�ٶȴ����� �������������ٶ�
		  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  }
	  }
    if(gimbal_move.gimbal_roll_interval>0.6f)//�Ҳ����������ֵ
	  {
	  	gimbal_move.gimbal_roll_interval = 0.6f;
	  	if(gimbal_move.gimbal_roll_speed_set>0)//�����������ֵʱ�ٶȴ����� ��������ǰ�����ٶ�
	  	{
	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
	  	}
	  }	

		
  }
}



static void roll_control(void)
{
		action_roll.roll_angle = action_roll.sucker_roll_angleset*19;
		
	if(action_roll.roll_angle>50)
	{
		action_roll.sucker_roll_angleset=50.0f/19.0f;
	}	
	if(action_roll.roll_angle<-310)
	{
		action_roll.sucker_roll_angleset=-310.0f/19.0f;
	}
	
	
}
	fp32 aiuf,uawhfuaf,qegqeg;
fp32 iqufhi;
static void action_keyboard_control(void)
{

			rc_deadband_limit((fp32)rc_ctrl.rc.ch[2],aiuf,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[3],uawhfuaf,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[1],qegqeg,ACTION_RC_DEADLINE);
  //------------------------ģʽ�л�-------------------------//
//	if(aiuf<-40&&Key_A.mode_switch_flag == 0)
//	{
////	if(gimbal_move.rc_gimbal_roll>30&&Key_A.mode_switch_flag == 0)
////	{	
//		//SHIFT���л����ϲ��ƶ�ģʽ
//		Key_A.control_A++;
//		if(Key_A.control_A > 300)
//		{
//		  Key_A.mode_switch_flag = 1;
//		  key_mode_A = ACTION_MOVE;
//		}
//	}
//	else if(aiuf>40)
//	{
////	else if(gimbal_move.rc_gimbal_roll<-30&&Key_A.mode_switch_flag == 1)
////	{
//		//CTRL���л����²��ƶ�ģʽ
//		Key_A.control_A++;
//		if(Key_A.control_A > 300)
//		{
////		  Key_A.mode_switch_flag = 0;
//		  key_mode_A = ACTION_CHASSIS;
//		}
//		
//		
//		if(uawhfuaf>40)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_gold++;
//			if(Key_A.control_gold > 300)
//		  {
//		    Key_A.mode_switch_flag = 2;
//		    key_mode_A = ACTION_GOLD;
//	  	}
//		}
//		else if(uawhfuaf<-40)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_silver++;
//			if(Key_A.control_silver > 300)
//		  {
//		     Key_A.mode_switch_flag = 3;
//		    key_mode_A = ACTION_SILVER;
//	  	}
//		}
//		else if(qegqeg>200)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_catch++;
//			if(Key_A.control_catch > 300)
//		  {
//		     Key_A.mode_switch_flag = 4;
//		    key_mode_A = ACTION_CATCH;
//	  	}
//		}
//		else
//		{
//			Key_A.control_gold = 0;
//			Key_A.control_silver = 0;
//			Key_A.control_catch = 0;
//			
//		}
//	}
//	else Key_A.control_A = 0;
	
//  //------------------------ģʽ�л�-------------------------//
	if(uawhfuaf>40)
	{
		
		  key_mode_A = ACTION_CATCH;
		
	}
	
//		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT&&Key_A.mode_switch_flag == 0)
//	{
//	if(aiuf>30&&Key_A.fast_switch_flag == 0)
//	{	
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)
	{	
		//SHIFT���л����ϲ��ƶ�ģʽ
		Key_A.control_Aa++;
		Key_A.mode_switch_flag = 1;
		if(Key_A.control_Aa > 300)
		{
		  key_mode_A = ACTION_MOVE;
		}
	}
//	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL&&Key_A.fast_switch_flag == 0)
//	{
////	else if(aiuf<-30&&Key_A.fast_switch_flag == 0)
////	{
////	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL&&Key_A.mode_switch_flag == 0)
////	{
//		//CTRL���л����²��ƶ�ģʽ
//		Key_A.control_A++;
//		Key_A.mode_switch_flag = 1;
//		if(Key_A.control_A > 300)
//		{
//		  key_mode_A = ACTION_CHASSIS;
//		}
//	}
	else {Key_A.control_Aa = 0;
		Key_A.mode_switch_flag = 0;}
	
	
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)
	{
//		if(aiuf<-30)
//	{
//		Key_A.mode_switch_flag = 0;
//		Key_A.fast_switch_flag = 0;
    Key_A.control_Ac++;
		Key_A.mode_switch_flag = 1;
    if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
		{
//		if(uawhfuaf>30)
//		{
			
			Key_A.control_Ac = 0;
			Key_A.control_gold++;
			if(Key_A.control_gold > 300)
		  {
//				Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 1;
		    key_mode_A = ACTION_GOLD;
	  	}
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
		{
//				else if(uawhfuaf<-30)
//		{
			Key_A.control_Ac = 0;
			Key_A.control_silver++;
			if(Key_A.control_silver > 300)
		  {
//			  Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 2;
		    key_mode_A = ACTION_SILVER;
	  	}
		}
	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
	{
//				else if(qegqeg>30)
//		{
			Key_A.control_Ac = 0;
			Key_A.control_catch++;
			if(Key_A.control_catch > 300)
		  {
//			  Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 3;
		    key_mode_A = ACTION_CATCH;
	  	}
		}
	
		else
		{
			Key_A.fast_switch_flag = 0;
			Key_A.control_gold = 0;
			Key_A.control_silver = 0;
			Key_A.control_catch = 0;
					
		if(Key_A.control_Ac > 300)
		{
		  key_mode_A = ACTION_CHASSIS;
		}
		}
	}
	else
	{Key_A.mode_switch_flag = 0;
  	Key_A.control_Ac = 0;
	}
	
	
	 //------------------------��ʼ��ģʽ-------------------------//
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
	{
		//R���л�����ʼ��ģʽ
		Key_A.init_A++;
		if(Key_A.init_A > 300)
		{
		  Key_A.init_switch_flag = 1;
		}
		 
	}
	else 
	{
		Key_A.init_A = 0;
    Key_A.init_switch_flag = 0;
	}
	
	if(Key_A.init_switch_flag == 1)
    {
		  calibration();
		  forward_control();
		  pitch_control();
		  uplift_control();
      if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.limit_init_ready_p==1)
      {
		  	Key_A.init_switch_flag = 0;//�˳���ʼ��ģʽ��־λ
		  	Key_A.init_ready_flag = 1;
		  }
	  }
	
	
// //------------------------�ȼ�ģʽ-------------------------//
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT&&rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
	{
//	if(aiuf>30)
//	{
		//R���л�����ʼ��ģʽ
		Key_A.help_A++;
		if(Key_A.help_A > 100)
		{
		  Key_A.help_switch_flag_A = 1;
		}
	}
	else Key_A.help_A = 0;
  if(Key_A.help_switch_flag_A == 1)
  {
    
		action_forward.limit_flag[0] = 0;
		action_forward.limit_flag[1] = 0;
		action_uplift.uplift_limit_flag[0] = 0;
		action_uplift.uplift_limit_flag[1] = 0;
		action_pitch.pitch_limit_flag = 0;
		action_pitch.limit_init_p = 0;
			Key_A.help_switch_flag_A = 0;//�˳���ʼ��ģʽ��־λ
		action_mode = ACTION_EMPTY;
			Key_A.help_ready_flag_A = 1;
		
	}


  //------------------------�ϲ��ƶ�ģʽ-------------------------//
  if(key_mode_A == ACTION_MOVE&&Key_A.init_switch_flag == 0)
  {
//  if(key_mode_A == ACTION_MOVE&&Key_A.mode_switch_flag == 0)
//  {
  //------------------------̧��-------------------------//
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W){
	    Key_A.time_up++;
	  	if(Key_A.time_up > 300)
	  	Key_A.time_up_ready = 1;}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S){
	    Key_A.time_up++;
	  	if(Key_A.time_up > 300)
	  	Key_A.time_up_ready = -1;}
	  else
	    {Key_A.time_up = 0;
  		Key_A.time_up_ready = 0;}
  //-------------------------------------------------//	
  	 	if(Key_A.time_up_ready  == 1)
  	{
  		Key_A.speed_up_time++;
  		if(Key_A.speed_up_time>1200)Key_A.speed_up_time=1200;
	  	Key_A.q=Key_A.speed_up_time*0.0003f;
	  	action_uplift.uplift_angleset[0] += Key_A.q;
     action_uplift.uplift_angleset[1] -= Key_A.q;}
	  else if(Key_A.time_up_ready  == -1)
	  {Key_A.speed_up_time++;
  		if(Key_A.speed_up_time>1200)Key_A.speed_up_time=1200;
  		Key_A.q=Key_A.speed_up_time*0.0003f;
  		action_uplift.uplift_angleset[0] -= Key_A.q;
     action_uplift.uplift_angleset[1] += Key_A.q;}
	  else 
	  {Key_A.speed_up_time=0;
	  	Key_A.q=0;
		  action_uplift.uplift_angleset[0] += 0;
      action_uplift.uplift_angleset[1] -= 0;}
	uplift_control();

	 
  //------------------------ǰ��-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)
  	  {Key_A.time_forward++;
  		if(Key_A.time_forward > 300)
	  	Key_A.time_forward_ready = 1;}
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
  	  {Key_A.time_forward++;
	  	if(Key_A.time_forward > 300)
	  	Key_A.time_forward_ready = -1;}
	  else
	    {Key_A.time_forward = 0;
	  	Key_A.time_forward_ready = 0;}
  //-------------------------------------------------//	
	  if(Key_A.time_forward_ready  == 1)
	  {action_forward.forward_speedset[0] = ACTION_VX_MAX/5;//
     action_forward.forward_speedset[1] = -ACTION_VX_MAX/5;}
	  else if(Key_A.time_forward_ready  == -1)
	  {action_forward.forward_speedset[0] = -ACTION_VX_MAX/5;//
     action_forward.forward_speedset[1] = ACTION_VX_MAX/5;}
    else {action_forward.forward_speedset[0] = 0;//
     action_forward.forward_speedset[1] = 0;}	 
		
		 forward_control();
	
	 
  //------------------------pitch��-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
  	  {Key_A.time_pitch++;
  		if(Key_A.time_pitch > 300)
  		Key_A.time_pitch_ready = 1;}
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
  	  {Key_A.time_pitch++;
  		if(Key_A.time_pitch > 300)
  		Key_A.time_pitch_ready = -1;}
  	else
  	  {Key_A.time_pitch = 0;
  		Key_A.time_pitch_ready = 0;}
  //-------------------------------------------------//	
  	if(Key_A.time_pitch_ready  == 1)
  	  {Key_A.speed_pitch_time++;
  		if(Key_A.speed_pitch_time>1600)Key_A.speed_pitch_time=1600;
  		Key_A.pitch_angle_cail=Key_A.speed_pitch_time*0.00015f;
	  	action_pitch.sucker_pitch_angleset += Key_A.pitch_angle_cail;}
  	else if(Key_A.time_pitch_ready  == -1)
	    {Key_A.speed_pitch_time++;
	  	if(Key_A.speed_pitch_time>1600)Key_A.speed_pitch_time=1600;
  		Key_A.pitch_angle_cail=Key_A.speed_pitch_time*0.00015f;
  		action_pitch.sucker_pitch_angleset -= Key_A.pitch_angle_cail;}
  	else 
  	  {Key_A.speed_pitch_time=0;
    	Key_A.pitch_angle_cail=0;
	    action_pitch.sucker_pitch_angleset += 0;}	 
	pitch_control();
	 
  //------------------------roll��-------------------------//
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
	    {Key_A.time_roll++;
	  	if(Key_A.time_roll > 100)
  		Key_A.time_roll_ready = 1;}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
  	  {Key_A.time_roll++;
	  	if(Key_A.time_roll > 100)
	  	Key_A.time_roll_ready = -1;}
	  else
	    {Key_A.time_roll = 0;
	  	Key_A.time_roll_ready = 0;}
  //-------------------------------------------------//	
	  if(Key_A.time_roll_ready  == 1)
	  {action_roll.sucker_roll_angleset += 0.005f;}
	  else if(Key_A.time_roll_ready  == -1)
	  {action_roll.sucker_roll_angleset -= 0.005f;}
	  else 
	  {action_roll.sucker_roll_angleset += 0.0f;}	 
	 
		roll_control();
		
		
  ////------------------------����-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
  	{
			Key_A.time_pump++;
  		if(Key_A.time_pump>100)
			{
				PUMP_OPEN;
			}
  	}	
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
  	{
			Key_A.time_pump++;
  		if(Key_A.time_pump>100)
			{
				PUMP_OFF;
			}
  	}	
    else Key_A.time_pump = 0;
	
	
//  		gimbal_move.gimbal_roll_speed_set = -(fp32)rc_ctrl.mouse.x * 0.01f;//TTT
  	if((fp32)rc_ctrl.mouse.x<-20)
		{
			gimbal_move.gimbal_roll_speed_set = 0.2f;
		}
  	else if((fp32)rc_ctrl.mouse.x>20)
		{
			gimbal_move.gimbal_roll_speed_set = -0.2f;
		}
		else gimbal_move.gimbal_roll_speed_set = 0.0f;
		
  	if(gimbal_move.gimbal_roll_speed_set>2.0f)
  	{
  		gimbal_move.gimbal_roll_speed_set = 2.0f;
  	}
  	if(gimbal_move.gimbal_roll_speed_set<-2.0f)
  	{
  		gimbal_move.gimbal_roll_speed_set = -2.0f;
  	}
	
//  	gimbal_roll_control();
 


		Key_A.qw2wr = 1;


  }
  //------------------------�²��ƶ�ģʽ-------------------------//
	else if(key_mode_A == ACTION_CHASSIS&&Key_A.init_switch_flag == 0)
	{
//  else if(key_mode_A == ACTION_CHASSIS&&Key_A.mode_switch_flag == 0)
//  {

		action_forward.forward_speedset[0] = 0;
		action_forward.forward_speedset[0] = 0;
		
		action_uplift.uplift_angleset[0] += 0;
		action_uplift.uplift_angleset[0] -= 0;
		
		action_pitch.sucker_pitch_angleset += 0;
	  action_roll.sucker_roll_angleset += 0;
		
		//һ����ͷ
	  gimbal_roll_control();
	  if(gimbal_move.gimbal_roll_limit_flag == 1)
	  {
	    if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
	    {
		  Key_A.qw2wr = 2;
		   if(gimbal_move.gimbal_roll_interval<160.0f)
		  	{
		  	  gimbal_move.gimbal_roll_speed_set = 1.0f;
		  		if(gimbal_move.gimbal_roll_interval>160.0f)
		  	  {
		  	  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  		}
		  	}
			  if(gimbal_move.gimbal_roll_interval>192.0f)
			  {
			    gimbal_move.gimbal_roll_speed_set = -1.0f;
			  	if(gimbal_move.gimbal_roll_interval<192.0f)
			    {
			    gimbal_move.gimbal_roll_speed_set = 0.0f;
			  	}
			  }
		   }
		  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
	   {
		  Key_A.qw2wr = 2;
		  	gimbal_move.gimbal_roll_speed_set = 1.0f;
		  	if(gimbal_move.gimbal_roll_interval>458.0f)
		  	{
		  		gimbal_move.gimbal_roll_speed_set = 0;
		  	}
		  }
	   else {gimbal_move.gimbal_roll_speed_set = 0;}
	   }
	 }
	else if(key_mode_A == ACTION_CATCH)
	{
		
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//
		{
//	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//��������
//		{
			if(action_uplift.uplift_angleset[0]<14.0f)
			{
				action_uplift.uplift_angleset[0] = 14.0f;
			}
			action_uplift.uplift_angleset[0]+=0.15f;
			action_uplift.uplift_angleset[1]-=0.15f;
			uplift_control();
//			if(action_uplift.uplift_interval[0]>22.0f)
//			{
				if(action_uplift.uplift_angleset[0]>75.0f)
				{
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 75.0f;
				Key_A.catch_time++;}
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
				if(action_uplift.uplift_angleset[1]<-60.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -60.0f;}	
				
//			}
			if(Key_A.catch_time>100)
			{
				 Key_A.speed_pitch_catch_time++;
			 	 Key_A.pitch_angle_catch_limit = action_pitch.pitch_limit_max-90.0f;
  		   if(Key_A.speed_pitch_catch_time>1400)
				 {
					 Key_A.speed_pitch_catch_time=1400;
  		     Key_A.pitch_angle_catch=Key_A.speed_pitch_catch_time*0.00015f;
	     	   action_pitch.sucker_pitch_angleset += Key_A.pitch_angle_catch;
					 
					 if(action_pitch.sucker_pitch_angleset>Key_A.pitch_angle_catch_limit)
					 {action_pitch.sucker_pitch_angleset = Key_A.pitch_angle_catch_limit;
						 Key_A.pitch_angle_catch_limit_ready = 1;}
					 
					 action_forward.forward_speedset[0] = -1.0f;
           action_forward.forward_speedset[1] = 1.0f;
					 forward_control();
					 

				 }
				 
			}

	 }
					if(Key_A.pitch_angle_catch_limit_ready == 1)
				 {
				 	 action_uplift.uplift_angleset[0]-=0.15f;
			     action_uplift.uplift_angleset[1]+=0.15f;
					 uplift_control();
					 if(action_uplift.limit_init_ready_u[0] == 1&&action_uplift.limit_init_ready_u[1] == 1)
					 {
						 Key_A.pitch_angle_catch_limit_ready = 0;
						 Key_A.speed_pitch_catch_time = 0;
						 Key_A.catch_time = 0;
					 }
				 }
					
		
		
	}
	else if(key_mode_A == ACTION_GOLD)
	{
		
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//��������
		{
			
			action_uplift.uplift_angleset[1] -= 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)//��������
		{
			
			action_uplift.uplift_angleset[0] += 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)//��������
		{
			
			action_uplift.uplift_angleset[1] += 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)//��������
		{
			
			action_uplift.uplift_angleset[0] -= 0.1f;
			action_uplift.uplift_angleset[1] -= 0.1f;
		}
		else 
		{
			action_uplift.uplift_angleset[0] += 0.0f;
			action_uplift.uplift_angleset[1] += 0.0f;
		}
				
	}
	else if(key_mode_A == ACTION_SILVER)
	{
				 int silver_limit_flag;
		int time_sliver;
		if(action_uplift.uplift_angleset[0]<14.0f)
		{
			action_uplift.uplift_angleset[0] = 14.0f;
			action_uplift.uplift_angleset[0] -= 0.0f;
		}		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//
		{
//		  time_sliver++;
//			if(time_sliver>200)
//			{
//	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//��������
//		{

			uplift_control();
//			if(action_uplift.uplift_interval[0]>22.0f)
//			{
			if(action_uplift.uplift_angleset[0]<465.0f)
			{
			  action_uplift.uplift_angleset[0]+=0.15f;
				if(action_uplift.uplift_angleset[0]>465.0f)
				{
					iqufhi = 1;
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 465.0f;
				}
      }
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
			if(action_uplift.uplift_angleset[1]>-451.0f)
			{
			  action_uplift.uplift_angleset[1]-=0.15f;
				if(action_uplift.uplift_angleset[1]<-451.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -451.0f;}	
			}
			
			
			if(action_uplift.uplift_angleset[0]>465.0f)
			{
			  action_uplift.uplift_angleset[0]-=0.15f;
				if(action_uplift.uplift_angleset[0]<465.0f)
				{
					iqufhi = 2;
					action_uplift.uplift_angleset[0] -= 0;
				action_uplift.uplift_angleset[0] = 465.0f;
				}
      }
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
			if(action_uplift.uplift_angleset[1]<-451.0f)
			{
			  action_uplift.uplift_angleset[1]+=0.15f;
				if(action_uplift.uplift_angleset[1]>-451.0f)
				{
				action_uplift.uplift_angleset[1] += 0;
				action_uplift.uplift_angleset[1] = -451.0f;}	
			}			
			
			if(iqufhi == 1)
			{
				Key_A.catch_time++;
			}
			else if(iqufhi == 2)
			{
				Key_A.catch_time++;
			}
			else Key_A.catch_time = 0;
//			}
			if(Key_A.catch_time>100)
			{
				 Key_A.speed_pitch_silver_time++;
			 	 Key_A.pitch_angle_silver_limit = action_pitch.pitch_limit_max-176.0f;
  		     Key_A.pitch_angle_silver=Key_A.speed_pitch_silver_time*0.00015f;
	     	   action_pitch.sucker_pitch_angleset -= Key_A.pitch_angle_silver;
  		   if(Key_A.speed_pitch_silver_time>1200)
				 {
					 Key_A.speed_pitch_silver_time=1200;		 
			      action_roll.sucker_roll_angleset = -6.8f;
//					 if(action_roll.sucker_roll_angleset<-6.8)
//					 action_forward.forward_speedset[0] = -1.0;
//           action_forward.forward_speedset[1] = 1.0;
//					 forward_control();
					 silver_limit_flag = 1;
				 }
				 
				 if(action_pitch.sucker_pitch_angleset<Key_A.pitch_angle_silver_limit&&silver_limit_flag == 1)
					 {action_pitch.sucker_pitch_angleset = Key_A.pitch_angle_silver_limit;
						 Key_A.pitch_angle_silver_limit_ready = 1;}
				 
			}
			else Key_A.speed_pitch_silver_time = 0;
//		 }

	 }
		else {Key_A.catch_time = 0;
	 iqufhi = 0;}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E&&Key_A.pitch_angle_silver_limit_ready == 1)
		{
			silver_limit_flag = 0;
			action_uplift.uplift_angleset[0]+=0.15f;
			action_uplift.uplift_angleset[1]-=0.15f;

				
			if(action_uplift.uplift_angleset[0]>660.0f)
				{
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 660.0f;
				Key_A.catch_time_over++;}
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
				if(action_uplift.uplift_angleset[1]<-646.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -646.0f;}	
				
//			}
			if(Key_A.catch_time_over>100)
			{
				if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R&&Key_A.init_switch_flag == 0)
	      {
	     	//R���л�����ʼ��ģʽ
	    	  Key_A.init_A++;
		      if(Key_A.init_A > 300)
		      {
		      Key_A.init_switch_flag = 1;
		      } 
	     }
	     else Key_A.init_A = 0;
       if(Key_A.init_switch_flag == 1)
       {
		     calibration();
         if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.limit_init_ready_p==1)
         {
		     	Key_A.init_switch_flag = 0;//�˳���ʼ��ģʽ��־λ
		     	Key_A.init_ready_flag = 1;
	    	  }
       }


		 }				 
			
			
			
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
		{
			if(action_roll.sucker_roll_angleset<-12.1f)
			{
				action_roll.sucker_roll_angleset += 0.005f;
				if(action_roll.sucker_roll_angleset>-12.1f)
				{
					action_roll.sucker_roll_angleset=-12.1f;
				}
			}
			else if(action_roll.sucker_roll_angleset>-12.1f)
			{
				action_roll.sucker_roll_angleset -= 0.005f;
				if(action_roll.sucker_roll_angleset<-12.1f)
				{
					action_roll.sucker_roll_angleset=-12.1f;
				}				
			}
			else action_roll.sucker_roll_angleset += 0.0f;
			
			
			
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
		{
			if(action_roll.sucker_roll_angleset<-6.8f)
			{
				action_roll.sucker_roll_angleset += 0.005f;
				if(action_roll.sucker_roll_angleset>-6.8f)
				{
					action_roll.sucker_roll_angleset=-6.8f;
				}
			}
			else if(action_roll.sucker_roll_angleset>-6.8f)
			{
				action_roll.sucker_roll_angleset -= 0.005f;
				if(action_roll.sucker_roll_angleset<-6.8f)
				{
					action_roll.sucker_roll_angleset=-6.8f;
				}				
			}
			else action_roll.sucker_roll_angleset += 0.0f;
			
			
			
		}
		else  Key_A.catch_time_over=0;
		
	}
	else key_mode_A = ACTION_BUG;
	
//	
	
	if((fp32)rc_ctrl.mouse.press_l==1)
	{
		
		PUMP_OPEN;
	}
	else if((fp32)rc_ctrl.mouse.press_r==1)
	{
		PUMP_OFF;
	}
		 
	
	
	
	
	

}

static void action_can_tx(void)
{
	//���̵��

	//צ��
	CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_HIGH,
							 (int16_t)action_forward.forward_give_current[0],
							 (int16_t)action_forward.forward_give_current[1],
//	   0,
//	0,
							 0,
							 0);

	//��ת	���
	CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_LOW,
//#if OVERTURN_POWER == 1
							 (int16_t)gimbal_move.gimbal_roll_current,
//	0,
							 (int16_t)action_uplift.uplift_give_current[0],
//#else
//							 0,
//							 0,
//#endif
//#if DEPOSIT_POWER == 1
							 (int16_t)action_pitch.sucker_pitch_give_current,
							 (int16_t)action_uplift.uplift_give_current[1]);
							 
//#else
//							 0,
//							 0);
//#endif
	CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_HIGH,
//#if OVERTURN_POWER == 1
							 (int16_t)action_roll.sucker_roll_give_current, 
							 
							 0,
							 0,
               0);
							 

}



