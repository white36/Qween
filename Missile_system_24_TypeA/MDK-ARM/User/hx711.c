#include "hx711.h"
#include "stm32f4xx.h"                  // Device header
#include "main.h"

 
#define     HX711_DOUT(x)          HAL_GPIO_WritePin(HX711_DOUT_GPIO_Port, HX711_DOUT_GPIO_PIN, x)
#define     HX711_SCK(x)           HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_GPIO_PIN, x)
#define     HX711_SCK_STATE     	 HAL_GPIO_ReadPin(HX711_SCK_GPIO_Port, HX711_SCK_GPIO_PIN)
#define     HX711_DOUT_STATE    	 HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_GPIO_PIN)
 
static STRUCT_HX711_TYPEDEF hx711;
float Pulling_force;

 
/* �������ṹ�������ʼ�� */
static void hx711_struct_config(void)
{
	hx711.gain = GAIN_128;	/* ��ǰ���� 128 */
    hx711.isTare = true;	/* Ĭ�ϵ�һ�γ���ΪƤ�� */
    hx711.k = 0.004757f;	/* ������ת��ϵ�� */
    hx711.b = 0.f;			/* ���������У��/����ֵ */
    hx711.tare = 0.f;		/* ����Ƥ������ */
    hx711.actual = 0.f;		/* ���澻������ */
}
 
/* ��ȡ���������ص�ADC����ֵ���������� �ٶ�Ϊһ�����Թ�ϵ y=kx+b */
static unsigned long hx711_readCount(void)
{
    unsigned long count = 0;		/* ��Ҫ��ȡ25-27��0/1���� */
    unsigned char timeOut_cnt = 0; 	/* ��ⳬʱ����ֵ */
    
    HX711_SCK(0);
	/* ��ӳ�ʱ��� ��ֹһֱδʶ�𵽴��������³����������� */
    while(HX711_DOUT_STATE) { /* �ȴ�DOUT�Ӹߵ�ƽ���͵�ƽ���� */
        timeOut_cnt ++;
        HAL_Delay(1);
        if(timeOut_cnt > 1) return 0;
    }
    /* ��ȡ24λ��ADC���������� */
    for(unsigned char i=0; i<24; i++) {
        HX711_SCK(1);
        HAL_Delay(1);
        HX711_SCK(0);
        count |= (HX711_DOUT_STATE) << (24-i); /* �����ض�ȡ���� */
        HAL_Delay(1);
    }
    /* �������������������ͨ�� */
    for(unsigned char j=0; j<hx711.gain; j++) {
        HX711_SCK(1);
        HAL_Delay(1);
        HX711_SCK(0);
        HAL_Delay(1);
    }
    return (count ^ 0x800000);
}
 
/* ----------------------------- �ӿں��� ---------------------- */
/* ������ ��ʼ�� */
void hx711_init(void)
{
    hx711_struct_config();
}
 
/* �������ϵ� */
void hx711_power_off(void)
{
    HX711_SCK(0);
    HAL_Delay(1);
    HX711_SCK(1);
    HAL_Delay(100);
}
 
/* �������ϵ� */
void hx711_reset(void)
{
    if(HX711_SCK_STATE == 1) {
        HX711_SCK(0);
    }
}
 
/* ���ô�����ת��ϵ�� */
void hx711_set_convert_ratio(float ratio)
{
    hx711.k = ratio;
}
 
/* ���ô��������У��/����ֵ */
void hx711_set_offset_value(float offset)
{
    hx711.b = offset;
}
 
/* ���ô��������� */
void hx711_set_gain(ENUM_HX711_GAIN_TYPEDEF gain)
{
    hx711.gain = gain;
}
 
/* ��ȡ������Ƥ������ */
float hx711_get_tare_weight(void)
{
    hx711.isTare = true; /* ��ȡƤ�ص�ʱ�� ���ñ�־λ��λ */
    if(hx711.isTare == true) {
        hx711.isTare = false;
        hx711.tare = (float)hx711_readCount() * hx711.k + hx711.b; /* ��ʱ��ȡ��������ΪƤ�� */
    }
    return (hx711.tare);
}
 
float hx711_get_actual_weight(void)
{
    float weight = 0.f;
    
    if(hx711.isTare == false) { /* ���ñ�־λ��λ �����ʱӦ���㾻�� */
        weight = (float)hx711_readCount() * hx711.k + hx711.b;
        hx711.actual = weight - hx711.tare;
    }
    return (hx711.actual);
}

void hx711_get_weight(void)
{
	hx711_get_tare_weight();
	hx711_get_actual_weight();
	Pulling_force = hx711.actual;
}
