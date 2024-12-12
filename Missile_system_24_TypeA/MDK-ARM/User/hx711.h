#ifndef __HX711_H
#define __HX711_H
 
#include "stm32f4xx.h"                  // Device header

#include "stdbool.h"
 
/* ���ڱ���HX711������ */
typedef enum {
    GAIN_128 = 1,
    GAIN_32,
    GAIN_64,
} ENUM_HX711_GAIN_TYPEDEF;
 
/* �ṹ�� ����ʵ��HX711��������� */
typedef struct {
    ENUM_HX711_GAIN_TYPEDEF gain; /* ���� */
    bool isTare;    /* �жϴ����������Ƿ���Ƥ�� true-�� false-�� */
    float k;        /* ����ϵ�� */
    float b;        /* ����ֵ */
    float tare;     /* Ƥ�� */
    float actual;   /* ʵ�� */
} STRUCT_HX711_TYPEDEF;

 
/* ---------------- �����嵥 --------------- */
void        hx711_init(void);       				/* HX711��ʼ�� */
void        hx711_power_off(void);   				/* HX711�ϵ� */
void        hx711_reset(void);      				/* HX711�ϵ��λ */
void        hx711_set_convert_ratio(float ratio); 	/* ����HX711��������ϵ�� */
void        hx711_set_offset_value(float offset); 	/* ����HX711��������ֵ */
void        hx711_set_gain(ENUM_HX711_GAIN_TYPEDEF gain); /* ����HX711����ϵ�� */
float       hx711_get_tare_weight(void); 			/* ��ȡƤ�� */
float       hx711_get_actual_weight(void); 			/* ��ȡʵ�� */
void 				hx711_get_weight(void);
 
#endif
