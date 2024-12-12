#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

//#include "struct_typedef.h"
#include "bsp_wd.h"


// #define WEIGHT_RX_BUF_NUM 18u

// #define	WEIGHT_FRAME_LENGTH 9u

#define LARGE_BUFFER_SIZE 1024  // �ϴ󻺳����Ĵ�С
#define PACKAGE_HEADER_SIZE 3   // ��ͷ����
#define PACKAGE_SIZE 9          // ���ݰ����ܳ��ȣ����磺01 03 04 00 00 00 00 fa 33��


/* ----------------------- Data Struct ------------------------------------- */
//���ͽṹ��
typedef __packed struct
{
    uint8_t device_address;  // �豸��ַ
    uint8_t function_code;   // ������
    // uint8_t byte_count;      // �����ֽ���
    uint8_t data_high1;      // ��λ���ݼĴ���
	uint8_t data_high2;      // ��λ���ݼĴ���
    uint8_t data_low1;       // ��λ���ݼĴ���
	uint8_t data_low2;       // ��λ���ݼĴ���
    uint8_t crc1;            // CRC У����
	uint8_t crc2;            // CRC У����
  
} WeightSend_t;

//���������������ݣ��ȴ�������
typedef __packed struct
{
			uint32_t weight;
} WeightData_t;

extern WeightSend_t weihgt_send;
extern WeightData_t weight_data;

/* ----------------------- Internal Data ----------------------------------- */
/**
  * @brief       ����crcУ�����ĸߵ��ֽ�
  */
extern uint16_t SwapBytes(uint16_t crc);
/**
  * @brief          ����MODBUSЭ���CRC16У����
  * @param[in]      data: ��Ҫ����CRC���ֽ�����ָ��
  * @param[in]      length: ���ݵĳ��ȣ��ֽ�����
  * @retval         uint16_t: ����õ���CRC16У����
  */
extern uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

/**
  * @brief          �����������ݺ���
  * @param[in]      none
  * @retval         none
  */
extern void weight_send_IRQ(void);

/**
  * @brief          �������ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
extern void weight_data_init(void);

/**
  * @brief          ��ȡ��������ָ��
  * @param[in]      none
  * @retval         ��������ָ��
  */
extern const WeightData_t *get_weight_data_point(void);

/**
  * @brief          �����жϴ�������������ݣ���Ҫע����жϺ���
  * @param[in]      none
  * @retval         none
  */
extern void weight_data_IRQ(void);

#endif
