#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

#include "struct_typedef.h"
#include "bsp_wd.h"
#include "string.h"
//ʹ���ź���
#include "FreeRTOS.h"
#include "semphr.h"

#define WEIGHT_RX_BUF_NUM 24u

#define	WEIGHT_FRAME_LENGTH 12u

// #define LARGE_BUFFER_SIZE 18  // �ϴ󻺳����Ĵ�С
// #define PACKAGE_HEADER_SIZE 3   // ��ͷ����
// #define PACKAGE_SIZE 12          // ���ݰ����ܳ��ȣ����磺01 03 04 00 00 00 00 fa 33��

extern uint8_t *current_decode_buf; // ȫ�ֱ�������¼��ǰҪ����Ļ�����
extern SemaphoreHandle_t weight_decode_semaphore;
extern uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];

/* ----------------------- Data Struct ------------------------------------- */

// //���ͽṹ��
// typedef __packed struct
// {
//     uint8_t device_address;  // �豸��ַ
//     uint8_t function_code;   // ������
//     // uint8_t byte_count;      // �����ֽ���
//     uint8_t data_high1;      // ��λ���ݼĴ���
// 	uint8_t data_high2;      // ��λ���ݼĴ���
//     uint8_t data_low1;       // ��λ���ݼĴ���
// 	uint8_t data_low2;       // ��λ���ݼĴ���
//     uint8_t crc1;            // CRC У����
// 	uint8_t crc2;            // CRC У����
  
// } WeightSend_t;


//�ṹ�崢�������������ݣ��ȴ�������
typedef __packed struct
{
	uint32_t weight;
} WeightData_t;

/* ----------------------- Internal Data ----------------------------------- */
/**
  * @brief       �����ԣ�����crcУ�����ĸߵ��ֽ�
  */
extern uint16_t SwapBytes(uint16_t crc);
/**
  * @brief          ����MODBUSЭ���CRC16У����
  * @param[in]      data: ��Ҫ����CRC���ֽ�����ָ��
  * @param[in]      length: ���ݵĳ��ȣ��ֽ�����
  * @retval         uint16_t: ����õ���CRC16У����
  */
extern uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

// /**
//   * @brief          �����������ݺ���
//   * @param[in]      none
//   * @retval         none
//   */
// extern void weight_send_IRQ(void);

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
 * @brief          �����ض���ʽ��MODBUS��������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����������������MODBUS֡
 * @param[out]     weight_data: ���ڴ洢�������������ݵĽṹ��ָ��
 * @retval         none
 * @note           1. �������������������ض�ASCII���ʽ����
 *                 2. ����ȡ����֡�е��������ֽ��н���
 */
extern void weight_data_IRQ(void);

/**
 * @brief          �����ض���ʽ����������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����
 * @param[out]     weight_data: ��������������ݽṹ��
 * @retval         �����Ƿ�ɹ�
 */
extern void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data);

#endif
