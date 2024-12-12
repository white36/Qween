#include "weight_data.h"
#include "main.h"

#include <string.h>


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
  * @brief          ����MODBUSЭ�����������
  * @param[in]      modbus_buf: ���յ�MODBUSԭʼ����ָ��
  * @param[out]     weight_data: ��������������ݽṹ��
  * @retval         none
  */
static void modbus_to_weight(volatile const uint8_t *modbus_buf, WeightData_t *weight_data);

// �������ݽṹ��
WeightSend_t weight_send;
WeightData_t weight_data;
typedef enum {
    WAIT_FOR_HEADER,  // �ȴ����հ�ͷ
    RECEIVING_DATA    // ���ڽ�������
} RxState;

int a=0;
// ����ԭʼ���ݣ�Ϊ9���ֽڣ�����18���ֽڳ��ȣ���ֹDMA����Խ��
// static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
int data_ready = 0; // ����׼����־��0 ��ʾ false��1 ��ʾ true

/*********************************************************************************************************************************/
/**
  * @brief       ����crcУ�����ĸߵ��ֽ�
  */
uint16_t SwapBytes(uint16_t crc)
{
    return (crc >> 8) | (crc << 8);  // �����ߵ��ֽ�
}

/**
  * @brief          ����MODBUSЭ���CRC16У����
  * @param[in]      data: ��Ҫ����CRC���ֽ�����ָ��
  * @param[in]      length: ���ݵĳ��ȣ��ֽ�����
  * @retval         uint16_t: ����õ���CRC16У����
  */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];  // ��ÿ���ֽ���CRC�Ĵ����������
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;  // ������λΪ1��ִ�����Ʋ����
            }
            else
            {
                crc >>= 1;  // ������λΪ0��ֱ������
            }
        }
    }
    return SwapBytes(crc);  // �����ߵ��ֽں󣬷��ؽ��
}

/**
  * @brief          ���������ͺ���
  * @param[in]      none
  * @retval         none
  */
void weight_send_IRQ(void)
{
		weight_send.device_address = 0x01;  // �豸��ַ
    weight_send.function_code = 0x03;   // ������
    // weight_send.byte_count = 0x04;      // �����ֽ���
    weight_send.data_high1 = 0x00;     // ��λ�Ĵ�������
	  weight_send.data_high2 = 0x00;     // ��λ�Ĵ�������
    weight_send.data_low1 = 0x00;      // ��λ�Ĵ�������
	  weight_send.data_low2 = 0x02;      // ��λ�Ĵ�������
		weight_send.crc1 = 0xC4;
	  weight_send.crc2 = 0x0B;
   // weight_send.a++;
//	//��ʱ�������crc
//		uint8_t data_to_crc[] = {
//        weight_send.device_address,
//        weight_send.function_code,
//        // weight_send.byte_count,
//        (weight_send.data_high >> 8) & 0xFF,  // ���ֽ�
//        weight_send.data_high & 0xFF,         // ���ֽ�
//        (weight_send.data_low >> 8) & 0xFF,   // ���ֽ�
//        weight_send.data_low & 0xFF           // ���ֽ�
//    };
//	
//		Modbus_CRC16(data_to_crc,sizeof(data_to_crc));
//		weight_send.crc = Modbus_CRC16(data_to_crc, sizeof(data_to_crc));
//		
		//���ڷ���
//		HAL_UART_Transmit_IT(&huart6,(uint8_t *)&weight_send,sizeof(weight_send));
		if(a<5)
		{
			a++;
		}else if(a==5)
		{
		HAL_UART_Transmit_IT(&huart6,(uint8_t *)&weight_send,sizeof(weight_send));
			a=1;
		}
}
	

/*******************************************************************************************************************/
/**
  * @brief          �������ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
// void weight_data_init(void)
// {
//     WD_init(weight_rx_buf[0], weight_rx_buf[1], WEIGHT_RX_BUF_NUM);
// }

/**
  * @brief          ��ȡ��������ָ��
  * @param[in]      none
  * @retval         ��������ָ��
  */
const WeightData_t *get_weight_data_point(void)
{
    return &weight_data;
}

/**
  * @brief          �����жϴ�������������ݣ���Ҫע����жϺ���
  * @param[in]      none
  * @retval         none
  * rxne�жϴ洢���ݵ�����������⵽�����жϴ������ݣ�
  */

uint8_t large_buffer[LARGE_BUFFER_SIZE];
uint16_t buffer_index = 0;

void weight_data_IRQ(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE) // ���յ�����
    {
        uint8_t received_data = huart6.Instance->DR;

        // �����ݴ���󻺳���
        if (buffer_index < LARGE_BUFFER_SIZE) {
            large_buffer[buffer_index++] = received_data;
        } else {
            // �����������������ջ��������¿�ʼ
            buffer_index = 0;
        }

        // �ڻ�������Ѱ�Ұ�ͷ
        for (uint16_t i = 0; i <= buffer_index - PACKAGE_SIZE; i++) {
            // ����Ƿ������Ч��ͷ
            if (large_buffer[i] == 0x01 && 
                large_buffer[i + 1] == 0x03 && 
                large_buffer[i + 2] == 0x04) 
            {
                // �ҵ���ͷ��������������
                uint16_t received_crc = (large_buffer[i + PACKAGE_SIZE - 2] |
                                         (large_buffer[i + PACKAGE_SIZE - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(&large_buffer[i], PACKAGE_SIZE - 2);

                if (received_crc == calculated_crc) {
                    // CRCУ��ͨ������������
                    modbus_to_weight(&large_buffer[i], &weight_data);

                    // �ƶ������������Ѵ�����������
                    memmove(large_buffer, &large_buffer[i + PACKAGE_SIZE], 
                            buffer_index - (i + PACKAGE_SIZE));
                    buffer_index -= (i + PACKAGE_SIZE);
                    break;
                }
            }
        }
    }
}



/**
  * @brief          ����MODBUSЭ�����������
  * @param[in]      modbus_buf: ���յ�MODBUSԭʼ����ָ��
  * @param[out]     weight_data: ��������������ݽṹ��
  * @retval         none
  */
static void modbus_to_weight(volatile const uint8_t *modbus_buf, WeightData_t *weight_data)
{
    if (modbus_buf == NULL || weight_data == NULL)
    {
        return;
    }

    // ����MODBUSЭ���ȡ�����Ĵ������ϳ�32λ��������
    weight_data->weight = (modbus_buf[3] << 24) | (modbus_buf[4] << 16) | (modbus_buf[5] << 8) | modbus_buf[6];

    // ��ԭʼ����ת��Ϊ�����������赥λ��ǧ�ˣ��ⲿ�ָ��ݾ���Э�鵥λ������
//    weight_data->weight = (float)weight_raw / 1000.0f;  // ����ÿ�Ĵ���ֵ��λΪ�ˣ�ת��Ϊǧ��
}
