/**   ****************************(C) ��Ȩ���� 2024 none****************************
 * @file       weight_data.c
 * @brief      �������������ݽ��պͽ���ģ��
 *             �������MODBUSЭ����������ݽ��գ�
 *             ʹ�ô���˫����DMA���и�Ч���ݽ���
 *             ������CRC16У��
 *
 * @details    ʵ�ֹ��ܣ�
 *             - ����������ͨ�ŵ�UART��DMA����
 *             - CRC16У��ͼ��㼰��֤
 *             - ��ASCII�����ʮ�����Ƹ�ʽ�н�����������
 *             - ˫�������ݽ�����������ܣ���ֹռ��CPU��ռ����������������У�
 *
 * @note       ���ݽ�����freertos�н��У���ֹռ��CPU��ռ����������������У����Ƿ�ֹ��ռ�ĵڶ����ֶΣ�
 *
 * @history    �汾        ����            ����           �޸�����
 *             V1.0.0     2024-11-27      BaiShuhao      ����������������޸ĵļ�¼
 *
 * @verbatim
 * ==============================================================================
 *  ͨ��Э�����飺
 *  - �豸��ַ: 0x01
 *  - ������: 0x03 
 *  - ���ݸ�ʽ: ��ת��ΪASCII�����ʮ����������ֵ 6���ֽ�
 * ==============================================================================
 * @endverbatim
 ****************************(C) ��Ȩ���� none****************************
 */
#include "weight_data.h"

#include "main.h"
#include "usart.h"
#include "stdlib.h"//����atof����

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/***************************************************��������**********************************************************/

uint8_t *current_decode_buf; // ȫ�ֱ�������¼��ǰҪ����Ļ�����
SemaphoreHandle_t weight_decode_semaphore;

// typedef enum {
//     WAIT_FOR_HEADER,  // �ȴ����հ�ͷ
//     RECEIVING_DATA    // ���ڽ�������
// } RxState;

// int a=0;
// int data_ready = 0; // ����׼����־��0 ��ʾ false��1 ��ʾ true
// ����ԭʼ���ݣ�˫����������
static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
/***************************************************�ṹ��***********************************************************/

// WeightData_t weight_data;

// �����������ݽṹ��
// WeightSend_t weight_send = {
//     .device_address = 0x01,  // �豸��ַ
//     .function_code = 0x03,   // ������
//     .data_high1 = 0x00,      // ��λ���ݼĴ���1
//     .data_high2 = 0x00,      // ��λ���ݼĴ���2
//     .data_low1 = 0x00,       // ��λ���ݼĴ���1
//     .data_low2 = 0x02,       // ��λ���ݼĴ���2
//     .crc1 = 0xC4,            // CRC У����1
//     .crc2 = 0x0B             // CRC У����2
// };

/***************************************************��������***********************************************************/

/**
 * @brief          ����crcУ�����ĸߵ��ֽ�
 * @param[in]      crc: ��Ҫ�����ߵ��ֽڵ�CRCֵ
 * @retval         uint16_t: �������CRCֵ
 */
uint16_t SwapBytes(uint16_t crc);

/**
 * @brief          ����MODBUSЭ���CRC16У����
 * @param[in]      data: ��Ҫ����CRC���ֽ�����ָ��
 * @param[in]      length: ���ݵĳ��ȣ��ֽ�����
 * @retval         uint16_t: ����õ���CRC16У����
 */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

/**
 * @brief          �������ճ�ʼ��
 * @param[in]      none
 * @retval         none
 */
void weight_data_init(void);

/**
 * @brief          ��ȡ��������ָ��
 * @param[in]      none
 * @retval         const WeightData_t*: ��������ָ��
 */
// const WeightData_t *get_weight_data_point(void);

/**
 * @brief          �����ض���ʽ��MODBUS��������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����������������MODBUS֡
//  * @param[out]     weight_data: ���ڴ洢�������������ݵĽṹ��ָ��
 * @retval         none
 */
void weight_data_IRQ(void);

/**
 * @brief          �����ض���ʽ����������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����
 * @param[out]     weight_data: ��������������ݽṹ��
 * @retval         none
 */
void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data);

/****************************************************��������******************************************************/

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
//    return SwapBytes(crc);  // �����ߵ��ֽں󣬷��ؽ��
	return crc;  // ֱ�ӷ��ؽ��
}



// /**
//   * @brief          ���������ͺ���
//   * @param[in]      none
//   * @retval         none
//   */
// void weight_send_IRQ(void)
// {
// 	if(a<5)
// 	{
// 		a++;
// 	}else if(a==5){
// 		HAL_UART_Transmit(&huart6,(uint8_t *)&weight_send,sizeof(weight_send),30);
// 		a=0;
// 	}
// }
	

/********************************************���ݽ���***********************************************************/


/**
  * @brief          �������ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void weight_data_init(void)
{
    /* ��ʼ��DMA˫������� */
	WD_init(weight_rx_buf[0],   // ��һ������
            weight_rx_buf[1],   // �ڶ�������
            WEIGHT_RX_BUF_NUM);

}

/**ֱ������ͷ�ļ�����
  * @brief          ��ȡ��������ָ��
  * @param[in]      none
  * @retval         ��������ָ��
  */
// const WeightData_t *get_weight_data_point(void)
// {
//     return &weight_data;
// }

/**
 * @brief          �����ض���ʽ��MODBUS��������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����������������MODBUS֡
//  * @param[out]     weight_data: ���ڴ洢�������������ݵĽṹ��ָ��
 * @retval         none
 * @note           1. �������������������ض�ASCII���ʽ����
 *                 2. ����ȡ����֡�е��������ֽ��н���
 *                 3. PE ��־����ͨ��һ���������������ȡ SR Ȼ���ȡ��д�� DR��*�� RXNE ������ʱ*
 *                 __HAL_UART_CLEAR_PEFLAG indeed is implemented as read SR, then read DR. No writing SR.
 */
void weight_data_IRQ(void)
{
    if (huart6.Instance->SR & UART_FLAG_IDLE) { //λ�����ȡ״̬�Ĵ����Ŀ����жϱ�־λ
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);   //__HAL_UART_CLEAR_PEFLAG ʵ����ִ����һ������������
                                            // �ȶ�ȡ SR��Ȼ���ȡ DR��������Ӳ�� IDLE �ж��Ѿ��������Ӷ������־λ��
                                            // ��ֹ�ظ������жϣ�IDLE ��־λ������ᵼ���жϲ��ϴ�������
                                            // Ϊ�´μ������֡����״̬����׼����

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //�жϵ�ǰ������
        {
            /* ��ǰʹ���ڴ滺����0 */
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // ��ȡ�������ݳ���
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR; //Number of Data to Transfer Register

            // �����趨���ݴ�������Ĵ������ݳ���
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // �л���������1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // ������յ�������/
            if (this_time_rx_len == WEIGHT_FRAME_LENGTH) {
                current_decode_buf      = weight_rx_buf[0];
                uint16_t received_crc   = (weight_rx_buf[0][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[0], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    xSemaphoreGiveFromISR(weight_decode_semaphore, NULL);//crcУ��Ҳ�����Ƶ�freertos�ע��Ҫ����һ��(��֮ǰò���Ѿ�����У�飬���Դ���ʲ��������)
                }
            }
        }
        else
        {
            /* ��ǰʹ���ڴ滺����1 */
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // ��ȡ�������ݳ���
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // �����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // �л���������0
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);

            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // ������յ�������
            if (this_time_rx_len == WEIGHT_FRAME_LENGTH) {
                current_decode_buf      = weight_rx_buf[1];
                uint16_t received_crc   = (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[1], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    xSemaphoreGiveFromISR(weight_decode_semaphore, NULL);//crcУ��Ҳ�����Ƶ�freertos�ע��Ҫ����һ��(��֮ǰò���Ѿ�����У�飬���Դ���ʲ��������)
                }
            }
        }
    }
}

/**
 * @brief          �����ض���ʽ����������
 * @param[in]      weight_rx_buf: ���յ�ԭʼ���ݻ�����
 * @param[out]     weight_data: ��������������ݽṹ��
 * @retval         none
 * @note           ����ASCII���ʾ�ĸ������ַ���������������ת��Ϊ������
 */
void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data)
{
    // ������Ч�Լ��
    if (weight_rx_buf == NULL || weight_data == NULL) {
        return;
    }

    // ȷ��������ȷ
    if (weight_rx_buf[0] != 0x01 || weight_rx_buf[1] != 0x03) {
        return;
    }

    // ������������
    // ��ʽΪ ASCII ���ʾ�ĸ������ַ���
    // (��ascll���ʾ���ַ���Ϊý�飬��ȡÿ��16���Ƶ��ֽڵĵڶ�λ->��Ϊ�ַ�����10���Ƶ���->ϵͳ����ת���ַ���ת��Ϊ������)
    float decoded_weight = 0.0f;
    char weight_str[7]   = {0};

    // ��ȡÿ���ֽڵĵڶ���ʮ����������
    weight_str[0] = weight_rx_buf[3] & 0x0F;
    weight_str[1] = weight_rx_buf[4] & 0x0F;
    weight_str[2] = weight_rx_buf[5] & 0x0F;
    weight_str[3] = '.';
    weight_str[4] = weight_rx_buf[7] & 0x0F;
    weight_str[5] = weight_rx_buf[8] & 0x0F;
    weight_str[6] = weight_rx_buf[9] & 0x0F;

    // ת��Ϊ�ɶ����ַ���
    for (int i = 0; i < 7; i++) {
        if (i != 3) {             // ����С����
            weight_str[i] += '0'; // ת��Ϊ ASCII �ַ�
        }
    }

    // ʹ�ñ�׼�⺯��ת��Ϊ������
    decoded_weight = atof(weight_str);

    // �洢����������
    weight_data->weight = decoded_weight;
}
