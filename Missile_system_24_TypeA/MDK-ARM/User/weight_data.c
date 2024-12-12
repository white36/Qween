#include "weight_data.h"

#include "main.h"
#include "usart.h"
#include "string.h"

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
WeightData_t weight_data;
WeightSend_t weight_send = {
    .device_address = 0x01,  // �豸��ַ
    .function_code = 0x03,   // ������
    .data_high1 = 0x00,      // ��λ���ݼĴ���1
    .data_high2 = 0x00,      // ��λ���ݼĴ���2
    .data_low1 = 0x00,       // ��λ���ݼĴ���1
    .data_low2 = 0x02,       // ��λ���ݼĴ���2
    .crc1 = 0xC4,            // CRC У����1
    .crc2 = 0x0B             // CRC У����2
};

typedef enum {
    WAIT_FOR_HEADER,  // �ȴ����հ�ͷ
    RECEIVING_DATA    // ���ڽ�������
} RxState;

int a=0;
// ����ԭʼ���ݣ�Ϊ9���ֽڣ�����18���ֽڳ��ȣ���ֹDMA����Խ��
// static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
int data_ready = 0; // ����׼����־��0 ��ʾ false��1 ��ʾ true
// ����ԭʼ���ݣ�˫����������
static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
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
//    return SwapBytes(crc);  // �����ߵ��ֽں󣬷��ؽ��
	return crc;  // ֱ�ӷ��ؽ��
}

/**
  * @brief          ���������ͺ���
  * @param[in]      none
  * @retval         none
  */
void weight_send_IRQ(void)
{
	if(a<5)
	{
		a++;
	}else if(a==5){
		HAL_UART_Transmit(&huart6,(uint8_t *)&weight_send,sizeof(weight_send),30);
		a=0;
	}
}
	

/*******************************************************************************************************************/


/**
  * @brief          �������ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void weight_data_init(void)
{
    /* ��ʼ��DMA˫������� */
//    WD_Init(&huart6, 
//            weight_rx_buf[0],   // ��һ������
//            weight_rx_buf[1],   // �ڶ�������
//            WEIGHT_RX_BUF_NUM); // ��������С
	WD_init(weight_rx_buf[0],   // ��һ������
            weight_rx_buf[1],   // �ڶ�������
            WEIGHT_RX_BUF_NUM);

}

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
 * @brief UART6 DMA˫������մ�����
 * @param huart: UART���ָ��
 * @param Size: ���յ������ݴ�С
 * @details �ú�������DMA˫������л��������������ݵ���������
 */
//static void USER_USART6_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* ��鵱ǰDMA�������Ƿ�Ϊ��һ��������CTλΪRESET�� */
//    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
//        /* ��ʱ�ر�DMA���԰�ȫ�޸����� */
//        __HAL_DMA_DISABLE(huart->hdmarx);

//        /* ͨ������CTλ�л����ڶ������� */
//        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

//        /* ����DMA��������׼����һ�ν��� */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, WEIGHT_RX_BUF_NUM);

//        /* ������յ������ݴ�С����MODBUS֡���� */
//        if (Size == PACKAGE_SIZE) {
//            /* �����һ�������е��������� */
//            modbus_to_weight(weight_rx_buf[0], &weight_data);
//        }
//    }
//    /* �����ǰ�ǵڶ���������CTλΪSET�� */
//    else {
//        /* ��ʱ�ر�DMA���԰�ȫ�޸����� */
//        __HAL_DMA_DISABLE(huart->hdmarx);

//        /* ͨ�����CTλ�л��ص�һ������ */
//        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

//        /* ����DMA��������׼����һ�ν��� */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, WEIGHT_RX_BUF_NUM);

//        /* ������յ������ݴ�С����MODBUS֡���� */
//        if (Size == PACKAGE_SIZE) {
//            /* ����ڶ��������е��������� */
//            modbus_to_weight(weight_rx_buf[1], &weight_data);
//        }
//    }

//    /* ��������DMA����ʼ��һ�ν��� */
//    __HAL_DMA_ENABLE(huart->hdmarx);
//}


/**
 * @brief HAL UART�����¼��ص�����
 * @param huart: UART���ָ��
 * @param Size: ���յ������ݴ�С
 * @details ��UART�������ʱ�����ûص�����
 */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* ����¼��Ƿ�����UART6 */
//    if (huart == &huart6) {
//        /* ����UART6���� */
//        USER_USART6_RxHandler(huart, Size);
//    }
// }

/**
  * @brief          �����жϴ�������������ݣ���Ҫע����жϺ���
  * @param[in]      none
  * @retval         none
  * rxne�жϴ洢���ݵ�����������⵽�����жϴ������ݣ�
  */

// uint8_t large_buffer[LARGE_BUFFER_SIZE];
// uint16_t buffer_index = 0;

// void weight_data_IRQ(void)
// {
    
//     if (huart6.Instance->SR & UART_FLAG_RXNE) // ���յ�����
//     {
//         uint8_t received_data = huart6.Instance->DR;
        

// /*******************************************************  �����Զ���CPU������  **************************************************************/
//         // �����ݴ���󻺳���
//         if (buffer_index < LARGE_BUFFER_SIZE) {
//             large_buffer[buffer_index++] = received_data;
//         } else {
//             // �����������������ջ��������¿�ʼ
//             buffer_index = 0;
//         }

//         // �ڻ�������Ѱ�Ұ�ͷ
//         for (uint16_t i = 0; i <= buffer_index - PACKAGE_SIZE; i++) {
//             // ����Ƿ������Ч��ͷ
//             if (large_buffer[i] == 0x01 && 
//                 large_buffer[i + 1] == 0x03 && 
//                 large_buffer[i + 2] == 0x2B) 
//             {
//                 // �ҵ���ͷ��������������
//                 uint16_t received_crc = (large_buffer[i + PACKAGE_SIZE - 2] |
//                                          (large_buffer[i + PACKAGE_SIZE - 1] << 8));
//                 uint16_t calculated_crc = Modbus_CRC16(&large_buffer[i], PACKAGE_SIZE - 2);

//                 if (received_crc == calculated_crc) 
// 				  {
//                     // CRCУ��ͨ������������
//                     modbus_to_weight(&large_buffer[i], &weight_data);

//                     // �ƶ������������Ѵ�����������
//                     memmove(large_buffer, &large_buffer[i + PACKAGE_SIZE], 
//                             buffer_index - (i + PACKAGE_SIZE));
//                     buffer_index -= (i + PACKAGE_SIZE);
//                     break;
//                 }
//       
//}
//         }
//     }
// }

void weight_data_IRQ(void)
{
    if (huart6.Instance->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* ��ǰʹ���ڴ滺����0 */
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // ��ȡ�������ݳ���
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // �����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // �л���������1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // ������յ�������/
            if (this_time_rx_len == WEIGHT_FRAME_LENGTH) {
                uint16_t received_crc   = (weight_rx_buf[0][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[0], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    modbus_to_weight(weight_rx_buf[0], &weight_data);
                }
            }
        } else {
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
                uint16_t received_crc   = (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[1], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    modbus_to_weight(weight_rx_buf[1], &weight_data);
                }
            }
        }
    }
}

// ��HAL���ʼ�������У���Ҫ����DMA��˫������ģʽ
// ���磺hdma_usart6_rx.Init.Mode = DMA_DOUBLEBUFFER_MODE;

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
