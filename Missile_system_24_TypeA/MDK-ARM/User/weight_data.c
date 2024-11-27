#include "weight_data.h"

#include "main.h"
#include "usart.h"
#include "string.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
  * @brief          解析MODBUS协议的重量数据
  * @param[in]      modbus_buf: 接收的MODBUS原始数据指针
  * @param[out]     weight_data: 解析后的重量数据结构体
  * @retval         none
  */
static void modbus_to_weight(volatile const uint8_t *modbus_buf, WeightData_t *weight_data);

// 重量数据结构体
WeightData_t weight_data;
WeightSend_t weight_send = {
    .device_address = 0x01,  // 设备地址
    .function_code = 0x03,   // 功能码
    .data_high1 = 0x00,      // 高位数据寄存器1
    .data_high2 = 0x00,      // 高位数据寄存器2
    .data_low1 = 0x00,       // 低位数据寄存器1
    .data_low2 = 0x02,       // 低位数据寄存器2
    .crc1 = 0xC4,            // CRC 校验码1
    .crc2 = 0x0B             // CRC 校验码2
};

typedef enum {
    WAIT_FOR_HEADER,  // 等待接收包头
    RECEIVING_DATA    // 正在接收数据
} RxState;

int a=0;
// 接收原始数据，为9个字节，给了18个字节长度，防止DMA传输越界
// static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
int data_ready = 0; // 数据准备标志，0 表示 false，1 表示 true
// 接收原始数据，双缓冲区定义
static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
/*********************************************************************************************************************************/
/**
  * @brief       交换crc校验结果的高低字节
  */
uint16_t SwapBytes(uint16_t crc)
{
    return (crc >> 8) | (crc << 8);  // 交换高低字节
}

/**
  * @brief          计算MODBUS协议的CRC16校验码
  * @param[in]      data: 需要计算CRC的字节数据指针
  * @param[in]      length: 数据的长度（字节数）
  * @retval         uint16_t: 计算得到的CRC16校验码
  */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];  // 将每个字节与CRC寄存器进行异或
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;  // 如果最低位为1，执行右移并异或
            }
            else
            {
                crc >>= 1;  // 如果最低位为0，直接右移
            }
        }
    }
//    return SwapBytes(crc);  // 交换高低字节后，返回结果
	return crc;  // 直接返回结果
}

/**
  * @brief          重量请求发送函数
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
  * @brief          重量接收初始化
  * @param[in]      none
  * @retval         none
  */
void weight_data_init(void)
{
    /* 初始化DMA双缓冲接收 */
//    WD_Init(&huart6, 
//            weight_rx_buf[0],   // 第一缓冲区
//            weight_rx_buf[1],   // 第二缓冲区
//            WEIGHT_RX_BUF_NUM); // 缓冲区大小
	WD_init(weight_rx_buf[0],   // 第一缓冲区
            weight_rx_buf[1],   // 第二缓冲区
            WEIGHT_RX_BUF_NUM);

}

/**
  * @brief          获取重量数据指针
  * @param[in]      none
  * @retval         重量数据指针
  */
const WeightData_t *get_weight_data_point(void)
{
    return &weight_data;
}
/**
 * @brief UART6 DMA双缓冲接收处理函数
 * @param huart: UART句柄指针
 * @param Size: 接收到的数据大小
 * @details 该函数处理DMA双缓冲的切换，用于重量数据的连续接收
 */
//static void USER_USART6_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* 检查当前DMA缓冲区是否为第一缓冲区（CT位为RESET） */
//    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
//        /* 暂时关闭DMA，以安全修改配置 */
//        __HAL_DMA_DISABLE(huart->hdmarx);

//        /* 通过设置CT位切换到第二缓冲区 */
//        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

//        /* 重置DMA计数器，准备下一次接收 */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, WEIGHT_RX_BUF_NUM);

//        /* 如果接收到的数据大小符合MODBUS帧长度 */
//        if (Size == PACKAGE_SIZE) {
//            /* 处理第一缓冲区中的重量数据 */
//            modbus_to_weight(weight_rx_buf[0], &weight_data);
//        }
//    }
//    /* 如果当前是第二缓冲区（CT位为SET） */
//    else {
//        /* 暂时关闭DMA，以安全修改配置 */
//        __HAL_DMA_DISABLE(huart->hdmarx);

//        /* 通过清除CT位切换回第一缓冲区 */
//        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

//        /* 重置DMA计数器，准备下一次接收 */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, WEIGHT_RX_BUF_NUM);

//        /* 如果接收到的数据大小符合MODBUS帧长度 */
//        if (Size == PACKAGE_SIZE) {
//            /* 处理第二缓冲区中的重量数据 */
//            modbus_to_weight(weight_rx_buf[1], &weight_data);
//        }
//    }

//    /* 重新启用DMA，开始下一次接收 */
//    __HAL_DMA_ENABLE(huart->hdmarx);
//}


/**
 * @brief HAL UART接收事件回调函数
 * @param huart: UART句柄指针
 * @param Size: 接收到的数据大小
 * @details 当UART接收完成时触发该回调函数
 */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* 检查事件是否来自UART6 */
//    if (huart == &huart6) {
//        /* 处理UART6接收 */
//        USER_USART6_RxHandler(huart, Size);
//    }
// }

/**
  * @brief          串口中断处理接收重量数据，需要注册进中断函数
  * @param[in]      none
  * @retval         none
  * rxne中断存储数据到缓冲区，检测到空闲中断处理数据，
  */

// uint8_t large_buffer[LARGE_BUFFER_SIZE];
// uint16_t buffer_index = 0;

// void weight_data_IRQ(void)
// {
    
//     if (huart6.Instance->SR & UART_FLAG_RXNE) // 接收到数据
//     {
//         uint8_t received_data = huart6.Instance->DR;
        

// /*******************************************************  串口自定义CPU缓冲区  **************************************************************/
//         // 将数据存入大缓冲区
//         if (buffer_index < LARGE_BUFFER_SIZE) {
//             large_buffer[buffer_index++] = received_data;
//         } else {
//             // 如果缓冲区已满，清空缓冲区重新开始
//             buffer_index = 0;
//         }

//         // 在缓冲区中寻找包头
//         for (uint16_t i = 0; i <= buffer_index - PACKAGE_SIZE; i++) {
//             // 检查是否存在有效包头
//             if (large_buffer[i] == 0x01 && 
//                 large_buffer[i + 1] == 0x03 && 
//                 large_buffer[i + 2] == 0x2B) 
//             {
//                 // 找到包头，检查包的完整性
//                 uint16_t received_crc = (large_buffer[i + PACKAGE_SIZE - 2] |
//                                          (large_buffer[i + PACKAGE_SIZE - 1] << 8));
//                 uint16_t calculated_crc = Modbus_CRC16(&large_buffer[i], PACKAGE_SIZE - 2);

//                 if (received_crc == calculated_crc) 
// 				  {
//                     // CRC校验通过，处理数据
//                     modbus_to_weight(&large_buffer[i], &weight_data);

//                     // 移动缓冲区，将已处理的数据清除
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
            /* 当前使用内存缓冲区0 */
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // 获取接收数据长度
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // 切换到缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // 处理接收到的数据/
            if (this_time_rx_len == WEIGHT_FRAME_LENGTH) {
                uint16_t received_crc   = (weight_rx_buf[0][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[0], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    modbus_to_weight(weight_rx_buf[0], &weight_data);
                }
            }
        } else {
            /* 当前使用内存缓冲区1 */
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // 获取接收数据长度
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // 切换到缓冲区0
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);

            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // 处理接收到的数据
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

// 在HAL库初始化配置中，需要配置DMA的双缓冲区模式
// 例如：hdma_usart6_rx.Init.Mode = DMA_DOUBLEBUFFER_MODE;

/**
  * @brief          解析MODBUS协议的重量数据
  * @param[in]      modbus_buf: 接收的MODBUS原始数据指针
  * @param[out]     weight_data: 解析后的重量数据结构体
  * @retval         none
  */
static void modbus_to_weight(volatile const uint8_t *modbus_buf, WeightData_t *weight_data)
{
    if (modbus_buf == NULL || weight_data == NULL)
    {
        return;
    }

    // 根据MODBUS协议读取两个寄存器，合成32位重量数据
    weight_data->weight = (modbus_buf[3] << 24) | (modbus_buf[4] << 16) | (modbus_buf[5] << 8) | modbus_buf[6];

    // 将原始数据转换为浮点数（假设单位是千克，这部分根据具体协议单位调整）
//    weight_data->weight = (float)weight_raw / 1000.0f;  // 假设每寄存器值单位为克，转化为千克
}
