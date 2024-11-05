#include "weight_data.h"
#include "main.h"

#include <string.h>


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
WeightSend_t weight_send;
WeightData_t weight_data;
typedef enum {
    WAIT_FOR_HEADER,  // 等待接收包头
    RECEIVING_DATA    // 正在接收数据
} RxState;

int a=0;
// 接收原始数据，为9个字节，给了18个字节长度，防止DMA传输越界
// static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
int data_ready = 0; // 数据准备标志，0 表示 false，1 表示 true

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
    return SwapBytes(crc);  // 交换高低字节后，返回结果
}

/**
  * @brief          重量请求发送函数
  * @param[in]      none
  * @retval         none
  */
void weight_send_IRQ(void)
{
		weight_send.device_address = 0x01;  // 设备地址
    weight_send.function_code = 0x03;   // 功能码
    // weight_send.byte_count = 0x04;      // 数据字节数
    weight_send.data_high1 = 0x00;     // 高位寄存器数据
	  weight_send.data_high2 = 0x00;     // 高位寄存器数据
    weight_send.data_low1 = 0x00;      // 低位寄存器数据
	  weight_send.data_low2 = 0x02;      // 低位寄存器数据
		weight_send.crc1 = 0xC4;
	  weight_send.crc2 = 0x0B;
   // weight_send.a++;
//	//临时数组计算crc
//		uint8_t data_to_crc[] = {
//        weight_send.device_address,
//        weight_send.function_code,
//        // weight_send.byte_count,
//        (weight_send.data_high >> 8) & 0xFF,  // 高字节
//        weight_send.data_high & 0xFF,         // 低字节
//        (weight_send.data_low >> 8) & 0xFF,   // 高字节
//        weight_send.data_low & 0xFF           // 低字节
//    };
//	
//		Modbus_CRC16(data_to_crc,sizeof(data_to_crc));
//		weight_send.crc = Modbus_CRC16(data_to_crc, sizeof(data_to_crc));
//		
		//串口发送
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
  * @brief          重量接收初始化
  * @param[in]      none
  * @retval         none
  */
// void weight_data_init(void)
// {
//     WD_init(weight_rx_buf[0], weight_rx_buf[1], WEIGHT_RX_BUF_NUM);
// }

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
  * @brief          串口中断处理接收重量数据，需要注册进中断函数
  * @param[in]      none
  * @retval         none
  * rxne中断存储数据到缓冲区，检测到空闲中断处理数据，
  */

uint8_t large_buffer[LARGE_BUFFER_SIZE];
uint16_t buffer_index = 0;

void weight_data_IRQ(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        uint8_t received_data = huart6.Instance->DR;

        // 将数据存入大缓冲区
        if (buffer_index < LARGE_BUFFER_SIZE) {
            large_buffer[buffer_index++] = received_data;
        } else {
            // 如果缓冲区已满，清空缓冲区重新开始
            buffer_index = 0;
        }

        // 在缓冲区中寻找包头
        for (uint16_t i = 0; i <= buffer_index - PACKAGE_SIZE; i++) {
            // 检查是否存在有效包头
            if (large_buffer[i] == 0x01 && 
                large_buffer[i + 1] == 0x03 && 
                large_buffer[i + 2] == 0x04) 
            {
                // 找到包头，检查包的完整性
                uint16_t received_crc = (large_buffer[i + PACKAGE_SIZE - 2] |
                                         (large_buffer[i + PACKAGE_SIZE - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(&large_buffer[i], PACKAGE_SIZE - 2);

                if (received_crc == calculated_crc) {
                    // CRC校验通过，处理数据
                    modbus_to_weight(&large_buffer[i], &weight_data);

                    // 移动缓冲区，将已处理的数据清除
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
