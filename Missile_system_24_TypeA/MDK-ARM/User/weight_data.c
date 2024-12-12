/**   ****************************(C) 版权所有 2024 none****************************
 * @file       weight_data.c
 * @brief      重量传感器数据接收和解码模块
 *             处理基于MODBUS协议的重量数据接收，
 *             使用串口双缓冲DMA进行高效数据接收
 *             并进行CRC16校验
 *
 * @details    实现功能：
 *             - 重量传感器通信的UART和DMA配置
 *             - CRC16校验和计算及验证
 *             - 从ASCII编码的十六进制格式中解码重量数据
 *             - 双缓冲数据接收以提高性能（防止占用CPU挤占其他任务的正常运行）
 *
 * @note       数据解码在freertos中进行，防止占用CPU挤占其他任务的正常运行（这是防止挤占的第二个手段）
 *
 * @history    版本        日期            作者           修改内容
 *             V1.0.0     2024-11-27      BaiShuhao      可利用这个做代码修改的记录
 *
 * @verbatim
 * ==============================================================================
 *  通信协议详情：
 *  - 设备地址: 0x01
 *  - 功能码: 0x03 
 *  - 数据格式: 可转换为ASCII编码的十六进制重量值 6个字节
 * ==============================================================================
 * @endverbatim
 ****************************(C) 版权所有 none****************************
 */
#include "weight_data.h"

#include "main.h"
#include "usart.h"
#include "stdlib.h"//包含atof函数

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/***************************************************变量定义**********************************************************/

uint8_t *current_decode_buf; // 全局变量，记录当前要解码的缓冲区
SemaphoreHandle_t weight_decode_semaphore;

// typedef enum {
//     WAIT_FOR_HEADER,  // 等待接收包头
//     RECEIVING_DATA    // 正在接收数据
// } RxState;

// int a=0;
// int data_ready = 0; // 数据准备标志，0 表示 false，1 表示 true
// 接收原始数据，双缓冲区定义
static uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];
/***************************************************结构体***********************************************************/

// WeightData_t weight_data;

// 重量请求数据结构体
// WeightSend_t weight_send = {
//     .device_address = 0x01,  // 设备地址
//     .function_code = 0x03,   // 功能码
//     .data_high1 = 0x00,      // 高位数据寄存器1
//     .data_high2 = 0x00,      // 高位数据寄存器2
//     .data_low1 = 0x00,       // 低位数据寄存器1
//     .data_low2 = 0x02,       // 低位数据寄存器2
//     .crc1 = 0xC4,            // CRC 校验码1
//     .crc2 = 0x0B             // CRC 校验码2
// };

/***************************************************函数声明***********************************************************/

/**
 * @brief          交换crc校验结果的高低字节
 * @param[in]      crc: 需要交换高低字节的CRC值
 * @retval         uint16_t: 交换后的CRC值
 */
uint16_t SwapBytes(uint16_t crc);

/**
 * @brief          计算MODBUS协议的CRC16校验码
 * @param[in]      data: 需要计算CRC的字节数据指针
 * @param[in]      length: 数据的长度（字节数）
 * @retval         uint16_t: 计算得到的CRC16校验码
 */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

/**
 * @brief          重量接收初始化
 * @param[in]      none
 * @retval         none
 */
void weight_data_init(void);

/**
 * @brief          获取重量数据指针
 * @param[in]      none
 * @retval         const WeightData_t*: 重量数据指针
 */
// const WeightData_t *get_weight_data_point(void);

/**
 * @brief          解析特定格式的MODBUS重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区，包含完整的MODBUS帧
//  * @param[out]     weight_data: 用于存储解析后重量数据的结构体指针
 * @retval         none
 */
void weight_data_IRQ(void);

/**
 * @brief          解析特定格式的重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区
 * @param[out]     weight_data: 解析后的重量数据结构体
 * @retval         none
 */
void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data);

/****************************************************函数内容******************************************************/

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



// /**
//   * @brief          重量请求发送函数
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
	

/********************************************数据接收***********************************************************/


/**
  * @brief          重量接收初始化
  * @param[in]      none
  * @retval         none
  */
void weight_data_init(void)
{
    /* 初始化DMA双缓冲接收 */
	WD_init(weight_rx_buf[0],   // 第一缓冲区
            weight_rx_buf[1],   // 第二缓冲区
            WEIGHT_RX_BUF_NUM);

}

/**直接引用头文件就行
  * @brief          获取重量数据指针
  * @param[in]      none
  * @retval         重量数据指针
  */
// const WeightData_t *get_weight_data_point(void)
// {
//     return &weight_data;
// }

/**
 * @brief          解析特定格式的MODBUS重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区，包含完整的MODBUS帧
//  * @param[out]     weight_data: 用于存储解析后重量数据的结构体指针
 * @retval         none
 * @note           1. 函数假设重量数据以特定ASCII码格式编码
 *                 2. 仅提取数据帧中的重量部分进行解码
 *                 3. PE 标志可以通过一个序列来清除：读取 SR 然后读取或写入 DR，*当 RXNE 被设置时*
 *                 __HAL_UART_CLEAR_PEFLAG indeed is implemented as read SR, then read DR. No writing SR.
 */
void weight_data_IRQ(void)
{
    if (huart6.Instance->SR & UART_FLAG_IDLE) { //位运算读取状态寄存器的空闲中断标志位
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);   //__HAL_UART_CLEAR_PEFLAG 实际上执行了一个“读操作”
                                            // 先读取 SR，然后读取 DR，这会告诉硬件 IDLE 中断已经被处理，从而清除标志位。
                                            // 防止重复触发中断（IDLE 标志位不清除会导致中断不断触发）。
                                            // 为下次检测数据帧空闲状态做好准备。

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //判断当前缓冲区
        {
            /* 当前使用内存缓冲区0 */
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // 获取接收数据长度
            this_time_rx_len = WEIGHT_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR; //Number of Data to Transfer Register

            // 重新设定数据传输计数寄存器数据长度
            hdma_usart6_rx.Instance->NDTR = WEIGHT_RX_BUF_NUM;

            // 切换到缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            // 处理接收到的数据/
            if (this_time_rx_len == WEIGHT_FRAME_LENGTH) {
                current_decode_buf      = weight_rx_buf[0];
                uint16_t received_crc   = (weight_rx_buf[0][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[0], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    xSemaphoreGiveFromISR(weight_decode_semaphore, NULL);//crc校验也可以移到freertos里、注意要调试一次(但之前貌似已经过了校验，所以大概率不会出问题)
                }
            }
        }
        else
        {
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
                current_decode_buf      = weight_rx_buf[1];
                uint16_t received_crc   = (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 2] |
                                         (weight_rx_buf[1][WEIGHT_FRAME_LENGTH - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(weight_rx_buf[1], WEIGHT_FRAME_LENGTH - 2);

                if (received_crc == calculated_crc) {
                    xSemaphoreGiveFromISR(weight_decode_semaphore, NULL);//crc校验也可以移到freertos里、注意要调试一次(但之前貌似已经过了校验，所以大概率不会出问题)
                }
            }
        }
    }
}

/**
 * @brief          解析特定格式的重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区
 * @param[out]     weight_data: 解析后的重量数据结构体
 * @retval         none
 * @note           借助ASCII码表示的浮点数字符串，将重量数据转换为浮点数
 */
void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data)
{
    // 参数有效性检查
    if (weight_rx_buf == NULL || weight_data == NULL) {
        return;
    }

    // 确保数据正确
    if (weight_rx_buf[0] != 0x01 || weight_rx_buf[1] != 0x03) {
        return;
    }

    // 解码重量数据
    // 格式为 ASCII 码表示的浮点数字符串
    // (以ascll码表示的字符串为媒介，提取每个16进制的字节的第二位->变为字符串中10进制的数->系统函数转将字符串转换为浮点数)
    float decoded_weight = 0.0f;
    char weight_str[7]   = {0};

    // 提取每个字节的第二个十六进制数字
    weight_str[0] = weight_rx_buf[3] & 0x0F;
    weight_str[1] = weight_rx_buf[4] & 0x0F;
    weight_str[2] = weight_rx_buf[5] & 0x0F;
    weight_str[3] = '.';
    weight_str[4] = weight_rx_buf[7] & 0x0F;
    weight_str[5] = weight_rx_buf[8] & 0x0F;
    weight_str[6] = weight_rx_buf[9] & 0x0F;

    // 转换为可读的字符串
    for (int i = 0; i < 7; i++) {
        if (i != 3) {             // 跳过小数点
            weight_str[i] += '0'; // 转换为 ASCII 字符
        }
    }

    // 使用标准库函数转换为浮点数
    decoded_weight = atof(weight_str);

    // 存储解码后的重量
    weight_data->weight = decoded_weight;
}
