#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

#include "struct_typedef.h"
#include "bsp_wd.h"
#include "string.h"
//使用信号量
#include "FreeRTOS.h"
#include "semphr.h"

#define WEIGHT_RX_BUF_NUM 24u

#define	WEIGHT_FRAME_LENGTH 12u

// #define LARGE_BUFFER_SIZE 18  // 较大缓冲区的大小
// #define PACKAGE_HEADER_SIZE 3   // 包头长度
// #define PACKAGE_SIZE 12          // 数据包的总长度（例如：01 03 04 00 00 00 00 fa 33）

extern uint8_t *current_decode_buf; // 全局变量，记录当前要解码的缓冲区
extern SemaphoreHandle_t weight_decode_semaphore;
extern uint8_t weight_rx_buf[2][WEIGHT_RX_BUF_NUM];

/* ----------------------- Data Struct ------------------------------------- */

// //发送结构体
// typedef __packed struct
// {
//     uint8_t device_address;  // 设备地址
//     uint8_t function_code;   // 功能码
//     // uint8_t byte_count;      // 数据字节数
//     uint8_t data_high1;      // 高位数据寄存器
// 	uint8_t data_high2;      // 高位数据寄存器
//     uint8_t data_low1;       // 低位数据寄存器
// 	uint8_t data_low2;       // 低位数据寄存器
//     uint8_t crc1;            // CRC 校验码
// 	uint8_t crc2;            // CRC 校验码
  
// } WeightSend_t;


//结构体储存解码后重量数据，等待被调用
typedef __packed struct
{
	uint32_t weight;
} WeightData_t;

/* ----------------------- Internal Data ----------------------------------- */
/**
  * @brief       （可以）交换crc校验结果的高低字节
  */
extern uint16_t SwapBytes(uint16_t crc);
/**
  * @brief          计算MODBUS协议的CRC16校验码
  * @param[in]      data: 需要计算CRC的字节数据指针
  * @param[in]      length: 数据的长度（字节数）
  * @retval         uint16_t: 计算得到的CRC16校验码
  */
extern uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

// /**
//   * @brief          请求重量数据函数
//   * @param[in]      none
//   * @retval         none
//   */
// extern void weight_send_IRQ(void);

/**
  * @brief          重量接收初始化
  * @param[in]      none
  * @retval         none
  */
extern void weight_data_init(void);

/**
  * @brief          获取重量数据指针
  * @param[in]      none
  * @retval         重量数据指针
  */
extern const WeightData_t *get_weight_data_point(void);

/**
 * @brief          解析特定格式的MODBUS重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区，包含完整的MODBUS帧
 * @param[out]     weight_data: 用于存储解析后重量数据的结构体指针
 * @retval         none
 * @note           1. 函数假设重量数据以特定ASCII码格式编码
 *                 2. 仅提取数据帧中的重量部分进行解码
 */
extern void weight_data_IRQ(void);

/**
 * @brief          解析特定格式的重量数据
 * @param[in]      weight_rx_buf: 接收的原始数据缓冲区
 * @param[out]     weight_data: 解析后的重量数据结构体
 * @retval         解码是否成功
 */
extern void decode_weight_data(volatile const uint8_t *weight_rx_buf, WeightData_t *weight_data);

#endif
