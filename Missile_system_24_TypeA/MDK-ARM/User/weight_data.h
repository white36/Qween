#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

//#include "struct_typedef.h"
#include "bsp_wd.h"


// #define WEIGHT_RX_BUF_NUM 18u

// #define	WEIGHT_FRAME_LENGTH 9u

#define LARGE_BUFFER_SIZE 1024  // 较大缓冲区的大小
#define PACKAGE_HEADER_SIZE 3   // 包头长度
#define PACKAGE_SIZE 9          // 数据包的总长度（例如：01 03 04 00 00 00 00 fa 33）


/* ----------------------- Data Struct ------------------------------------- */
//发送结构体
typedef __packed struct
{
    uint8_t device_address;  // 设备地址
    uint8_t function_code;   // 功能码
    // uint8_t byte_count;      // 数据字节数
    uint8_t data_high1;      // 高位数据寄存器
	uint8_t data_high2;      // 高位数据寄存器
    uint8_t data_low1;       // 低位数据寄存器
	uint8_t data_low2;       // 低位数据寄存器
    uint8_t crc1;            // CRC 校验码
	uint8_t crc2;            // CRC 校验码
  
} WeightSend_t;

//储存解码后重量数据，等待被调用
typedef __packed struct
{
			uint32_t weight;
} WeightData_t;

extern WeightSend_t weihgt_send;
extern WeightData_t weight_data;

/* ----------------------- Internal Data ----------------------------------- */
/**
  * @brief       交换crc校验结果的高低字节
  */
extern uint16_t SwapBytes(uint16_t crc);
/**
  * @brief          计算MODBUS协议的CRC16校验码
  * @param[in]      data: 需要计算CRC的字节数据指针
  * @param[in]      length: 数据的长度（字节数）
  * @retval         uint16_t: 计算得到的CRC16校验码
  */
extern uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);

/**
  * @brief          请求重量数据函数
  * @param[in]      none
  * @retval         none
  */
extern void weight_send_IRQ(void);

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
  * @brief          串口中断处理接收重量数据，需要注册进中断函数
  * @param[in]      none
  * @retval         none
  */
extern void weight_data_IRQ(void);

#endif
