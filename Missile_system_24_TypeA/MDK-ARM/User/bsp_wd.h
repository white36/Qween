#ifndef BSP_WD_H
#define BSP_WD_H
//#include "struct_typedef.h"
#include "main.h"

//extern void WD_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength);

extern void WD_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
