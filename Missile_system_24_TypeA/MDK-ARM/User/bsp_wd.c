#include "bsp_wd.h"
#include "main.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
/**
  * @brief          初始化UART DMA双缓冲接收
  * @param[in]      huart: UART句柄指针
  * @param[in]      DstAddress: 第一缓冲区地址
  * @param[in]      SecondMemAddress: 第二缓冲区地址
  * @param[in]      DataLength: 数据长度
  * @retval         none
  */
//void WD_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
//{
//    /* 配置UART接收模式为IDLE空闲中断模式 */
//    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
//	
//    huart->RxXferSize = DataLength;
//    
//    /* 使能UART的DMA接收和IDLE中断 */
//    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
//    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

//    /* 启动DMA双缓冲接收 */
//    HAL_DMAEx_MultiBufferStart(huart->hdmarx, 
//                              (uint32_t)&huart->Instance->DR,  // 使用DR寄存器
//                              (uint32_t)DstAddress, 
//                              (uint32_t)SecondMemAddress, 
//                              DataLength);
//}


void WD_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能 DMA 串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //失能奇偶校验（可能多此一举）
    CLEAR_BIT(huart6.Instance->CR1, USART_CR1_PCE);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // 失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    // 设置 DMA 外设地址：USART6 的数据寄存器地址
    hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
    
    // 设置内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    
    // 设置内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    
    // 设置要接收的数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    
    // 使能双缓冲区模式
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    
    // 使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}
