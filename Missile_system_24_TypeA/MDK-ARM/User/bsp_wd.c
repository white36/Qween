#include "bsp_wd.h"
#include "main.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

void WD_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // ʹ�� DMA ���ڽ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //ʧ����żУ�飨���ܶ��һ�٣�
    CLEAR_BIT(huart6.Instance->CR1, USART_CR1_PCE);

    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // ʧЧ DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    // ���� DMA �����ַ��USART6 �����ݼĴ�����ַ
    hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
    
    // �����ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    
    // �����ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    
    // ����Ҫ���յ����ݳ���
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    
    // ʹ��˫������ģʽ
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    
    // ʹ�� DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}