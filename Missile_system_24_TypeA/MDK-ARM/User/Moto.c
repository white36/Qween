
#include "Moto.h"
 
 
Moto_DataTypedef Moto_Data1[8] = {0};
Moto_DataTypedef Moto_Data2[8] = {0};


	/**
  * @brief          ������ƺ���
  * @param[in]      3508���Ʒ�Χ -16384~0~16384 
  * @param[in]      6020���Ʒ�Χ [-30000,30000]
  * @param[in]      2006���Ʒ�Χ  -10000~0~10000
  * @retval         none
	*/
void CAN_CMD_MOTO(CAN_HandleTypeDef *CAN_ID,uint32_t MOTO_ID,int16_t data0, int16_t data1, int16_t data2, int16_t data3)
{
	  CAN_TxHeaderTypeDef  tx_message;
    uint8_t              can_send_data[8];
	
    uint32_t send_mail_box;
    tx_message.StdId = MOTO_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = (data0 >> 8);
    can_send_data[1] = data0;
    can_send_data[2] = (data1 >> 8);
    can_send_data[3] = data1;
    can_send_data[4] = (data2 >> 8);
    can_send_data[5] = data2;
    can_send_data[6] = (data3 >> 8);
    can_send_data[7] = data3;
    HAL_CAN_AddTxMessage(CAN_ID, &tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t CAN_rec[8];
	int32_t diff;

	if(hcan ->Instance == CAN1)    //�ж����ĸ�CAN�յ�����Ϣ
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, CAN_rec);
		
		Moto_Data1[rx_header.StdId-0x201].angle_last = Moto_Data1[rx_header.StdId-0x201].angle;
		Moto_Data1[rx_header.StdId-0x201].angle = (uint16_t)(CAN_rec[1] | CAN_rec[0]<<8);
		Moto_Data1[rx_header.StdId-0x201].speed = (uint16_t)(CAN_rec[3] | CAN_rec[2]<<8);
		Moto_Data1[rx_header.StdId-0x201].current = (uint16_t)(CAN_rec[5] | CAN_rec[4]<<8);
		Moto_Data1[rx_header.StdId-0x201].temperate = CAN_rec[6];
		
		diff = Moto_Data1[rx_header.StdId-0x201].angle - Moto_Data1[rx_header.StdId-0x201].angle_last;
		if(diff < -4096) diff += 8192;
		else if(diff > 4096) diff-=8192;
		Moto_Data1[rx_header.StdId-0x201].angle_sum += diff;
	}
	else if(hcan ->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, CAN_rec);
		
		Moto_Data2[rx_header.StdId-0x201].angle_last = Moto_Data2[rx_header.StdId-0x201].angle;
		Moto_Data2[rx_header.StdId-0x201].angle = (uint16_t)(CAN_rec[1] | CAN_rec[0]<<8);
		Moto_Data2[rx_header.StdId-0x201].speed = (int16_t)(CAN_rec[3] | CAN_rec[2]<<8);
		Moto_Data2[rx_header.StdId-0x201].current = (uint16_t)(CAN_rec[5] | CAN_rec[4]<<8);
		Moto_Data2[rx_header.StdId-0x201].temperate = CAN_rec[6];
		
		diff = Moto_Data2[rx_header.StdId-0x201].angle - Moto_Data2[rx_header.StdId-0x201].angle_last;
		if(diff < -4096) diff += 8192;
		else if(diff > 4096) diff-=8192;
		Moto_Data2[rx_header.StdId-0x201].angle_sum += diff;
		
	}
}

/**
  * @brief          CAN��������ʼ������
  * @param[in]      h_can:CAN���ָ��
  * @retval         none
  */

HAL_StatusTypeDef CAN_Filter_Init(CAN_HandleTypeDef *h_can) 
{
  CAN_FilterTypeDef sFilterConfig;
 
  sFilterConfig.FilterBank = 2;   //chenal 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //��ʶ������λģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //������λ��Ϊ����32λ
  sFilterConfig.FilterIdHigh = 0x0000;  //��ʶ���Ĵ���   
  sFilterConfig.FilterIdLow = 0x0000;   //��ʶ���Ĵ���   
  //MASK bit 0 means don't care,bit 0 means match 
  sFilterConfig.FilterMaskIdHigh = 0x0000;   //���μĴ���  //ֻ�����ڱ�ʶ������λģʽ�У��ڱ�ʶ���б�ģʽ��Ϊ��ʶ���Ĵ��� 
  sFilterConfig.FilterMaskIdLow = 0x0000;    //���μĴ���                                 
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //FIFO0���жϺ�FIFO1���ж��ǲ�һ���ģ������ǰѽ��յ��ı��ķ��뵽FIFO0��
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;   //enable filter
  sFilterConfig.SlaveStartFilterBank = 0;    //Ϊ����canѡ��ʼ�Ĺ��˿⣬���ڵ���CANʵ�����������û������
  if (HAL_CAN_ConfigFilter(h_can, &sFilterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  //regist RX_IT
  if (HAL_CAN_ActivateNotification(h_can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //ע��CAN_IT_RX_FIFO0_MSG_PENDING ��Ӧ�Ļص�����ԭ��
  {
    return HAL_ERROR;
  }
 return HAL_OK;
}



