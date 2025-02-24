#include "can_receive.h"

CAN_FilterTypeDef CAN1_FilterConfig;//can1�ṹ��
CAN_RxFrameTypeDef hcanRxFrame;//can���սṹ��
CAN_TxFrameTypeDef hcanTxFrame;//can���ͽṹ
//extern motor_measure_t motor_chassis[7];//rm������ݷ��ؽṹ��
//extern DM_Motor_t DM_Motor[3];//DM������ݷ��ؽṹ��
extern supercap_measure_t supercap_measure;//���ʿ������ݷ���ָ��

/**
************************************************************************
* @brief:      	CAN_Filter_Init: CAN�˲�����ʼ������
* @param[in]:   sFilterConfig: 	 ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// ���˲�
	sFilterConfig->FilterMaskIdLow = 0;						// ���˲�
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// �˲���������FIFO0
	sFilterConfig->FilterBank = 0;							// �����˲���0   ��canΪ0��13��˫canΪ14��28
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan, sFilterConfig);
	HAL_CAN_Start(&hcan);
	
//	sFilterConfig->FilterBank = 14;							// �����˲���0   ��canΪ0��13��˫canΪ14��28
//	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
//	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
//	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
//	sFilterConfig->SlaveStartFilterBank = 0;
//	HAL_CAN_ConfigFilter(&hcan2, sFilterConfig);
//	HAL_CAN_Start(&hcan2);
}

/**
************************************************************************
* @brief:      	CAN_Init: CAN��ʼ��
* @param[in]:   void
* @retval:     	void
* @details:    	CAN��ʼ��
************************************************************************
**/
void CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_Init(&sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
//	// ʹ�ܽ����ж�
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
************************************************************************
* @brief:      	CAN_SendData_int16_t: CAN���ͺ���,����int16_t����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   stdId:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   data:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @retval:     	uint8_t
* @details:    	ͨ��CAN����int16_t����
************************************************************************
**/
uint8_t CAN_SendData_int16_t(CAN_HandleTypeDef *hcan,uint32_t stdId,int16_t *data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame.data[0] = (uint8_t)(data[0] >> 8);
	txFrame.data[1] = (uint8_t)((data[0] << 8) >> 8);
	txFrame.data[2] = (uint8_t)(data[1] >> 8);
	txFrame.data[3] = (uint8_t)((data[1] << 8) >> 8);
	txFrame.data[4] = (uint8_t)(data[2] >> 8);
	txFrame.data[5] = (uint8_t)((data[2] << 8) >> 8);
	txFrame.data[6] = (uint8_t)(data[3] >> 8);
	txFrame.data[7] = (uint8_t)((data[3] << 8) >> 8);		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
************************************************************************
* @brief:      	CAN_SendData_uint8_t: CAN���ͺ���,����uint8_t����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   stdId:    CANid��ַ
* @param[in]:   data:     ��������
* @param[in]:   len:      ���͵����ݳ���
* @retval:     	uint8_t
* @details:    	ͨ��CAN����uint8_t����
************************************************************************
**/
uint8_t CAN_SendData_uint8_t(CAN_HandleTypeDef *hcan,uint32_t stdId,uint8_t *data,uint32_t len)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	
	//�жϲ���ֵ���ݳ���
	if(len<=8)	
	{
	  txFrame.header.DLC=8;     // ���ͳ��ȣ�8byte
	}
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = data[i];
	}		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
************************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: CAN�����жϺ���
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @retval:     	void
* @details:    	ͨ��CAN���߽��շ�������
************************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//can�������ݴ洢
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&hcanRxFrame.header,hcanRxFrame.data);
	
	switch(hcanRxFrame.header.StdId)
	{
//		//RM���
//		case CAN_all_ID:
//		case CAN_first_ID:
//		case CAN_second_ID: 
//		case CAN_third_ID:
//		case CAN_forth_ID:
//		case CAN_gimbal_fir:
//		case CAN_gimbal_sec:
//		{
//			static uint8_t i = 0;
//			//��ȡ���ID
//			i=hcanRxFrame.header.StdId-CAN_first_ID;
//			//can�������ݽ���
//			CAN_rx_rm_Data(&motor_chassis[i],hcanRxFrame.data);
//			break;
//		}
//		//MITģʽ��,DM���
//		case DM_all_Master_ID:
//		case DM_first_Master_ID:
//		case DM_second_Master_ID:
//		{
//			static uint8_t i = 0;
//			//��ȡ���ID
//			i=hcanRxFrame.header.StdId-DM_first_Master_ID;
//			//can�������ݽ���
//			CAN_rx_DM_Data(&DM_Motor[i],hcanRxFrame.data);
//			break;
//		}
		case 0x00:
		{
			CAN_supercap_rx_Data(&supercap_measure,hcanRxFrame.data);
			break;
		}
		default:break;		
	}
}

	