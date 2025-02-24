#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "main.h"
#include "can.h"
#include "function.h"
#include "cap.h"

extern CAN_HandleTypeDef hcan;

typedef struct{
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
}CAN_RxFrameTypeDef;

typedef struct{
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
}CAN_TxFrameTypeDef;
typedef struct{
	int16_t ecd;//电容电压
	int16_t speed_rpm;//电容电流
	int16_t given_current;//转矩电流
	uint8_t temperate;//电机温度
	int16_t last_ecd;//电机前角度
}motor_measure_t;
extern motor_measure_t motor_chassis[7];//rm电机数据返回结构体


typedef enum
{
	CAN_all_ID=0x200,
	CAN_first_ID=0x201,
	CAN_second_ID=0x202,
	CAN_third_ID=0x203,
	CAN_forth_ID=0x204,
	
	CAN_supercap_ID=0x300,
} can_id;//canID


void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig);
void CAN_Init(void);
uint8_t CAN_SendData_int16_t(CAN_HandleTypeDef *hcan,uint32_t stdId,int16_t *data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
