#include "function.h"

extern UART_HandleTypeDef huart2;

//串口数据包打印函数
void MyPrintf(UART_HandleTypeDef *huart,const char *__format, ...)
{
	#define TX_BUF_LEN  256     /* 发送缓冲区容量，根据需要进行调整 */
	uint8_t TxBuf[TX_BUF_LEN];  /* 发送缓冲区                       */
	
  va_list ap;
  va_start(ap, __format);
  
  /* 清空发送缓冲区 */
  memset(TxBuf, 0x00, TX_BUF_LEN);
  
  /* 填充发送缓冲区 */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);
  int len = strlen((const char*)TxBuf);
  
  /* 往串口发送数据 */
  HAL_UART_Transmit(huart, (uint8_t*)&TxBuf, len, 0xFFFF);
}

//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-PI~PI）
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//限幅函数
void LimitMax(fp32 *input,fp32 max)
{
	if(*input>max)
	{
		*input=max;
	}
	else if(*input<-max)
	{
		*input=-max;
	}
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
************************************************************************
* @brief:      	float_to_int16: 浮点数转换为无符号整数函数
* @param[in]:   a_float:	待转换的浮点数
* @param[in]:   a_max:		范围最大值
* @param[in]:   a_min:		范围最小值
* @param[in]:   b_max:		范围最大值
* @param[in]:   b_min:		范围最小值
* @retval:     	有符号整数结果
* @details:    	将给定的浮点数 a_float 在指定范围 [a_min, a_max] 内进行线性映射，映射结果为一个指定位数的有符号整数
************************************************************************
**/
int16_t float_to_int16(float a_float, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a_float - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}
