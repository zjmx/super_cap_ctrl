#include "state_led.h"

system_data_t system_data;
//led״̬
//��˸:��������
//����:PWM���Ϲر�
void led_error()
{
	switch(system_data.sysyem_state)
	{
		case normal:
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		}break;
		case error:
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}break;
		default:break;
	}
}




