#ifndef STATE_LED_H
#define STATE_LED_H

#include "main.h"

#define normal  0
#define error   1

typedef struct _system_data_t{
	int8_t sysyem_state;
}system_data_t;

//led״̬
void led_error();

#endif



