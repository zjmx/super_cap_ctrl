#ifndef __CAP_H
#define __CAP_H

#include "main.h"
#include "stdint.h"
#include "can.h"
#include "tim.h"
#include "adc.h"
#include "hrtim.h"
#include "can_receive.h"
#include "math.h"
#include "pid.h"

#define PERIOD 46080
#define aver_sample_len 10

//输出模式选择
#define Wait      0//待机
#define Buck 			1//Buck模式
#define Boost 		2//Boost模式
#define PWM_off 	3//关闭输出

//占空比限制
#define CONDUCTION_DUTY 0.90f
#define BUCK_DUTY_MAX 	0.90f
#define BUCK_DUTY_MIN 	0.01f
#define BOOST_DUTY_MAX 	0.90f
#define BOOST_DUTY_MIN 	0.03f

// PID参数
#define BUCK_I_P 0.0003f
#define BUCK_I_I 0.0038f
#define BUCK_I_D 0
#define BUCK_I_PMAX 1.0f
#define BUCK_I_DMAX 0
#define BUCK_I_IMAX (1/BUCK_I_I)

#define BUCK_V_P 3.5615f
#define BUCK_V_I 0.1599f
#define BUCK_V_D 0
#define BUCK_V_PIDMAX 12.5f
#define BUCK_V_PMAX 0.1f
#define BUCK_V_DMAX 0
#define BUCK_V_IMAX (BUCK_V_PIDMAX/BUCK_V_I)

#define BOOST_I_P 0.0008f
#define BOOST_I_I 0.0032f
#define BOOST_I_D 0
#define BOOST_I_PMAX 12.5f
#define BOOST_I_DMAX 0
#define BOOST_I_IMAX (1/BOOST_I_I)

#define BOOST_V_P 11.8479f
#define BOOST_V_I 0.1136f
#define BOOST_V_D 0
#define BOOST_V_PIDMAX 12.5f
#define BOOST_V_PMAX 10.0f
#define BOOST_V_DMAX 0
#define BOOST_V_IMAX (BOOST_V_PIDMAX/BOOST_V_I)

//采样参数结构体
typedef struct _adc_data_t{
	uint16_t adc_list_aver[6];
	uint32_t adc_list_sum[6];
	uint16_t adc1_list_record[aver_sample_len][3];
	uint16_t adc2_list_record[aver_sample_len][3];
	
	//参数整定系数
	float bat_v;
	float bat_i;
	float cap_v;
	float cap_i;
	float chas_v;
	float chas_i;
	float bat_v_m;
	float bat_i_m;
	float cap_v_m;
	float cap_i_m;
	float chas_v_m;
	float chas_i_m;
	float bat_v_a;
	float bat_i_a;
	float cap_v_a;
	float cap_i_a;
	float chas_v_a;
	float chas_i_a;
}adc_data_t;

//功率输出结构体
typedef struct _power_data_t{
	uint8_t power_mode;          //输出模式选择
	float cap_max_change_I; //设定电容组最大充电电流
	float cap_max_change_V; //设定电容组最大充电电压
	float cap_max_output_I; //设定电容组最大输出电流
	float cap_max_output_V;	//设定电容组最大输出电压
	float cap_min_output_V;	//设定电容组最小截至电压
	
	float power_need;				//输出功率需求值
	float power_limit;			//功率限定值
	
}power_data_t;

//输入，输出，电容保护结构体
typedef struct _VI_over_data_t{
	float max_short_i; 	//最大短路电流
	float min_short_v; 	//最小短路电压
	float max_over_i;  	//最大输出,电容电流 
	float max_over_v;  	//最大输出,电容电流
	float max_under_v; 	//最小输入电压，欠压
}VI_over_data_t;

typedef struct{	
	uint8_t power_mode;     //输出模式选择
	uint8_t power_state;    //状态
	float cap_max_change_I; //设定电容组最大充电电流
	float cap_max_output_I; //设定电容组最大输出电流
	float power_need;				//设定需求功率
}supercap_measure_t;
extern supercap_measure_t supercap_measure;//功率控制数据返回指针

//系统初始化
void system_init();
//模拟数据获取与处理
void adc_solve();
//PWM功率输出
void Power_Loop();
//mos拯救者
void voltage_circuitr_off();
//模式选择
void mode_choose();
//CAN发送数据包
void can_send();
//CAN通讯返回数据解算
void CAN_supercap_rx_Data(supercap_measure_t *can_rx_data,uint8_t data[8]);

#define Floor(x) (int32_t)(x)
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define Constrain_PID(amt,high,low)		Constrain_float(amt,high,low) //PID限幅,浮点数限幅

//PWM占空比转换0~1->0~40960
#define Set_HRTIMA(x) (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = (PERIOD) - (x))
#define Set_HRTIMD(x) (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (x))
#define Set_Sample(x) (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = (PERIOD) - (x))


#endif



