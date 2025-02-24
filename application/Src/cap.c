#include "cap.h"

//pid控制环参数
float PID_BUCK_V[3] ={BUCK_V_P,BUCK_V_I,BUCK_V_D},
			PID_BUCK_I[3] ={BUCK_I_P,BUCK_I_I,BUCK_I_D},
			PID_BOOST_V[3]={BOOST_V_P,BOOST_V_I,BOOST_V_D},
			PID_BOOST_I[3]={BOOST_I_P,BOOST_I_I,BOOST_I_D};
//电流环限幅，pwm输出
float expect_i,pwm_out;	
//A通道pwm占空比输出，D通道pwm占空比输出
float MosA_Duty, MosD_Duty, Samp_Dupy;

//pid数据结构体
pid_type_def pid_buck_v,pid_buck_i,pid_boost_v,pid_boost_i;
			
//功率控制数据返回指针
supercap_measure_t supercap_measure;
			
//CAN发送数据包变量
int16_t can_send_data[4] = {0};

//运放采样校准系数
adc_data_t adc_data= 
{//y=k*x+b
//	.bat_v_m=1.0187f,   .bat_v_a=0.0034f,
//	.bat_i_m=0.8863,  	.bat_i_a=-0.0713f,
//	.cap_v_m=1.0158f,   .cap_v_a=0.0398f,
//	.cap_i_m=-0.9289f,  .cap_i_a=-0.127f,
//	.chas_v_m=1.0126f,  .chas_v_a=0.0323f,
//	.chas_i_m=-0.8648f, .chas_i_a=0.1839f, 

	.bat_v_m=1.0187f,   	.bat_v_a=0.0034f,
	.bat_i_m=1.0094f,  		.bat_i_a=-0.1620f,
	.cap_v_m=1.0158f,   	.cap_v_a=0.0398f,
	.cap_i_m=-0.991f,  		.cap_i_a=-0.0985f,
	.chas_v_m=1.0126f,  	.chas_v_a=0.0323f,
	.chas_i_m=-0.9301f,  	.chas_i_a=-0.119f,
};

//充放电功率控制参数
power_data_t power_data=
{
	.power_mode=Wait,//待机模式
	.cap_max_change_V=24.0f,
	.cap_max_change_I=1.0f,
	.cap_max_output_V=24.0f,
	.cap_max_output_I=1.0f,
	.cap_min_output_V=3.5f,
	
	.power_limit=24,//设定输出功率为24W
	.power_need=48,//设定需求功率为48W
};

//输入，输出，电容保护参数
VI_over_data_t VI_over_data=
{
	.max_short_i=3, 	//最大短路电流
	.min_short_v=8, 	//最小短路电压
	.max_over_i=3,  	//最大输出,电容电流 
	.max_over_v=30,  	//最大输出,电容电流
	.max_under_v=8, 	//最小输入电压，欠压
};

/*******功能函数*******/
//浮点数限幅函数
float Constrain_float(float amt, float high, float low)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}
/*******功能函数*******/

//系统初始化
void system_init(void)
{
	//tim
	HAL_TIM_Base_Start_IT(&htim2);
	
	//adc
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data.adc1_list_record,3*aver_sample_len);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc_data.adc2_list_record,3*aver_sample_len);
	
	//hrtim
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道打开
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //开启子定时器
	
	//can
	CAN_Init();
	//pid
	PID_init(&pid_buck_v,PID_BUCK_V,BUCK_V_PMAX,(BUCK_V_PIDMAX/BUCK_V_I),BUCK_V_PIDMAX);//电压环
	PID_init(&pid_buck_i,PID_BUCK_I,BUCK_I_PMAX,(1/BUCK_I_I),0.90);//电流环
	
	PID_init(&pid_boost_v,PID_BOOST_V,BOOST_V_PMAX,(BOOST_V_PIDMAX/BOOST_V_I),BOOST_V_PIDMAX);//电压环
	PID_init(&pid_boost_i,PID_BOOST_I,BOOST_I_PMAX,(1/BOOST_I_I),0.90);//电流环
}

//模拟数据获取与处理
void adc_solve(void)
{
	adc_data.bat_v = (float)adc_data.adc_list_aver[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
	adc_data.bat_i = ((float)adc_data.adc_list_aver[5] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
	adc_data.cap_v = (float)adc_data.adc_list_aver[3] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
//	adc_data.cap_i = ((float)adc_data.adc_list_aver[4] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.cap_i_m + adc_data.cap_i_a;//在定时器中断里计算[4]
	adc_data.chas_v = (float)adc_data.adc_list_aver[1] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
	adc_data.chas_i = -((float)(adc_data.adc_list_aver[2]) / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.chas_i_m + adc_data.chas_i_a;
	//清零
	for(uint8_t i = 0; i < 4; i++)
	{
		adc_data.adc_list_sum[i] = 0;
	}
	adc_data.adc_list_sum[5] = 0;
	//求和
	for(uint8_t i = 0; i < aver_sample_len; i++)
	{
		for(uint8_t j = 0; j < 3; j++)
		{
			adc_data.adc_list_sum[j] += adc_data.adc1_list_record[i][j];
		}
		adc_data.adc_list_sum[3] += adc_data.adc2_list_record[i][0];
		adc_data.adc_list_sum[5] += adc_data.adc2_list_record[i][2];
	}
	//取平均
	for(uint8_t i = 0; i < 4; i++)
	{
		adc_data.adc_list_aver[i] = adc_data.adc_list_sum[i] / aver_sample_len;
	}
	adc_data.adc_list_aver[5] = adc_data.adc_list_sum[5] / aver_sample_len;		
}

//PWM功率输出
void Power_Loop()
{
	switch(power_data.power_mode)
	{
		case Buck:
		{
			//电容组充电为正电流，放电为负电流
			//BUCK充电模式	
			//目的:为电容组充电，使得电容组的电压能保持在24V左右		
			//电压外环
			expect_i=PID_calc(&pid_buck_v,adc_data.cap_v,power_data.cap_max_change_V);
			//功率环电流限幅
			expect_i=min(expect_i,power_data.cap_max_change_I);
			//电流内环
			pwm_out=PID_calc(&pid_buck_i,adc_data.cap_i,expect_i);
			MosD_Duty=CONDUCTION_DUTY;
			// 占空比限幅
			MosA_Duty=1-Constrain_float(pwm_out,BUCK_DUTY_MAX,BUCK_DUTY_MIN);
			
			//转换为PWM占空比
			Set_HRTIMA(Floor(MosA_Duty * PERIOD));
			Set_HRTIMD(Floor(MosD_Duty * PERIOD));
			Set_Sample(Floor(MosA_Duty * PERIOD)/2); 	
		}break;
		case Boost:
		{
			//BOOST放电模式
			//当底盘输出端输出功率不足时，负载会将底盘输出端电压拉低，此时利用电容组电能使得输出端电压维持在24V左右
			//当电容组电压为8V时，不能继续向底盘输出电流，切换为纯底盘输出模式
			//目的:利用电容组的电能，使得保持底盘输出端的电压在24V左右
			//电压外环
			expect_i=PID_calc(&pid_boost_v,adc_data.chas_v,power_data.cap_max_output_V);
			//功率环电流限幅
			expect_i=min(expect_i,power_data.cap_max_output_I);
			//电流内环
			pwm_out=PID_calc(&pid_boost_i,-adc_data.cap_i,expect_i);//此处将电流值转为正值，方便控制
			MosD_Duty=CONDUCTION_DUTY;
			// 占空比限幅
			MosA_Duty=Constrain_float(pwm_out,BOOST_DUTY_MAX,BOOST_DUTY_MIN);
			
			//转换为PWM占空比
			Set_HRTIMA(Floor(MosA_Duty * PERIOD));
			Set_HRTIMD(Floor(MosD_Duty * PERIOD));
			Set_Sample(Floor(MosA_Duty * PERIOD)/2);
		}break;
		case PWM_off:
		{
 			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
		}break;
		default:break;
	}
}

 
//当输出端电流较大且电压较低时，判定为发生短路
void voltage_circuitr_off()
{
	int8_t reset_num;
	
	if((fabs(adc_data.chas_v)<VI_over_data.min_short_v&&fabs(adc_data.chas_i)>VI_over_data.max_short_i)//输出短路保护
		||(fabs(adc_data.chas_i)>VI_over_data.max_over_i||fabs(adc_data.cap_i)>VI_over_data.max_over_i)  //输出,电容软件过流保护
		||(fabs(adc_data.chas_v)>VI_over_data.max_over_v||fabs(adc_data.cap_v)>VI_over_data.max_over_v)	 //输出,电容软件过压保护
		||(fabs(adc_data.bat_v)>VI_over_data.max_over_v)  																							 //输入过压保护
		||(fabs(adc_data.chas_i)>VI_over_data.max_under_v))															 						 //输入欠压保护 
	{
		//关闭PWM输出
		power_data.power_mode=PWM_off;
	}
	//输出短路保护恢复 
	//发生短路保护后，停止输出后检查故障情况，等待重启
	if(power_data.power_mode==PWM_off)
	{
		
	}
}
int16_t ddd=0;
//模式选择
void mode_choose()
{
	//当电容非满电状态，且实际底盘输出为0时，开始Buck充电
	//电容电压与设定满电电压误差在1V左右判断为满电，后续修改为电荷量计算
	if(fabs(adc_data.cap_v-power_data.cap_max_change_V)>0.9f
		 &&fabs(adc_data.chas_i)<0.1f&&ddd==0)	 
	{
		power_data.power_mode=Buck;
	}
	//电容满电后开启Boost模式，准备输出
	if(adc_data.cap_v>10.0f&&fabs(adc_data.cap_i)<0.1f&&ddd==1)
	{
		power_data.power_mode=Boost;
	}
}

//根据限制的功率，限制电池的输入功率(即提高电容组的输出功率)
//更新功率限制
void power_limit()
{
	//获取电容输出电流设定值
	//超电目的：控制电池输入功率保持在恒定值
	//方式：控制电池输入电流=限定功率/24
	//（输出功率需求-限定功率）/24=电容组输出电流限定值
	power_data.cap_max_output_I=(power_data.power_need-power_data.power_limit)/power_data.cap_max_output_V;
}

//状态灯模式

//CAN发送数据包
void can_send()
{
	can_send_data[0] = float_to_uint(adc_data.cap_v,-30,30,12);
	can_send_data[1] = float_to_uint(adc_data.cap_i,-12.5,12.5,16);
	can_send_data[2] = 0;
	CAN_SendData_int16_t(&hcan,0x030,can_send_data);
}
//CAN通讯返回数据解算
void CAN_supercap_rx_Data(supercap_measure_t *can_rx_data,uint8_t data[8])
{
	can_rx_data->power_mode=data[0];
	can_rx_data->power_state=data[1];
	can_rx_data->cap_max_change_I=((data)[2]<<8|(data)[3]);
	can_rx_data->cap_max_output_I=((data)[4]<<8|(data)[5]);
	can_rx_data->power_need=((data)[6]<<8|(data)[7]);
}

