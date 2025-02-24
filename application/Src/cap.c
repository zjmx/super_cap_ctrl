#include "cap.h"

//pid���ƻ�����
float PID_BUCK_V[3] ={BUCK_V_P,BUCK_V_I,BUCK_V_D},
			PID_BUCK_I[3] ={BUCK_I_P,BUCK_I_I,BUCK_I_D},
			PID_BOOST_V[3]={BOOST_V_P,BOOST_V_I,BOOST_V_D},
			PID_BOOST_I[3]={BOOST_I_P,BOOST_I_I,BOOST_I_D};
//�������޷���pwm���
float expect_i,pwm_out;	
//Aͨ��pwmռ�ձ������Dͨ��pwmռ�ձ����
float MosA_Duty, MosD_Duty, Samp_Dupy;

//pid���ݽṹ��
pid_type_def pid_buck_v,pid_buck_i,pid_boost_v,pid_boost_i;
			
//���ʿ������ݷ���ָ��
supercap_measure_t supercap_measure;
			
//CAN�������ݰ�����
int16_t can_send_data[4] = {0};

//�˷Ų���У׼ϵ��
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

//��ŵ繦�ʿ��Ʋ���
power_data_t power_data=
{
	.power_mode=Wait,//����ģʽ
	.cap_max_change_V=24.0f,
	.cap_max_change_I=1.0f,
	.cap_max_output_V=24.0f,
	.cap_max_output_I=1.0f,
	.cap_min_output_V=3.5f,
	
	.power_limit=24,//�趨�������Ϊ24W
	.power_need=48,//�趨������Ϊ48W
};

//���룬��������ݱ�������
VI_over_data_t VI_over_data=
{
	.max_short_i=3, 	//����·����
	.min_short_v=8, 	//��С��·��ѹ
	.max_over_i=3,  	//������,���ݵ��� 
	.max_over_v=30,  	//������,���ݵ���
	.max_under_v=8, 	//��С�����ѹ��Ƿѹ
};

/*******���ܺ���*******/
//�������޷�����
float Constrain_float(float amt, float high, float low)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}
/*******���ܺ���*******/

//ϵͳ��ʼ��
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
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //�����Ӷ�ʱ��
	
	//can
	CAN_Init();
	//pid
	PID_init(&pid_buck_v,PID_BUCK_V,BUCK_V_PMAX,(BUCK_V_PIDMAX/BUCK_V_I),BUCK_V_PIDMAX);//��ѹ��
	PID_init(&pid_buck_i,PID_BUCK_I,BUCK_I_PMAX,(1/BUCK_I_I),0.90);//������
	
	PID_init(&pid_boost_v,PID_BOOST_V,BOOST_V_PMAX,(BOOST_V_PIDMAX/BOOST_V_I),BOOST_V_PIDMAX);//��ѹ��
	PID_init(&pid_boost_i,PID_BOOST_I,BOOST_I_PMAX,(1/BOOST_I_I),0.90);//������
}

//ģ�����ݻ�ȡ�봦��
void adc_solve(void)
{
	adc_data.bat_v = (float)adc_data.adc_list_aver[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
	adc_data.bat_i = ((float)adc_data.adc_list_aver[5] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
	adc_data.cap_v = (float)adc_data.adc_list_aver[3] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
//	adc_data.cap_i = ((float)adc_data.adc_list_aver[4] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.cap_i_m + adc_data.cap_i_a;//�ڶ�ʱ���ж������[4]
	adc_data.chas_v = (float)adc_data.adc_list_aver[1] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
	adc_data.chas_i = -((float)(adc_data.adc_list_aver[2]) / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.chas_i_m + adc_data.chas_i_a;
	//����
	for(uint8_t i = 0; i < 4; i++)
	{
		adc_data.adc_list_sum[i] = 0;
	}
	adc_data.adc_list_sum[5] = 0;
	//���
	for(uint8_t i = 0; i < aver_sample_len; i++)
	{
		for(uint8_t j = 0; j < 3; j++)
		{
			adc_data.adc_list_sum[j] += adc_data.adc1_list_record[i][j];
		}
		adc_data.adc_list_sum[3] += adc_data.adc2_list_record[i][0];
		adc_data.adc_list_sum[5] += adc_data.adc2_list_record[i][2];
	}
	//ȡƽ��
	for(uint8_t i = 0; i < 4; i++)
	{
		adc_data.adc_list_aver[i] = adc_data.adc_list_sum[i] / aver_sample_len;
	}
	adc_data.adc_list_aver[5] = adc_data.adc_list_sum[5] / aver_sample_len;		
}

//PWM�������
void Power_Loop()
{
	switch(power_data.power_mode)
	{
		case Buck:
		{
			//��������Ϊ���������ŵ�Ϊ������
			//BUCK���ģʽ	
			//Ŀ��:Ϊ�������磬ʹ�õ�����ĵ�ѹ�ܱ�����24V����		
			//��ѹ�⻷
			expect_i=PID_calc(&pid_buck_v,adc_data.cap_v,power_data.cap_max_change_V);
			//���ʻ������޷�
			expect_i=min(expect_i,power_data.cap_max_change_I);
			//�����ڻ�
			pwm_out=PID_calc(&pid_buck_i,adc_data.cap_i,expect_i);
			MosD_Duty=CONDUCTION_DUTY;
			// ռ�ձ��޷�
			MosA_Duty=1-Constrain_float(pwm_out,BUCK_DUTY_MAX,BUCK_DUTY_MIN);
			
			//ת��ΪPWMռ�ձ�
			Set_HRTIMA(Floor(MosA_Duty * PERIOD));
			Set_HRTIMD(Floor(MosD_Duty * PERIOD));
			Set_Sample(Floor(MosA_Duty * PERIOD)/2); 	
		}break;
		case Boost:
		{
			//BOOST�ŵ�ģʽ
			//�����������������ʲ���ʱ�����ػὫ��������˵�ѹ���ͣ���ʱ���õ��������ʹ������˵�ѹά����24V����
			//���������ѹΪ8Vʱ�����ܼ������������������л�Ϊ���������ģʽ
			//Ŀ��:���õ�����ĵ��ܣ�ʹ�ñ��ֵ�������˵ĵ�ѹ��24V����
			//��ѹ�⻷
			expect_i=PID_calc(&pid_boost_v,adc_data.chas_v,power_data.cap_max_output_V);
			//���ʻ������޷�
			expect_i=min(expect_i,power_data.cap_max_output_I);
			//�����ڻ�
			pwm_out=PID_calc(&pid_boost_i,-adc_data.cap_i,expect_i);//�˴�������ֵתΪ��ֵ���������
			MosD_Duty=CONDUCTION_DUTY;
			// ռ�ձ��޷�
			MosA_Duty=Constrain_float(pwm_out,BOOST_DUTY_MAX,BOOST_DUTY_MIN);
			
			//ת��ΪPWMռ�ձ�
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

 
//������˵����ϴ��ҵ�ѹ�ϵ�ʱ���ж�Ϊ������·
void voltage_circuitr_off()
{
	int8_t reset_num;
	
	if((fabs(adc_data.chas_v)<VI_over_data.min_short_v&&fabs(adc_data.chas_i)>VI_over_data.max_short_i)//�����·����
		||(fabs(adc_data.chas_i)>VI_over_data.max_over_i||fabs(adc_data.cap_i)>VI_over_data.max_over_i)  //���,���������������
		||(fabs(adc_data.chas_v)>VI_over_data.max_over_v||fabs(adc_data.cap_v)>VI_over_data.max_over_v)	 //���,���������ѹ����
		||(fabs(adc_data.bat_v)>VI_over_data.max_over_v)  																							 //�����ѹ����
		||(fabs(adc_data.chas_i)>VI_over_data.max_under_v))															 						 //����Ƿѹ���� 
	{
		//�ر�PWM���
		power_data.power_mode=PWM_off;
	}
	//�����·�����ָ� 
	//������·������ֹͣ����������������ȴ�����
	if(power_data.power_mode==PWM_off)
	{
		
	}
}
int16_t ddd=0;
//ģʽѡ��
void mode_choose()
{
	//�����ݷ�����״̬����ʵ�ʵ������Ϊ0ʱ����ʼBuck���
	//���ݵ�ѹ���趨�����ѹ�����1V�����ж�Ϊ���磬�����޸�Ϊ���������
	if(fabs(adc_data.cap_v-power_data.cap_max_change_V)>0.9f
		 &&fabs(adc_data.chas_i)<0.1f&&ddd==0)	 
	{
		power_data.power_mode=Buck;
	}
	//�����������Boostģʽ��׼�����
	if(adc_data.cap_v>10.0f&&fabs(adc_data.cap_i)<0.1f&&ddd==1)
	{
		power_data.power_mode=Boost;
	}
}

//�������ƵĹ��ʣ����Ƶ�ص����빦��(����ߵ�������������)
//���¹�������
void power_limit()
{
	//��ȡ������������趨ֵ
	//����Ŀ�ģ����Ƶ�����빦�ʱ����ں㶨ֵ
	//��ʽ�����Ƶ���������=�޶�����/24
	//�������������-�޶����ʣ�/24=��������������޶�ֵ
	power_data.cap_max_output_I=(power_data.power_need-power_data.power_limit)/power_data.cap_max_output_V;
}

//״̬��ģʽ

//CAN�������ݰ�
void can_send()
{
	can_send_data[0] = float_to_uint(adc_data.cap_v,-30,30,12);
	can_send_data[1] = float_to_uint(adc_data.cap_i,-12.5,12.5,16);
	can_send_data[2] = 0;
	CAN_SendData_int16_t(&hcan,0x030,can_send_data);
}
//CANͨѶ�������ݽ���
void CAN_supercap_rx_Data(supercap_measure_t *can_rx_data,uint8_t data[8])
{
	can_rx_data->power_mode=data[0];
	can_rx_data->power_state=data[1];
	can_rx_data->cap_max_change_I=((data)[2]<<8|(data)[3]);
	can_rx_data->cap_max_output_I=((data)[4]<<8|(data)[5]);
	can_rx_data->power_need=((data)[6]<<8|(data)[7]);
}

