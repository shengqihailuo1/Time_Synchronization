#include "timer.h"
#include "led.h"
#include "usart.h"
#include "sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


/*

void TIM1_Init(uint16_t arr, uint16_t psc)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // ??TIM1??

    TIM1->ARR = arr - 1; // ??????
    TIM1->PSC = psc - 1; // ????

    TIM1->DIER |= TIM_DIER_UIE; // ??????
    TIM1->CR1 |= TIM_CR1_CEN; // ?????

    NVIC_EnableIRQ(TIM1_UP_IRQn); // ??TIM1??
    NVIC_SetPriority(TIM1_UP_IRQn, 0); // ????????
}

extern vu32 global_time1;
void TIM1_UP_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;

        global_time1++; 
			
    }
}


*/

/*
uint16_t arr_global = 0;
void TIM1_Init(uint16_t arr, uint16_t psc) {

		GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
		arr_global = arr;
	
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // ??TIM1??
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
	
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //�TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
	

    TIM_TimeBaseStructure.TIM_Period = arr; // ???????????????????????????
    TIM_TimeBaseStructure.TIM_Prescaler = psc; // ??????TIMx???????????
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ??????:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ???????,????
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // ???TIMx
	
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  // TIM1??
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // ?????0?
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  // ????3?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ?????
    NVIC_Init(&NVIC_InitStructure);  // ??NVIC_InitStruct???????????NVIC???
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM??1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // ????
    TIM_OCInitStructure.TIM_Pulse = arr * 0.2; // ??????20%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // ????????
    TIM_OC2Init(TIM1, &TIM_OCInitStructure); // ???TIM1??2
	
    TIM_Cmd(TIM1, ENABLE);  // ??TIM1??
	
}
*/
/*
uint16_t arr_global = 0;
void TIM1_Init(uint16_t arr, uint16_t psc) {
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
			arr_global = arr;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//?????1??
 
   //???TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //???????????????????????????
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //??????TIMx??????????? 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //??TIM_TimeBaseInitStruct?????????TIMx???????
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //?????
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//??????
	TIM_ITConfig(TIM1, TIM_IT_Update|TIM_IT_Trigger,ENABLE ); //?????TIM1??,?????? ??????? 
	
	//?????NVIC??
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //?????0?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
	NVIC_Init(&NVIC_InitStructure);  //???NVIC???
 
	TIM_Cmd(TIM1, ENABLE);  //??TIM1	
	
 
}
*/
extern uint16_t received_cam_freq;
uint16_t arr_global = 0;
void TIM1_Init(uint16_t arr, uint16_t psc) 
{  
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

//arr_global = arr;
		
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

/*
//PA11��Ϊpps�����źţ�1hz
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
GPIO_Init(GPIOA,&GPIO_InitStructure);
		
TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2;
TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse=arr/5;//0.8
TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
//TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;
TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//����ʱ������͵�ƽ
TIM_OC4Init(TIM1,&TIM_OCInitStructure);//pa11��ch4
*/

//PA10��Ϊ����Ĵ����źţ�40��20hz��
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
GPIO_Init(GPIOA,&GPIO_InitStructure);

//TIM_OCStructInit(&TIM_OCInitStructure);
TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2;//TIM_OCMode_PWM1
TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse=arr * 0.5;
TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;
TIM_OC3Init(TIM1,&TIM_OCInitStructure);//pa10��ch3

TIM_TimeBaseInitStructure.TIM_Period = arr;
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInitStructure.TIM_RepetitionCounter =0;
TIM_TimeBaseInit(TIM1, & TIM_TimeBaseInitStructure);

//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//���Ҫ�õ�pa11���Ϳ�������
TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1??
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //????3?
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
NVIC_Init(&NVIC_InitStructure);  //???NVIC???

TIM_Cmd(TIM1,ENABLE);
TIM_CtrlPWMOutputs(TIM1, ENABLE);
}




//unsigned char checksum;
vu32 count = 0;
vu32 sum_ms = 0;
//char time2_snum[11];
char time2_snum[64];
uint32_t pwm_counter = 0;
char nmea[128];
char gprmc_data[128]; 
u16 can_send_gprmc = 0,can_send_cam = 0; //ȫ�ֱ���

void TIM1_UP_IRQHandler(void) {
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); 
		count ++;
		//sum_ms = 1000 / received_cam_freq * (count-1);//�ܺ�����		
		sum_ms = 1000.0 / received_cam_freq * count;//�ܺ�����			
    if (sum_ms  % 1000 == 0) {

			//memset(nmea, 0, sizeof(nmea));
			sprintf(nmea, "$GPRMC,%02d%02d%02d.00,A,2302.24277910,N,11323.43787740,E,0.017,308.4,120318,3.2,W,A*", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60);				
			//checksum = calculateChecksum(nmea);
			//printf("%s%02X", nmea, checksum);

			//snprintf(gprmc_data, sizeof(gprmc_data), "%s%02X", nmea, checksum);		
			//printf("%s", gprmc_data);
			//memset(gprmc_data, 0, sizeof(gprmc_data));
			can_send_gprmc = 1;
			
    }	
		
		if ( (received_cam_freq == 20 && sum_ms % 50 == 0) || (received_cam_freq == 40 && sum_ms % 25 == 0)  || (received_cam_freq == 10 && sum_ms % 100 == 0) ) 
		{
			
			
			memset(time2_snum, 0, sizeof(time2_snum));
			////���������Ƶ��ʱ20hz���򾭹���50ms��Ҫ����һ��$cam��Ϣ�����������Ƶ��ʱ40hz���򾭹���25ms��Ҫ����һ��$cam��Ϣ.
			sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			printf("%s",time2_snum);
			
			
			//sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			//can_send_cam = 1;
			
		}
		
	}
}


/*ԭ����
unsigned char checksum;
vu32 count = 0;
vu32 sum_ms = 0;
//char time2_snum[11];
char time2_snum[128];
uint32_t pwm_counter = 0;
char nmea[128];
void TIM1_UP_IRQHandler(void) {
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {	
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); 
		count ++;
		//sum_ms = 1000 / received_cam_freq * (count-1);//�ܺ�����		
		sum_ms = 1000.0 / received_cam_freq * count;//�ܺ�����			
    if (sum_ms  % 1000 == 0) {
			memset(nmea, 0, sizeof(nmea));
			sprintf(nmea, "$GPRMC,%02d%02d%02d.00,A,2302.24277910,N,11323.43787740,E,0.017,308.4,120318,3.2,W,A*", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60);				
			checksum = calculateChecksum(nmea);
			printf("%s%02X", nmea, checksum);
		}	
	
		if ( (received_cam_freq == 20 && sum_ms % 50 == 0) || (received_cam_freq == 40 && sum_ms % 25 == 0) ) 
		{
			memset(time2_snum, 0, sizeof(time2_snum));
			//���������Ƶ��ʱ20hz���򾭹���50ms��Ҫ����һ��$cam��Ϣ�����������Ƶ��ʱ40hz���򾭹���25ms��Ҫ����һ��$cam��Ϣ.
			sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#\n", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			printf("%s",time2_snum);
			
		}
	}
}
*/



/*
//���Ҫ��ͬһ��time1������1hz��pps�źź�40/20hz����������źţ���������ĺ��������ǣ����ɵ�pps�ź���ë�̣����ܻ�Ӱ��livox����ʱ�䡣
unsigned char checksum;
vu32 count = 0;
vu32 sum_ms = 0;
char time2_snum[11];
uint32_t pwm_counter = 0;
char nmea[128];
void TIM1_UP_IRQHandler(void) {
	    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
				
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); 
    
		count ++;
    sum_ms = 1000 / received_cam_freq * (count-1);//�ܺ�����
				
        if (sum_ms % 1000 == 0) {
			//������;�����һ�롣��ÿ�뿪ʼʱ����pwm����
			//�ȴ���time3����pps�źţ��ٴ�ӡgprmc����
            pwm_counter = 0;
        }    
				
				
		if (pwm_counter <= received_cam_freq / 5) {//����pps�ߵ�ƽά��200ms
			TIM_SetCompare4(TIM1, arr_global); // �ߵ�ƽ
		} else {
	TIM_SetCompare4(TIM1, 0); //�͵�ƽ
		}
		pwm_counter++;
				
				
    if (sum_ms  % 1000 == 0) {
		
			memset(nmea, 0, sizeof(nmea));
			sprintf(nmea, "$GPRMC,%02d%02d%02d.00,A,2302.24277910,N,11323.43787740,E,0.017,308.4,120318,3.2,W,A*", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60);				
			checksum = calculateChecksum(nmea);
			printf("%s%02X\n", nmea, checksum);
    }
		
		
		if ( (received_cam_freq == 20 && sum_ms % 50 == 0) || (received_cam_freq == 40 && sum_ms % 25 == 0) ) 
		{
			//���������Ƶ��ʱ20hz���򾭹���50ms��Ҫ����һ��$cam��Ϣ�����������Ƶ��ʱ40hz���򾭹���25ms��Ҫ����һ��$cam��Ϣ.
			sprintf(time2_snum,"%02d%02d%02d.%03d", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			time2_snum[10]=0;
			printf("$cam,");
			printf(time2_snum);
			printf("\n");
		}
		
		
	}
}
*/


	/*
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        static uint32_t delay_counter = 0;
        
        delay_counter++;
        if (delay_counter >= 200) { //��ʱ�ߵ�ƽ200ms
            // ?? TIM2
            TIM_Cmd(TIM2, DISABLE);
            
            GPIO_ResetBits(GPIOB, GPIO_Pin_5); // ����PB5Ϊ�͵�ƽ

            delay_counter = 0; // ?????
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ?????????
    }
}
*/

/*
//1,����TIM3_PWM_Init������
//2,����LED0_PWM_VAL�궨�壬����TIM3_CH2����									  
//////////////////////////////////////////////////////////////////////////////////  
   	  
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}
*/

/*
vu16 varl=0;
//��ʱ��3�жϷ������
vu16 var_Exp=0;
vu16 global_time ;
char snum[7];
vu16 shorttt=0;

void TIM3_IRQHandler(void)   //TIM3�ж�
{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		LED1=!LED1;
			
			//var_Exp �����ع�ʱ�������Ʋ���
		//TIM2->CNT=TIM2->ARR/2;  //���ﱻ��ע�͵��ˣ����Ҫtime���10HZ�Ļ�����ȡ��ע�͡�

		PCout(13)=0;
		}		
		global_time++;
		shorttt=0;
		sprintf(snum, "%06d", global_time-1); //��000000��ʼ��������"0000xx",ʱ����
		snum[6]=0;
		printf("$GPRMC,");
		printf(snum);
		printf(".00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*23\n");

}
*/

/*
//����У���
unsigned char calculateChecksum(const char *data) {
    unsigned char checksum = 0;
    int i;

    for (i = 1; data[i] != '*'; i++) {
        checksum ^= data[i];
    }

    return checksum;
}
vu16 totalSeconds ;
char gprmc[128];
unsigned char checksum;
void TIM3_IRQHandler(void) {
		char *nmea = (char*)malloc(128);			
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  // ?????????
        LED1=!LED1;
    }
		
		totalSeconds++;		
		 sprintf(gprmc, "$GPRMC,%02d%02d%02d.00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A", (totalSeconds-1) / 3600, ((totalSeconds-1) % 3600) / 60,(totalSeconds-1) % 60);
		checksum = calculateChecksum(gprmc);
		   sprintf(nmea, "%s*%02X\n", gprmc, checksum);

    printf("%s", nmea);

        free(nmea);
		
}
*/


/*
//����У���
int calculate_checksum(char *data) {
    int checksum = 0;
    int i;
    for (i = 1; data[i] != '*'; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void TIM3_IRQHandler(void) {
	static char nmea_data[120];
	static int time_seconds = 0;
	static int checksum = 0;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 
	LED1=!LED1;
	}
	time_seconds++;

	
	sprintf(nmea_data, "$GPRMC,%02d%02d%02d.00,A,2302.24277910,N,11323.43787740,E,0.017,308.4,120318,3.2,W,A*", (time_seconds-1) / 3600,((time_seconds-1) % 3600) / 60,(time_seconds-1) % 60);				
	//sprintf(nmea_data, "$GPRMC,%02d%02d%02d.00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*", (time_seconds-1) / 3600,((time_seconds-1) % 3600) / 60,(time_seconds-1) % 60);				
	checksum = calculate_checksum(nmea_data);
	printf("%s%02X\n", nmea_data, checksum);
}


//��ʱ��2�жϷ������  //40hz,  ÿ25ms�ʹ���һ���жϺ�����
vu16 time2_global_time;//Ĭ��ֵΪ0
vu16 h=0;
vu16 m=0;
vu16 s=0;
vu32 ms=0;//���Դ�49��ĺ���������
char time2_snum[11];
extern uint16_t received_freq;
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
		time2_global_time++;
		ms = (time2_global_time-1) * 25;  //40hz  �ܺ������� 
		//ms = (time2_global_time-1) * (1000.0f / received_freq);//�ܺ������� 
    h = ms / (1000 * 60 * 60);
    ms %= (1000 * 60 * 60);
    m = ms / (1000 * 60);
    ms %= (1000 * 60);
    s = ms / 1000;
    ms = ms % 1000;
		sprintf(time2_snum, "%02d%02d%02d.%03d", h, m, s, ms);
		time2_snum[10]=0;
		printf("$cam,");
		printf(time2_snum);
		printf("\n");
}
*/


/*
void my_TIM2_PWM_Init(uint32_t arr,uint32_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	TIM_DeInit(TIM2);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
		 
	TIM_OCInitStructure.TIM_Pulse = 50;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
	
 	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE); 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

 //	TIM_SetCompare2(TIM2,50);	 // ccr  set pwm value
	TIM_SetCompare2(TIM2,TIM2->ARR/2);
}
*/



//����Ҫ��Ƶ�ʼ��������arr��psc�������������ʵ������κ�Ƶ�ʡ�
//my_TIM2_PWM_Init(period,prescaler);
void my_TIM2_PWM_Init(uint32_t arr,uint32_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	TIM_DeInit(TIM2);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr-1; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
		 
	TIM_OCInitStructure.TIM_Pulse = arr / 2;
	//TIM_OCInitStructure.TIM_Pulse = 50; //ԭ����
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
	
 	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE); 

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

 //	TIM_SetCompare2(TIM2,50);	 // ccr  set pwm value
	TIM_SetCompare2(TIM2,TIM2->ARR/2);
}


/*
void TIM2_PWM_Init(u16 arr,u16 psc) //ԭʼ�汾
{  
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	TIM_DeInit(TIM2);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH1  B6 pin out
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	 
	TIM_OCInitStructure.TIM_Pulse = 50;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
	
//	TIM_OCInitStructure.TIM_Pulse = 500;
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
//	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
//	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  

 	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);  //??TIM4
	
 //	TIM_SetCompare2(TIM2,50);	 // ccr  set pwm value
TIM_SetCompare2(TIM2,TIM2->ARR/2);
}
*/


/*
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
 
  //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //�TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
  //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	//TIM_SetCompare2(TIM3,TIM3->ARR/2);	
	TIM_SetCompare2(TIM3,TIM3->ARR * 0.9);	//ռ�ձ�Ϊ10%��100ms
	//TIM_SetCompare2(TIM3,TIM3->ARR * 0.8);//����ռ�ձ�Ϊ20%
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}
*/




void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
 
  //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //�TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
  //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	//TIM_SetCompare2(TIM3,TIM3->ARR/2);	
	TIM_SetCompare2(TIM3,TIM3->ARR * 0.9);	//ռ�ձ�Ϊ10%��100ms
	//TIM_SetCompare2(TIM3,TIM3->ARR * 0.8);//����ռ�ձ�Ϊ20%
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	*/
}
