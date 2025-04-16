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
	
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //砊IM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	

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
//PA11作为pps触发信号，1hz
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
GPIO_Init(GPIOA,&GPIO_InitStructure);
		
TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2;
TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse=arr/5;//0.8
TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
//TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;
TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//空闲时刻输出低电平
TIM_OC4Init(TIM1,&TIM_OCInitStructure);//pa11是ch4
*/

//PA10作为相机的触发信号，40或20hz。
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
TIM_OC3Init(TIM1,&TIM_OCInitStructure);//pa10是ch3

TIM_TimeBaseInitStructure.TIM_Period = arr;
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInitStructure.TIM_RepetitionCounter =0;
TIM_TimeBaseInit(TIM1, & TIM_TimeBaseInitStructure);

//TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//如果要用到pa11，就开启这行
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
u16 can_send_gprmc = 0,can_send_cam = 0; //全局变量

void TIM1_UP_IRQHandler(void) {
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); 
		count ++;
		//sum_ms = 1000 / received_cam_freq * (count-1);//总毫秒数		
		sum_ms = 1000.0 / received_cam_freq * count;//总毫秒数			
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
			////接受相机的频率时20hz，则经过了50ms就要发送一次$cam消息；接受相机的频率时40hz，则经过了25ms就要发送一次$cam消息.
			sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			printf("%s",time2_snum);
			
			
			//sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			//can_send_cam = 1;
			
		}
		
	}
}


/*原来的
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
		//sum_ms = 1000 / received_cam_freq * (count-1);//总毫秒数		
		sum_ms = 1000.0 / received_cam_freq * count;//总毫秒数			
    if (sum_ms  % 1000 == 0) {
			memset(nmea, 0, sizeof(nmea));
			sprintf(nmea, "$GPRMC,%02d%02d%02d.00,A,2302.24277910,N,11323.43787740,E,0.017,308.4,120318,3.2,W,A*", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60);				
			checksum = calculateChecksum(nmea);
			printf("%s%02X", nmea, checksum);
		}	
	
		if ( (received_cam_freq == 20 && sum_ms % 50 == 0) || (received_cam_freq == 40 && sum_ms % 25 == 0) ) 
		{
			memset(time2_snum, 0, sizeof(time2_snum));
			//接受相机的频率时20hz，则经过了50ms就要发送一次$cam消息；接受相机的频率时40hz，则经过了25ms就要发送一次$cam消息.
			sprintf(time2_snum,"$cam,2018.03.12,%02d:%02d:%02d:%03d#\n", (sum_ms/1000) / 3600, ((sum_ms/1000) % 3600) / 60,(sum_ms / 1000) % 60,sum_ms % 1000);
			printf("%s",time2_snum);
			
		}
	}
}
*/



/*
//如果要用同一个time1来生成1hz的pps信号和40/20hz的相机触发信号，就用这里的函数。但是：生成的pps信号有毛刺，可能会影响livox计算时间。
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
    sum_ms = 1000 / received_cam_freq * (count-1);//总毫秒数
				
        if (sum_ms % 1000 == 0) {
			//到这里就经过了一秒。在每秒开始时更新pwm波形
			//先触发time3发送pps信号，再打印gprmc数据
            pwm_counter = 0;
        }    
				
				
		if (pwm_counter <= received_cam_freq / 5) {//设置pps高电平维持200ms
			TIM_SetCompare4(TIM1, arr_global); // 高电平
		} else {
	TIM_SetCompare4(TIM1, 0); //低电平
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
			//接受相机的频率时20hz，则经过了50ms就要发送一次$cam消息；接受相机的频率时40hz，则经过了25ms就要发送一次$cam消息.
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
        if (delay_counter >= 200) { //延时高电平200ms
            // ?? TIM2
            TIM_Cmd(TIM2, DISABLE);
            
            GPIO_ResetBits(GPIOB, GPIO_Pin_5); // 设置PB5为低电平

            delay_counter = 0; // ?????
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ?????????
    }
}
*/

/*
//1,增加TIM3_PWM_Init函数。
//2,增加LED0_PWM_VAL宏定义，控制TIM3_CH2脉宽									  
//////////////////////////////////////////////////////////////////////////////////  
   	  
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}
*/

/*
vu16 varl=0;
//定时器3中断服务程序
vu16 var_Exp=0;
vu16 global_time ;
char snum[7];
vu16 shorttt=0;

void TIM3_IRQHandler(void)   //TIM3中断
{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		LED1=!LED1;
			
			//var_Exp 做半曝光时长的相移操作
		//TIM2->CNT=TIM2->ARR/2;  //这里被我注释掉了，如果要time输出10HZ的话，就取消注释。

		PCout(13)=0;
		}		
		global_time++;
		shorttt=0;
		sprintf(snum, "%06d", global_time-1); //从000000开始，产生："0000xx",时分秒
		snum[6]=0;
		printf("$GPRMC,");
		printf(snum);
		printf(".00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*23\n");

}
*/

/*
//计算校验和
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
//计算校验和
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


//定时器2中断服务程序  //40hz,  每25ms就触发一次中断函数。
vu16 time2_global_time;//默认值为0
vu16 h=0;
vu16 m=0;
vu16 s=0;
vu32 ms=0;//可以存49天的毫秒总数量
char time2_snum[11];
extern uint16_t received_freq;
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
		time2_global_time++;
		ms = (time2_global_time-1) * 25;  //40hz  总毫秒数量 
		//ms = (time2_global_time-1) * (1000.0f / received_freq);//总毫秒数量 
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



//根据要求频率计算出来的arr和psc，调用这个函数实现输出任何频率。
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
	//TIM_OCInitStructure.TIM_Pulse = 50; //原来的
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
void TIM2_PWM_Init(u16 arr,u16 psc) //原始版本
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
  //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //砊IM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
  //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	//TIM_SetCompare2(TIM3,TIM3->ARR/2);	
	TIM_SetCompare2(TIM3,TIM3->ARR * 0.9);	//占空比为10%，100ms
	//TIM_SetCompare2(TIM3,TIM3->ARR * 0.8);//设置占空比为20%
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
*/




void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
  //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //砊IM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
  //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	//TIM_SetCompare2(TIM3,TIM3->ARR/2);	
	TIM_SetCompare2(TIM3,TIM3->ARR * 0.9);	//占空比为10%，100ms
	//TIM_SetCompare2(TIM3,TIM3->ARR * 0.8);//设置占空比为20%
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	*/
}
