#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include <math.h> 
#include <string.h> 
#include <stdlib.h>




extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern int access_camera_fre_success;

//volatile uint32_t sys_time = 0;
uint16_t received_cam_freq = 20 ;	 
extern u16 can_send_gprmc,can_send_cam ; 
extern char gprmc_data[128]; 
extern char time2_snum[64];
extern char nmea[128];
//void SysTick_Handler(void)
//{
//    sys_time++; 
//}

//����У���
unsigned char calculateChecksum(const char *data) {
    unsigned char checksum = 0;
    int i;
    for (i = 1; data[i] != '*'; i++) {
        checksum ^= data[i];
    }
    return checksum;
}


int main(void)
{
	
	
	
	char received_data[ 200 ];
	uint32_t period,prescaler;
	float midFloat,clkFloat ;
	long clkInt;
	long midInt;
	int i;	
unsigned char checksum;
	
	
	
	//delay_init();	    	 //��ʱ������ʼ��	  
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ
	
	//LED_Init();			     //LED�˿ڳ�ʼ��
	
	//uart1_init(9600);  //usart1 ���ڳ�ʼ��Ϊ9600
	//uart1_init(115200);	 //���ڳ�ʼ��Ϊ115200,��Ϊtime2��Ƶ��Ϊ40hz������Ҫ�ߵ㡣
	//my_uart2_init(9600);//ʹ��pa2��pa3 ��Ϊ����usart2 ch340��TX��RX
	my_uart2_init(115200);//ʹ��pa2��pa3 ��Ϊ����usart2 ch340��TX��RX
	
	
	while(1)
	{
	        if (access_camera_fre_success) // �������
        {
            //memcpy(received_data, USART_RX_BUF, USART_RX_STA & 0x3FFF); // ????????????????
						memcpy(received_data, USART_RX_BUF, 200);
            received_cam_freq = atoi(received_data);
            USART_RX_STA = 0;
					break;
        }
	}
	printf("Received Camera frequency: %u\n", received_cam_freq);
	


/*
	// computer period and prescaler -------------------------------------------
	clkFloat = 72000000.0f/received_cam_freq;
	if(clkFloat-(long)clkFloat>=0.5f)  		clkInt = clkFloat+1;
	else							 		clkInt = (long)clkFloat;
	
	midFloat = sqrt(clkFloat);
	if(midFloat-(long)midFloat>=0.5f)  		midInt = (long)midFloat+1;
	else									midInt = (long)midFloat;
	for(i = midInt;i>=1;i--)
	{
		if(clkInt%i==0)
		{
			prescaler = i;
			period = clkInt/i;
			break;
		}
	}
	printf("computer : period = %u , prescaler = %u , \n",period , prescaler);
	//-------------------------------------------------------------------
*/
	
	
	
	//����ֻ����1hz��pps�źš�
	TIM3_PWM_Init(9999,7199);	 // 1 Hz pwm  pin_B5
	
	
	//���ܵ���Ƶ��Ϊ40����20����10
	if(received_cam_freq == 10){  
		TIM1_Init(9999,719);//�Լ������  
	}else	if(received_cam_freq == 20){
		//TIM1_Init(17999, 0); // 72MHz/(17999+1) = 20Hz,50ms  ԭ����
		//TIM1_Init(2000,1800); //�����period, prescaler  
		TIM1_Init(49999,71);//�Լ������
	}
	else if(received_cam_freq == 40){
		//TIM1_Init(24999, 0); // 72MHz/(24999+1) = 40Hz,25ms  ԭ����
		//TIM1_Init(2517,715); //�����period, prescaler
		TIM1_Init(24999,71);//�Լ������
	}
	else{
		printf("ERROR:The value of received_cam_freq must be 20 or 40!\n");
	}
		
	
	
	
	
	//TIM2_PWM_Init(999,7199); // 10 Hz    pin_A1    
	//my_TIM2_PWM_Init(249, 7199);//40hz     pin_A1 
		
	//my_TIM2_PWM_Init(period,prescaler); 
		
		// TIM1_Init(1000, 71);
		 
	while(1)
	{
		/*
		if(can_send_cam == 1){
			printf("%s",time2_snum);
			memset(time2_snum, 0, sizeof(time2_snum));
			can_send_cam = 0;
		}
		*/
		if(can_send_gprmc == 1){
			checksum = calculateChecksum(nmea);
			printf("%s%02X", nmea, checksum);
			//printf("%s", gprmc_data);
			memset(nmea, 0, sizeof(nmea));
			can_send_gprmc = 0;
		}
		
		//printf("D");
		//USART_SendData(USART1,'C');
	}
	
}
