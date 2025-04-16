#include "sys.h"
#include "usart.h"	

#include "stm32f10x_usart.h"
#include "stm32f10x.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

//********************************************************************************
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 

//�ض���fputc����   ��������Ǵ���usart2��
int fputc(int ch, FILE *f) {
    USART_SendData(USART2, (uint8_t) ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

// ��ʼ����׼�����
void stdout_init(void) {
   
    setvbuf(stdout, NULL, _IONBF, 0);
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = 0x00010003;
}

/*
//�ض���fputc����   ��������Ǵ���usart1�ģ���������Ҫ��usart2���ͣ�����Ҫ�ġ�
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
*/
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
int access_camera_fre_success = 0;


//����1.   ����Ҫ��pa10���pwm�������Ի����ô���2�շ����ݡ�
void uart1_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
}

void USART1_IRQHandler(void)	//����1�жϷ������
{
    u8 Res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        Res = USART_ReceiveData(USART1);
        
        if((USART_RX_STA & 0x8000) == 0)
        {
            if(USART_RX_STA & 0x4000)
            {
                if(Res != '\n'){
                    USART_RX_STA = 0;
										access_camera_fre_success = 0;
								}
                else{
                    USART_RX_STA |= 0x8000;//�������
										access_camera_fre_success = 1;
								}
																
            }
            else
            {
                if(Res == '\r')
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0x3FFF] = Res;
                    USART_RX_STA++;
                    if(USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0;
                }
            }
        }
    }
}

/*
void USART1_IRQHandler(void)                	//����1�жϷ������ ԭ����
{
	u8 Res;
	#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
		OSIntEnter();    
	#endif
		
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
			//��ȡһ�� char �� USARTx->DR
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
	#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
		OSIntExit();  											 
	#endif
} 

*/

//����2
void my_uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStrue;
	USART_InitTypeDef USART_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;
	
	// ??????
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
	// ??? ????IO?  TX-PA2  RX-PA3
		GPIO_InitStrue.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
		GPIO_InitStrue.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStrue);	 

	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	NVIC_InitStrue.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStrue);
	
	// ??? ??????
	USART_InitStrue.USART_BaudRate=bound; // ???
	USART_InitStrue.USART_WordLength=USART_WordLength_8b; // ?????????8?
	USART_InitStrue.USART_StopBits=USART_StopBits_1; // ?????
	USART_InitStrue.USART_Parity=USART_Parity_No; // ??????
	USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None; // ?????
	USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; // ?? ?? ?????
	USART_Init(USART2,&USART_InitStrue);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//??????
	USART_Cmd(USART2,ENABLE);//????
}

void USART2_IRQHandler( void )                                        
{
    u8 Res;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        Res = USART_ReceiveData(USART2);
        
        if((USART_RX_STA & 0x8000) == 0)
        {
            if(USART_RX_STA & 0x4000)
            {
                if(Res != '\n'){
                    USART_RX_STA = 0;
										access_camera_fre_success = 0;
								}
                else{
                    USART_RX_STA |= 0x8000;//�������
										access_camera_fre_success = 1;
								}
																
            }
            else
            {
                if(Res == '\r')
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0x3FFF] = Res;
                    USART_RX_STA++;
                    if(USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0;
                }
            }
        }
    }
}



/*
 u8 USART2_RX_BUF[250]; 
  u8 USART2_RX_CNT=0;
  u16 USART2_RX_STA=0;       //??????    
 void USART2_Send_Data(u8 *buf,u16 len)
  {
      u16 t;
        for(t=0;t<len;t++)        //??????
      {           
          while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);      
          USART_SendData(USART2,buf[t]);
      }     
      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);          
  }

  void USART2_Receive_Data(u8 *buf)
  {
      u8 rxlen=USART2_RX_CNT;
      u8 i=0;
      delay_ms(10);        //??10ms,????10ms?????????,???????
      while(rxlen!=USART2_RX_CNT)
      {
          rxlen=USART2_RX_CNT;
          delay_ms(10);
      }
          for(i=0;i<(USART2_RX_CNT);i++)
          {
              buf[i] = USART2_RX_BUF[i];    
              USART2_RX_BUF[i] = 0;
          }        
          USART2_RX_CNT=0;        //??
      
  }

	
	 void USART2_IRQHandler(void)
 {
     u8 res;        
      if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //?????
     {          
         res =USART_ReceiveData(USART2);     //????????        
         if(USART2_RX_STA==0)
         {
             USART2_RX_BUF[USART2_RX_CNT] = res;        //???????    
             //???????0xA0?0xA1????????,????????
             if(USART2_RX_BUF[USART2_RX_CNT-1]==0xA0&&USART2_RX_BUF[USART2_RX_CNT]==0xA1)
                 USART2_RX_STA=1;//????????
             USART2_RX_CNT++;                        //??????1 
         }
         } 
      
     //??-??????????SR,??DR???????????????
     if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) == SET)
     {
         USART_ReceiveData(USART2);
         USART_ClearFlag(USART2,USART_FLAG_ORE);
     }
      USART_ClearFlag(USART2,USART_IT_RXNE); //?????????    
 } 
*/

/*
void USART2_IRQHandler(void) // ����2�жϳ���
{
	u8 Res;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)) // ????
{
        Res = USART_ReceiveData(USART2);
        
        if((USART_RX_STA & 0x8000) == 0)
        {
            if(USART_RX_STA & 0x4000)
            {
                if(Res != '\n'){
                    USART_RX_STA = 0;
										access_camera_fre_success = 0;
								}
                else{
                    USART_RX_STA |= 0x8000;//�������
										access_camera_fre_success = 1;
								}
																
            }
            else
            {
                if(Res == '\r')
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0x3FFF] = Res;
                    USART_RX_STA++;
                    if(USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0;
                }
            }
        }
    }
}
*/

 

#endif	

