#include "sys.h"
#include "usart.h"	

#include "stm32f10x_usart.h"
#include "stm32f10x.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//********************************************************************************
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 

//重定义fputc函数   这个函数是串口usart2的
int fputc(int ch, FILE *f) {
    USART_SendData(USART2, (uint8_t) ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

// 初始化标准输出流
void stdout_init(void) {
   
    setvbuf(stdout, NULL, _IONBF, 0);
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = 0x00010003;
}

/*
//重定义fputc函数   这个函数是串口usart1的，我们现在要用usart2发送，所以要改。
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
*/
#endif 

/*使用microLib的方法*/
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
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
int access_camera_fre_success = 0;


//串口1.   后面要用pa10输出pwm波，所以换成用串口2收发数据。
void uart1_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 
}

void USART1_IRQHandler(void)	//串口1中断服务程序
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
                    USART_RX_STA |= 0x8000;//接受完成
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
void USART1_IRQHandler(void)                	//串口1中断服务程序 原来的
{
	u8 Res;
	#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
		OSIntEnter();    
	#endif
		
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
			//获取一个 char 从 USARTx->DR
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
	#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
		OSIntExit();  											 
	#endif
} 

*/

//串口2
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
                    USART_RX_STA |= 0x8000;//接受完成
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
void USART2_IRQHandler(void) // 串口2中断程序
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
                    USART_RX_STA |= 0x8000;//接受完成
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

