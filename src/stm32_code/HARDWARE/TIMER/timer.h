#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/4
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//********************************************************************************

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void my_TIM2_PWM_Init(uint32_t arr,uint32_t psc);
void TIM1_Init(uint16_t arr, uint16_t psc);

//void my_TIM2_PWM_Init(uint16_t f);
//void TIM2_PWM_Init(u16 arr,u16 psc);
unsigned char calculateChecksum(const char *data);

#endif
