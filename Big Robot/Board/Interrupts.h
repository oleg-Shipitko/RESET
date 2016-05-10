#ifndef _INTERRUPTS_INCLUDED_
#define _INTERRUPTS_INCLUDED_

#include "stm32f4xx.h"

#define EXTI_FALLING_EDGE	0
#define EXTI_RISING_EDGE	1
#define EXTI_BOTH_EDGES		2

void USART3_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI0_IRQHandler(void);
void add_ext_interrupt(unsigned char pin, char edge);
extern void USB_OTG_BSP_TimerIRQ (void);
extern char traceFlag;
void delay(__IO uint32_t nCount);



#endif
