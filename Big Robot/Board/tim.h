#ifndef _TIM_INCLUDED_
#define _TIM_INCLUDED_

#include "stm32f4xx.h"

void timEncoderConfigure(TIM_TypeDef * TIM);
void timPWMConfigure(TIM_TypeDef * TIM, uint16_t prescaler, uint16_t autoReset,
                     char ch1, char ch2, char ch3, char ch4);
void timPIDConfigure(TIM_TypeDef * TIM, uint16_t prescaler, uint16_t autoReset);

#endif
