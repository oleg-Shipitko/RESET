#include "tim.h"

////////////////////////////////////////////////////////////////////////////////
//_________________________________TIMERS_____________________________________//
////////////////////////////////////////////////////////////////////////////////
void timEncoderConfigure(TIM_TypeDef * TIM)
{
  TIM->PSC   = 0x0;
  TIM->ARR   = 0xFFFF;
  TIM->CR1   = TIM_CR1_ARPE     | TIM_CR1_CEN;
  TIM->SMCR  = TIM_SMCR_SMS_0   | TIM_SMCR_SMS_1|TIM_SMCR_ETF_0|TIM_SMCR_ETF_1|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3;
  TIM->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM->CNT   = 0;
}
////////////////////////////////////////////////////////////////////////////////
void timPWMConfigure(TIM_TypeDef * TIM, uint16_t prescaler, uint16_t autoReset,
                     char ch1, char ch2, char ch3, char ch4)
{
  if(ch1)
  {
    TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1FE;
    TIM->CCER  |= TIM_CCER_CC1E;
  }
  if(ch2)
  {
    TIM->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2FE;
    TIM->CCER  |= TIM_CCER_CC2E;
  }
  if(ch3)
  {
    TIM->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3FE;
    TIM->CCER  |= TIM_CCER_CC3E;
  }
  if(ch4)
  {
    TIM->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4FE;
    TIM->CCER  |= TIM_CCER_CC4E;
  }

  TIM->PSC  = prescaler;
  TIM->ARR  = autoReset;
  TIM->CR1  = TIM_CR1_ARPE  | TIM_CR1_CEN | TIM_CR1_URS;
  TIM->DIER = TIM_DIER_UIE;
  TIM->BDTR = TIM_BDTR_MOE  | TIM_BDTR_AOE;

  TIM->BDTR = TIM_BDTR_MOE  | TIM_BDTR_AOE;
  TIM->CCR1 = 0x00;
  TIM->CCR2 = 0x00;
  TIM->EGR  = TIM_EGR_UG;
}
////////////////////////////////////////////////////////////////////////////////
void timPIDConfigure(TIM_TypeDef * TIM, uint16_t prescaler, uint16_t autoReset)
{
  TIM->CNT   = 0;
  TIM->PSC   = prescaler;
  TIM->ARR   = autoReset;
  TIM->DIER |= TIM_DIER_UIE;
  TIM->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}
////////////////////////////////////////////////////////////////////////////////
