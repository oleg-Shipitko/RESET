/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f4xx_conf.h"
float value = 0;

void GPIOInit()
{
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

      GPIO_InitTypeDef GPIO;
      GPIO_StructInit(&GPIO);
      GPIO.GPIO_Mode = GPIO_Mode_AN;
      GPIO.GPIO_Pin = GPIO_Pin_1;

      GPIO_Init(GPIOA, &GPIO);
}

void ADCInit()
{
       ADC_InitTypeDef ADC_InitStructure;
       ADC_StructInit(&ADC_InitStructure);

       ADC_CommonInitTypeDef adc_init;
       ADC_CommonStructInit(&adc_init);
       /* разрешаем тактирование AЦП1 */
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
       /* сбрасываем настройки АЦП */
       ADC_DeInit();
       ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
       /* АЦП1 и АЦП2 работают независимо */
       adc_init.ADC_Mode = ADC_Mode_Independent;
       adc_init.ADC_Prescaler = ADC_Prescaler_Div2;
       /* выключаем scan conversion */
       ADC_InitStructure.ADC_ScanConvMode = DISABLE;
       /* Не делать длительные преобразования */
       ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
       /* Начинать преобразование программно, а не по срабатыванию триггера */
       ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
       /* 12 битное преобразование. результат в 12 младших разрядах результата */
       ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
       /* инициализация */
       ADC_CommonInit(&adc_init);
       ADC_Init(ADC1, &ADC_InitStructure);
       /* Включаем АЦП1 */
       ADC_Cmd(ADC1, ENABLE);

 // настройка канала
 ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
}

uint16_t ReadADC1(uint8_t channel)
{
   // начинаем работу
   ADC_SoftwareStartConv(ADC1);
   // ждём пока преобразуется напряжение в код
   while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
   // возвращаем результат
   return ADC_GetConversionValue(ADC1);
}


int main(void)
{
    GPIOInit();
    ADCInit();
  while(1)
  {
    value = ReadADC1(1) * 0.0822 * 2.54;
  }
}
