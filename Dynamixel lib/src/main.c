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
#include "stm32f4xx.h"
#include <stdbool.h>
#include "gpio.h"
//#include "Dynamixel_control.h"

#define BUTTON pin_id(PORTA, 0)

#define CW  0x0400
#define CCW 0x0000


void RCC_Config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO1;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO1.GPIO_Mode = GPIO_Mode_AF;
    GPIO1.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO1.GPIO_OType = GPIO_OType_PP;
    GPIO1.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PB10-Tx PB11-Rx

    GPIO_Init(GPIOB, &GPIO1);

   /* GPIO_InitTypeDef GPIO1;
    GPIO1.GPIO_Mode = GPIO_Mode_IN;
    GPIO1.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO1.GPIO_OType = GPIO_OType_PP;
    GPIO1.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO1.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO1);*/
    conf_pin(BUTTON, INPUT, PUSH_PULL, LOW_S, PULL_DOWN);
}

void USART_Config(void)
{
    clearServoReceiveBuffer();
    USART_InitTypeDef USART;
    USART.USART_BaudRate = 115200;
    USART.USART_WordLength = USART_WordLength_8b;
    USART.USART_StopBits = USART_StopBits_1;
    USART.USART_Parity = USART_Parity_No;
    USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART);
    USART_Cmd(USART3, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;
    // configure the USART3 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init (&NVIC_InitStructure);

	// enable the USART3 receive interrupt
	USART_ITConfig (USART3, USART_IT_RXNE, ENABLE);

    // enable USART3
    USART_Cmd (USART3, ENABLE);
}

void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks-- );
}


    uint16_t curSpeed, curAngle;


int main(void)
{

    SystemInit();
    RCC_Config();
    GPIO_Config();
    USART_Config();

    /*uint8_t ID = 1;

    bool isFinished = false;

    uint16_t startingPos = 0;
    uint16_t endingPos = 300;

    while(1)
    {
        setServoTorque(ID, 500);
        setServoCWAngleLimit(ID, 0);
        setServoCCWAngleLimit(ID, 1023);
        setServoAngle(ID, startingPos);
        soft_delay(20000000);
        soft_delay(20000000);
        setServoAngle(ID, endingPos);
        soft_delay(20000000);
        soft_delay(20000000);
        setServoAngle(ID, startingPos);
    }

}*/

 //  Towers manipulator controlled from the button on discovery
 //  !!! Servo 11 uses 115200 baudrate !!!

  uint8_t ID = 11;
  uint16_t stVal = 0;
  uint16_t finalVal = 300;
  bool flag = 0;

  while(1)
  {
      while (1)
      {
          if (pin_val(BUTTON))
          {
             soft_delay(1000);
              if (pin_val(BUTTON))
              break;
         }
      }
      flag = setServoAngle(ID, stVal);
      while (1)
      {
          if (!pin_val(BUTTON))
          {
             soft_delay(1000);
              if (!pin_val(BUTTON))
              break;
          }
      }
      flag = setServoAngle(ID, finalVal);
  }
}
