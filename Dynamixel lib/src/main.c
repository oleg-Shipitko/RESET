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


#define ID_RIGHT 18             // If to look on robot from the front
#define ID_LEFT 1
#define OPEN_ANG_RIGHT 300
#define OPEN_ANG_LEFT 0
#define CLOSED_ANG_RIGHT 0
#define CLOSED_ANG_LEFT 300
#define ONE_CUBE_CATCHED_ANGLE 30      // angle defining difference in manipulators angles in 1 cube is caught
#define TWO_CUBES_CATCHED_ANGLE 90     // angle defining difference in manipulators angles in 2 cubes are caught

void RCC_Config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO1;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO1.GPIO_Mode = GPIO_Mode_AF;
    GPIO1.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO1.GPIO_OType = GPIO_OType_PP;
    GPIO1.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC10-Tx PC11-Rx
    //GPIO1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PB10-Tx PB11-Rx

    GPIO_Init(GPIOC, &GPIO1);

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
    USART.USART_BaudRate = 1000000;
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

    //#define id_right 18
    uint8_t id_right = 18;
    uint8_t id_left = 1;

    uint16_t left_open = 0;  // left open
    uint16_t left_closed = 150; // left closed

    uint16_t right_closed = 150;  // right closed
    uint16_t right_open = 300; // right open

    int8_t num = 0;
    uint16_t delay = 10;

    float right_servo_angle = 0;
    float left_servo_angle = 0;
    float prev_right_servo_angle = 1;
    float prev_left_servo_angle = 1;

bool openCubes()
{
    setServoAngle(id_right, right_open);
    soft_delay(1000);
    setServoAngle(id_left, left_open);

    return 0;
}

bool closeCubes(uint8_t *numberOfCubesCatched)
{


    setServoTorque((uint8_t)id_right, 500);
    setServoTorque(id_left, 500);
    setServoAngle((uint8_t)id_right, right_closed);
    setServoAngle(id_left, left_closed);
    soft_delay(100000);
    while((prev_right_servo_angle != right_servo_angle) || (prev_left_servo_angle != left_servo_angle))
    {
        prev_right_servo_angle = right_servo_angle;
        prev_left_servo_angle = left_servo_angle;
        getServoAngle((uint8_t)id_right, &right_servo_angle);
        soft_delay(1000);
        getServoAngle(id_left, &left_servo_angle);
    }
    float difference = right_servo_angle - left_servo_angle;
    if ((difference > 1) && (difference <= 15))
    {
        *numberOfCubesCatched = 1;
    }
    else if ((difference > 15))
    {
        *numberOfCubesCatched = 2;
    }
    else
        *numberOfCubesCatched = 0;              // no cubes were caught or number of cubes is unknown

    right_servo_angle = 0;
    left_servo_angle = 0;
    prev_right_servo_angle = 1;
    prev_left_servo_angle = 1;

    return 0;
}



int main(void)
{
    SystemInit();
    RCC_Config();
    GPIO_Config();
    USART_Config();

    setServoReturnDelayMicros((uint8_t)id_right, delay);
    setServoReturnDelayMicros((uint8_t)id_left, delay);

    while(1)
    {
        openCubes();
       // getServoAngle((uint8_t)id_right, &right_servo_angle);
        // getServoAngle((uint8_t)id_left, &left_servo_angle);
        soft_delay(1000);
       // getServoAngle(id_left, &left_servo_angle);
        soft_delay(20000000);
        //getServoAngle(id_right, &right_servo_angle);
        soft_delay(1000);
        closeCubes(&num);
        soft_delay(20000000);
    }
}


 //  Towers manipulator controlled from the button on discovery
 //  !!! Servo 11 uses 115200 baudrate !!!

//  uint8_t ID = 11;
//  uint16_t stVal = 0;
//  uint16_t finalVal = 300;
//  bool flag = 0;
//
//  while(1)
//  {
//      while (1)
//      {
//          if (pin_val(BUTTON))
//          {
//
//             soft_delay(1000);
//              if (pin_val(BUTTON))
//              break;
//         }
//      }
//      flag = setServoAngle(ID, stVal);
//      while (1)
//      {
//          if (!pin_val(BUTTON))
//          {
//             soft_delay(1000);
//              if (!pin_val(BUTTON))
//              break;
//          }
//      }
//      flag = setServoAngle(ID, finalVal);
//  }
//}



