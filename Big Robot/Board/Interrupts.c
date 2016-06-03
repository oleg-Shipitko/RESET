#include "Interrupts.h"

#include "gpio.h"
#include "Regulator.h"
#include "Path.h"
#include "Pins.h"
#include <math.h>
#include "usart.h"
#include "robot.h"
#include "board.h"
#include "Manipulators.h"

int indexSpeeds = 0, indexDists = 0;
char traceFlag, movFlag, endFlag;

int16_t int_cnt = 0;

int16_t vabrationCnt = 0;


////////////////////////////////////////////////////////////////////////////////
//_________________________________TIMERS_____________________________________//
////////////////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
{
  //USB_OTG_BSP_TimerIRQ();
}
////////////////////////////////////////////////////////////////////////////////

void TIM6_DAC_IRQHandler() // 100Hz  // Рассчет ПИД регуляторов колес, манипулятора и считывание данных сонаров
{
   //static char i=0; // Divider by 2 to get 10Hz frequency
   //   set_pin(PWM_DIR[8]);


  TIM6->SR = 0;
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  GetDataForRegulators(); // обновление входных данных для ПИД
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

  if (curState.filtering) SpeedFiltration(&vTargetGlobCA[0],&vTargetGlobF[0]);
  else
    {
      vTargetGlobF[0] = vTargetGlobCA[0];
      vTargetGlobF[1] = vTargetGlobCA[1];
      vTargetGlobF[2] = vTargetGlobCA[2];
}

if (curState.kinemEn) FunctionalRegulator(&vTargetGlobF[0], &robotCoordTarget[0], &robotCoordTarget[0], &regulatorOut[0]); // рассчет  кинематики и насыщения

  /////

  pidLowLevel();       // рассчет ПИД
  pidLowLevelManipulator();

  // Sonars
  getSonarData((char)ADC_SONAR_RIGHT, (char)SONAR_RIGHT);
  getSonarData((char)ADC_SONAR_LEFT, (char)SONAR_LEFT);
  getSonarData((char)ADC_SONAR_FRONT_1, (char)SONAR_FRONT_1);
  getSonarData((char)ADC_SONAR_FRONT_2, (char)SONAR_FRONT_2);
  getSonarData((char)ADC_SONAR_BACK, (char)SONAR_BACK);
  // IR
  getIRData((char)ADC_IR_FRONT, (char)IR_FRONT);
  getIRData((char)ADC_IR_BACK, (char)IR_BACK);
  getIRData((char)ADC_IR_LEFT, (char)IR_LEFT);
  getIRData((char)ADC_IR_RIGHT, (char)IR_RIGHT);
   //   reset_pin(PWM_DIR[8]);
}
////////////////////////////////////////////////////////////////////////////////

void TIM7_IRQHandler() // 33kHz
{


  TIM7->SR = 0;

////////////////////////////////////////////////////////////////////////////////


}
////////////////////////////////////////////////////////////////////////////////


void TIM8_UP_TIM13_IRQHandler() // рассчет траекторного регулятора
{
   // set_pin(PWM_DIR[8]);

  TIM13->SR = 0;
 NVIC_DisableIRQ(TIM6_DAC_IRQn);  //отключение ПИД на время расчета



//  if ((fabs(curPath.lengthTrace) <= fabs(curPath.Coord_local_track[0])) && // достигнута заданная точка по положению и углу
//     (fabs((curPath.phiZad)-(robotCoord[2])) < 0.02))
  if (((fabs(curPath.lengthTrace) - fabs(curPath.Coord_local_track[0])) < 0.005) && ((fabs(curPath.Coord_local_track[1])) < 0.05)&& // достигнута заданная точка по положению и углу
     (fabs((curPath.phiZad)-(robotCoord[2])) < 0.02))
        {

          traceFlag = 1;  // точка достигнута
        }
 else traceFlag = 0;
 if (!movFlag)
    if (points[0].movTask) movFlag = (points[0].movTask)(); else movFlag =1; // действие в процессе движения
 if (traceFlag&&movFlag&&(!endFlag))
    if (points[0].endTask) endFlag = ((char (*)(float))(points[0].endTask))(points[0].endTaskP1); else endFlag = 1; // действие в конечной точке
    if (traceFlag && movFlag && endFlag)
        {
          if (lastPoint > 0) //Остались ли точки в стеке
          {
            CreatePath(&points[1], &points[0], &curPath); // задать новый участок
           totalPointComplite++;

          removePoint(&points[0],&lastPoint); //удалить ткущую точку
          endFlag = 0;
          movFlag = 0;
          traceFlag = 0;
          }
        }


//////////////////////////// COMPUTING SPEEDS /////////////////////////////////

 if (curState.trackEn)
{
   TrackRegulator(&robotCoord[0], &robotSpeed[0], (&curPath), &vTargetGlob[0]); // расчет глобальных скоростей
}

if (curState.collisionAvoidance) collisionAvoidance(&vTargetGlob[0], &vTargetGlobCA[0]);
else
{
      vTargetGlobCA[0] = vTargetGlob[0];
      vTargetGlobCA[1] = vTargetGlob[1];
      vTargetGlobCA[2] = vTargetGlob[2];
}

///////////////////////////////////////////////////////////////////////////

//////////////////CALCULATING TIME for SWITCHING OFF VIBRATION//////////////

vabrationCnt++;
if (vabrationCnt  -  startingTime >= vibratingTime)
{
   switchOffVibration();
}
if (vabrationCnt  -  startingTime >= 17)
{
   switchOffVibration();
}

///////////////////////////////////////////////////////////////////////////

  NVIC_EnableIRQ(TIM6_DAC_IRQn); //включение ПИД
    // reset_pin(PWM_DIR[8]);
}



////////////////////////////////////////////////////////////////////////////////
//__________________________________EXTI______________________________________//
////////////////////////////////////////////////////////////////////////////////


//#define EXTI2_PIN               pin_id(PORTD,0)         //Разъем EXTI2//
void EXTI0_IRQHandler(void)
{
  EXTI->PR=0x1;
  char temp = 2;
  if ( pin_val(EXTI2_PIN) ) temp |=0x80;
  sendAnswer(0x1E,&temp, 1);
}

//#define EXTI5_PIN               pin_id(PORTD,1)         //Разъем EXTI5//
void EXTI1_IRQHandler(void)
{
  EXTI->PR=0x2;
  char temp = 5;
  if ( pin_val(EXTI5_PIN) ) temp |=0x80;
  sendAnswer(0x1E,&temp, 1);
}

//#define EXTI4_PIN               pin_id(PORTD,2)         //Разъем EXTI4//
void EXTI2_IRQHandler(void)
{
  EXTI->PR=0x4;
  char temp = 4;
  if ( pin_val(EXTI4_PIN) ) temp |=0x80;
  sendAnswer(0x1E,&temp, 1);
}

//#define EXTI6_PIN               pin_id(PORTD,3)         //Разъем EXTI6//
void EXTI3_IRQHandler(void)
{
  EXTI->PR=0x8;
  char temp = 6;
  if ( pin_val(EXTI6_PIN) ) temp |=0x80;
  sendAnswer(0x1E,&temp, 1);

}

//#define EXTI9_PIN               pin_id(PORTE,4)         //Разъем EXTI9//
void EXTI4_IRQHandler(void)
{
  EXTI->PR=0x10;
  char temp = 9;
  if ( pin_val(EXTI9_PIN) ) temp |=0x80;
  sendAnswer(0x1E,&temp, 1);

}

//#define EXTI7_PIN               pin_id(PORTD,6)         //Разъем EXTI7//
//#define EXTI8_PIN               pin_id(PORTD,7)         //Разъем EXTI8//
void EXTI9_5_IRQHandler(void)
{
  if (EXTI->PR&(1<<6))
  {
    EXTI->PR=(1<<6);
    char temp = 7;
    if ( pin_val(EXTI7_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<7))
  {
    EXTI->PR=(1<<7);
    char temp = 8;
    if ( pin_val(EXTI8_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }

}


//#define EXTI3_PIN               pin_id(PORTC,12)        //Разъем EXTI3//
//#define EXTI10_PIN              pin_id(PORTC,13)        //Разъем EXTI10//*/
//#define EXTI1_PIN               pin_id(PORTA,15)        //Разъем EXTI1///
void EXTI15_10_IRQHandler(void)
{
  if (EXTI->PR&(1<<12))
  {
    EXTI->PR=(1<<12);
    char temp = 3;
    if ( pin_val(EXTI3_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<13))
  {
    EXTI->PR=(1<<13);
    char temp = 10;
    if ( pin_val(EXTI10_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<15))
  {
      int_cnt++;
    EXTI->PR=(1<<15);
    char temp = 1;
    if ( pin_val(EXTI1_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }

}

////////////////////////////////////////////////////////////////////////////////
//___________________________________I2C______________________________________//
////////////////////////////////////////////////////////////////////////////////
void delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//___________________________________ADC______________________________________//
////////////////////////////////////////////////////////////////////////////////
void DMA2_Stream0_IRQHandler(void)
{
DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

}
////////////////////////////////////////////////////////////////////////////////
