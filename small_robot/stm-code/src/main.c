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
#include "Board.h"  //файл инициализации

#include "gpio.h" // работа с портами ввода-вывода
#include "Pins.h" // определение ножек на плате
#include "Interrupts.h"
#include "regulator.h"  // регул€торы колес, кинематика, траекторный

#include "usart.h" //обмен с измерительной тележкой
#include "robot.h"  //определение конфигурации робота и его основных функций
#include "Manipulators.h"
// обмен с компьютером
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char mode;

int main(void)
{
    __disable_irq();
   initAll();


      USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#else
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

       //сброс координат измерительной тележки в случае перезапуска контроллера


//    __disable_irq();
//    conf_pin(EXTI2_PIN, INPUT, PUSH_PULL, FAST_S, PULL_UP);
__enable_irq();

     //   char * str ="mobile robot V1.0";
    //char ch = 5;
    //float duty = 0.09;
    //float duty1 = 0.07;// закрыто (ѕ–»“я√»¬ј≈ћ)
    //float duty2 = 0.024;// открыто(ќ“—ќ≈ƒ»Ќя≈ћ)
//int  test =  0;


/*InPackStruct test;
test.command=0x25;
char robotCoord1[3] = {0.1,0.,0};
test.param = *robotCoord1;*/
//265
//145
//pathPointStr robotCoord1 = {0.0, 0.0, 3.14, NULL,NULL,0,stopVel,stopRot,0,1};
//float robotCoord1[3] = {0.1,0.,0};
float temp[3] ={0.25,-0.25,0};
float anlge =270;
float torka = 1000;
uint16_t detector = 0;
//int ttime = 19000000;
//int ttime1 = 10000000;
//uint16_t  angle = 130;
while(1){



     //curState.collisionAvEn =1;
/*    setServoTorque(DOORS_ID , 1000);
    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
    setServoCWAngleLimit(DOORS_ID,(uint16_t) 0),
    setServoCCWAngleLimit(DOORS_ID,(uint16_t) 1023),
    setServoAngle(DOORS_ID,(uint16_t) angle);
    softDelay(9000000);
    //setServoMovingSpeed(DOORS_ID,0,0x000);
*/
    if (pin_val (EXTI2_PIN))
            {   //curState.pidEnabled=1;
                curState.trackEn = 1;}
        else
          {curState.trackEn = 0;
            vTargetGlob[0]=0;
            vTargetGlob[1]=0;
            vTargetGlob[2]=0;

            //curState.pidEnabled=0;
    }



    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
   /* setServoTorque((uint8_t)2,(uint16_t)1000);
    setServoCWAngleLimit((uint8_t) 2, 0);
    setServoCCWAngleLimit((uint8_t) 2, 0);
    setServoMovingSpeed((uint8_t)2,(uint16_t)1000,0x0400);
    softDelay(ttime );
    setServoMovingSpeed((uint8_t)2,(uint16_t)0,0x0000);


    setServoCWAngleLimit((uint8_t) 2, 0);
    setServoCCWAngleLimit((uint8_t) 2, 0);
    setServoMovingSpeed((uint8_t)2,(uint16_t)1000,0x0000);
    softDelay(ttime1 );
    setServoMovingSpeed((uint8_t)2,(uint16_t)0,0x0000);
*/


       if (pin_val(GENERAL_PIN_0))
             {set_pin (PIN6_12V);}
        else {reset_pin (PIN6_12V);
    }

//####################################################################################################################
//####################################################################################################################
//####################################################################################################################
                        //ловл€ рыбок -- 17-19секунд 1 заезд
}
    OpenFishingManipulator();
    TearFish();
    softDelay(5000000);
      lastPoint++;
      points[lastPoint].center[0] = temp[0];   // проехать до конца
      points[lastPoint].center[1] = temp[1];
      points[lastPoint].center[2] = temp[2];
      points[lastPoint].speedVelTipe = standVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);

      lastPoint++; //задезть в уголок(1)
      points[lastPoint].center[0] = 0.25-0.01;
      points[lastPoint].center[1] = -0.25-0.01;
      points[lastPoint].center[2] = -0.32;
      points[lastPoint].speedVelTipe = standVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(4500000);


      lastPoint++;//залезть в уголок(2)
      points[lastPoint].center[0] = 0.25-0.01+(0.86*0.05);
      points[lastPoint].center[1] = -0.25-0.01-(0.5*0.05);
      points[lastPoint].center[2] = -0.32;
      points[lastPoint].speedVelTipe = standVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);
      softDelay(4500000);




      HalfOpenFishingManipulator();
      softDelay(9000000);
      lastPoint++;  //объехать
      points[lastPoint].center[0] = 0.25-0.1;
      points[lastPoint].center[1] = -0.25-0.1;
      points[lastPoint].center[2] = -0.0;
      points[lastPoint].speedVelTipe = normalVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);





      lastPoint++;   // объехать 2
      points[lastPoint].center[0] = 0.25-0.1+0.2;
      points[lastPoint].center[1] = -0.25-0.1-0.2;
      points[lastPoint].center[2] = 0;
      points[lastPoint].speedVelTipe = normalVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);



      lastPoint++;  // подъехать к сетке
      points[lastPoint].center[0] = 0.25-0.1+0.2+(0.86*0.07);
      points[lastPoint].center[1] = -0.25-0.1-0.2+(0.5*0.07);
      points[lastPoint].center[2] = 0;
      points[lastPoint].speedVelTipe = standVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);


    UnTearFish();
    softDelay(9000000);
    CloseFishingManipulator();




      lastPoint++;  //объехать -2
      points[lastPoint].center[0] = 0.25-0.1;
      points[lastPoint].center[1] = -0.25-0.1;
      points[lastPoint].center[2] = -0.0;
      points[lastPoint].speedVelTipe = normalVelFast;
      points[lastPoint].speedRotTipe = normalRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;



      lastPoint++;  // - начальное положение
      points[lastPoint].center[0] = 0.14;
      points[lastPoint].center[1] = -0.01;
      points[lastPoint].center[2] = 0.0;
      points[lastPoint].speedVelTipe = standVelFast;
      points[lastPoint].speedRotTipe = stopRotFast;
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);


      robotCoord[0]= 0.0;
      robotCoord[1]= 0.0;
      robotCoord[2]= 0.0;
      points[0].center[0]= 0.0;
      points[0].center[1]= 0.0;
      points[0].center[2]= 0.0;

      CreatePath(&points[0], &points[0], &curPath);




    while (flag) {};
      flag = 1;
//########################################################################################################
//########################################################################################################
//########################################################################################################
   }

/*
  }
}


  // execCommand(test);
*/

        //setVoltage((char)CH_FISHIN_GSERVO - 1,(float) DUTY_FISH_CATCH);

/*    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ang1);
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR,(uint16_t) FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR,(uint16_t) ANG_CLOSE_FISHING_MANIPULATOR);  // з */
    // float test = !pin_val(GENERAL_PIN_0);


    // set_pin (PIN6_12V); // крутимс€
    //softDelay(10000000);
    //softDelay(10000000);
    //softDelay(3000000);

    //reset_pin (PIN6_12V); // не крутимс€

  //  conf_pin(EXTI1_PIN, ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
   // conf_pin(EXTI1_PIN, INPUT, PUSH_PULL, FAST_S, PULL_UP);
/*      test = pin_val (EXTI2_PIN);
    if (pin_val (EXTI2_PIN))
    {
    //    conf_pin(EXTI1_PIN, INPUT, PUSH_PULL, FAST_S, PULL_UP);
        __enable_irq();
    }




 setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_UNCATCH); // отсоедин€ем
    softDelay(9000000);
    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_UNCATCH+0.007); //0
    softDelay(9000000);
    softDelay(9000000);
    softDelay(9000000);
*/
   /* setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_CATCH);  //присоедин€ем
    softDelay(9000000);
    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_CATCH-0.007); //180 0.095
    softDelay(9000000);
    softDelay(9000000);
    softDelay(9000000); */

/*
set_pin (PIN6_12V); // крутимс€
    softDelay(10000000);
    softDelay(10000000);
    softDelay( 000000);

    reset_pin (PIN6_12V); // не крутимс€
*/


/*
    setServoTorque(ID_FISHING_MANIPULATOR, FISHING_MANIPULATOR_TORQUE);
    setServoAngle(ID_FISHING_MANIPULATOR, ANG_OPEN_FISHING_MANIPULATOR); // открыто - ловить1!!

    softDelay(9000000);
    softDelay(9000000);
    softDelay(9000000);

    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_UNCATCH); // отсоедин€ем
    softDelay(9000000);
    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_UNCATCH+0.007); //0
    softDelay(9000000);
    softDelay(9000000);
    softDelay(9000000);



    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_CATCH);  //присоедин€ем
    softDelay(9000000);
    setVoltage(CH_FISHIN_GSERVO - 1, DUTY_FISH_CATCH-0.007); //180 0.095
    softDelay(9000000);
    softDelay(9000000);
    softDelay(9000000);


      setServoAngle(ID_FISHING_MANIPULATOR, ANG_HALF_CLOSE_FISHING_MANIPULATOR);  // полузакрыто - тащить!
      softDelay(9000000);
      softDelay(9000000);
      softDelay(9000000);
*/
        //uint16_t ang1= 145;
        //setServoAngle(id_right, ang1);
      // uint16_t ang2= 180-3;
      //setServoAngle(id_right, ang2);  // закрыто - спр€тать
      // открыто - ловить1!!


    // robotCoord[0]= 0.2;
    // robotCoord[1]= 0.2;
    // robotCoord[2]= 0;
     //CreatePath(&points[0], &robotCoord[0], &curPath);


       //setServoReturnDelayMicros(id_right,(const uint16_t) 0 );
        //setServoTorque(id_right,(const uint16_t) 1000);
        //setServoCCWAngleLimit(id_right,(uint16_t ) 300);
        //setServoCWAngleLimit(id_right ,(uint16_t ) 0);
        //setServoMovingSpeed(id_right, (const uint16_t) 0 ,(const uint16_t) 0x0000);
        //setServoMovingSpeed(id_right, (const uint16_t) 0 ,(const uint16_t) 0x0400);
        // закрыто - спр€тать
        //setServoAngle(ID_RIGHT , (uint16_t)OPEN_ANG_RIGHT );
        //setServoAngle(ID_RIGHT , 100 );
        //setServoAngle(ID_RIGHT , 000 );
   // char ch = 4;
    //sendAnswer(1,str, strlen(str)+1);
    //setVoltage(ch - 1, (float)0.12);
    //setVoltage(ch - 1, (float)0.045);
 /*char ch = 5;
    //float duty1 = 0.07;// закрыто (ѕ–»“я√»¬ј≈ћ)
    //float duty2 = 0.024;// открыто(ќ“—ќ≈ƒ»Ќя≈ћ)

    //setVoltage(ch,duty1);
    setVoltage(ch - 1, duty1); //180 0.095
    soft_delay(9000000);
    setVoltage(ch - 1, duty1-0.004); //180 0.095
    soft_delay(9000000);
    soft_delay(9000000);
    /float duty2 = 0.024;// открыто(ќ“—ќ≈ƒ»Ќя≈ћ)
    setVoltage(ch - 1, duty2); //0
    soft_delay(3000000);
    setVoltage(ch - 1, duty2+0.002); //0ы
*/
