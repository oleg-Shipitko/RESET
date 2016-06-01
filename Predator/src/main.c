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
#include "regulator.h"  // регуляторы колес, кинематика, траекторный

#include "usart.h" //обмен с измерительной тележкой
#include "robot.h"  //определение конфигурации робота и его основных функций
#include "Manipulators.h"

// обмен с компьютером
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "Dynamixel_control.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char command = 0;

char mode;

//char pwm_ch = 5;
//float dir = 1;
//float speed= 0.7;
//uint8_t number;

float temp;

int main(void)
{


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

openCubesCatcher();

  while(1)
   {
//      openCubesCatcher_2();
//      closeCubesCatcher_2();
//    moveCone();
//    softDelay(1000);
//    closeCone();
      char temp = pin_val (EXTI1_PIN);
      if (temp)
      {
        curState.trackEn = 1;
      }
      else
      {
        curState.trackEn = 0;
        vTargetGlob[0] = 0;
        vTargetGlob[1] = 0;
        vTargetGlob[2] = 0;
      }
     //openCubesCatcher();

//switchOnVibration();
//openWall();

//setSpeedMaxon(pwm_ch - 1, speed); // Maxons
//set_pin(PIN6_12V);

  }
}

