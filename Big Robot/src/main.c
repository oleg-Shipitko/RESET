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
#include "Board.h"  //���� �������������

#include "gpio.h" // ������ � ������� �����-������
#include "Pins.h" // ����������� ����� �� �����
#include "Interrupts.h"
#include "regulator.h"  // ���������� �����, ����������, �����������

#include "usart.h" //����� � ������������� ��������
#include "robot.h"  //����������� ������������ ������ � ��� �������� �������
#include "Manipulators.h"

// ����� � �����������
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "Dynamixel_control.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char command = 0;

char mode;


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

       //����� ��������� ������������� ������� � ������ ����������� �����������
       command = 0;
       outEnc.adress = 0x02;
       outEnc.sync = 0xAA;
       outEnc.Command =  ENC_SET_CUR_POS;
       outEnc.checkSum = packetCheck((char *) &outEnc,sizeof(outEnc) - 2);
       sendPacket((char *) &outEnc,sizeof(outEnc));

//switchOnVibration();
//switchOnBelts();


  while(1)
   {
      char temp = pin_val (EXTI1_PIN);
      if (temp)
      {
        curState.trackEn = 1;
      } else
      {
        curState.trackEn = 0;
        vTargetGlob[0] = 0;
        vTargetGlob[1] = 0;
        vTargetGlob[2] = 0;
      }

      if (robotSpeed[0] > robotSpeed[1] )
      {
          if (robotSpeed[0] > 0)
          {
              distance[4] = adcData[4] * 0.0822 * 2.54;
          }
          else
          {
              distance[2] = adcData[2] * 0.0822 * 2.54;
          }
      }
      else
      {
          if (robotSpeed[1] > 0)
          {
              distance[1] = adcData[1] * 0.0822 * 2.54;
          }
          else
          {
              distance[3] = adcData[3] * 0.0822 * 2.54;
          }
      }

      if (robotSpeed[2] > 1.5)
      {
              distance[1] = adcData[1] * 0.0822 * 2.54;
              distance[2] = adcData[2] * 0.0822 * 2.54;
              distance[3] = adcData[3] * 0.0822 * 2.54;
              distance[4] = adcData[4] * 0.0822 * 2.54;
      }

      for (int i = 0; i < 5; i++)
      {
          if (distance[i] < 10)
          {
              curState.trackEn = 0;
              vTargetGlob[0] = 0;
              vTargetGlob[1] = 0;
              vTargetGlob[2] = 0;
          }
          else
          {
              curState.trackEn = 1;
          }
            distance[i] = 0;
      }
  }
}

// 2 right
//3 back
//4 left
//5 front
