#include "robot.h"
#include "pins.h"
#include "usart.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "stm32fxxx_it.h"
#include "usbd_cdc_vcp.h"
#include "string.h"
#include "regulator.h"
#include "interrupts.h"
#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"


float robotCoordTarget[3] = {0,0,0}; // Целевые координаты робота в глоб сис-ме координат
float robotSpeedTarget[3] = {0,0,0}; // Целевые скорости робота в глоб сис-ме координат
float motorSpeed[4];                // скорости моторов
float motorCoord[4] = {0,0,0};      // общий пройденный колесом путь
float robotCoord[3] = {0,0,0};       // Координаты робота по показаниям измерительной тележки
float robotSpeed[3] = {0,0,0};       // скорость робота по показаниям измерительной тележки
robStateStruct curState = {1, 1, 1, 0};    // состояние регуляторов активен-1/неактвен -0
float distance[4] = {0,0,0,0};                      // расстояния по показаниям дальномеров

uint32_t * encCnt[4] ={ENCODER1_CNT, ENCODER2_CNT, ENCODER3_CNT, ENCODER4_CNT};  //массив указателей на счетчики энкодеров колес
char  WHEELS[4]= {WHEEL1_CH, WHEEL2_CH, WHEEL3_CH, WHEEL4_CH}; //каналы подкючения колес

//extern CDC_IF_Prop_TypeDef  APP_FOPS;

char execCommand(InPackStruct* cmd) //обработать входящую команду
{

switch(cmd->command)
{
  case 0x01: //Эхо
    {
     char *key=  cmd->param;

      if ((key[0] =='E')&&(key[1] =='C')&&(key[2] =='H')&&(key[3] =='O') )
      {
        char * str ="mobile robot V1.0";
        sendAnswer(cmd->command, str, strlen(str)+1);
        }
      }
  break;

  case 0x02:  //Установить текущие координаты
  {
      float *(temp) ={(float*)cmd->param};

      robotCoord[0]= temp[0];
      robotCoord[1]= temp[1];
      robotCoord[2]= temp[2];

      points[0].center[0]= temp[0];
      points[0].center[1]= temp[1];
      points[0].center[2]= temp[2];


      CreatePath(&points[0], &points[0], &curPath);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x03: //установить скважность шим
  {
      char  ch = *cmd->param;
      float  temp =*((float*)(cmd->param + 1));
      setPWM( ch - 1, temp);
      char * str ="Ok";
      sendAnswer(cmd->command, str, 3);

  }
  break;

  case 0x04:  //Установить бит направления
  {
      char * ch = cmd->param;
      set_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);

  }
  break;

  case 0x05:  //Снять бит направления
  {
      char * ch = cmd->param;
      reset_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x06:  //Установить напряжение на двигателе
  {
      char  ch = *cmd->param;
      float temp = *((float*)(cmd->param+1));
      setVoltage( ch-1, temp);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x08:  //Установить параметры регулятора
  {
      float *(temp) ={(float*)cmd->param};
      char i;
   for (i = 0; i<=3; i++)
  {
        wheelsPidStruct[i].p_k = temp[0];
        wheelsPidStruct[i].i_k = temp[1];
        wheelsPidStruct[i].d_k = temp[2];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x09:  //Установить требуюему скорость двигателей
  {
      float *(temp) ={(float*)cmd->param};
      char i;
   for (i = 0; i<=3; i++)
  {
  	regulatorOut[i] = temp[i];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0B:  //Включить рассчет кинематики
  {
      curState.kinemEn=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0C:  //Выключить рассчет кинематики
  {
      curState.kinemEn = 0;
      char * str = "Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0D:  //Задать скорости движения
  {
      float *(temp) ={(float*)cmd->param};
      char i;
   for (i = 0; i<=2; i++)
  {
        vTargetGlob[i] = temp[i];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0E:  //Включить траекторный регулятор
  {
      curState.trackEn=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0F:  //Выключить траекторный регулятор
  {
      curState.trackEn=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x10:  //Очистить очередь точек
  {
      while(lastPoint>0) removePoint(&points[0],&lastPoint);
      points[0].center[0]= robotCoord[0];
      points[0].center[1]= robotCoord[1];
      points[0].center[2]= robotCoord[2];

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x11:  //Добавить точку в очередь
  {

      float *(temp) ={(float*)(cmd->param)};
      char * ch = cmd->param + 12;
      lastPoint++;
      points[lastPoint].center[0] = temp[0];
      points[lastPoint].center[1] = temp[1];
      points[lastPoint].center[2] = temp[2];
      points[lastPoint].speedVelTipe = speedType[*ch];
      points[lastPoint].speedRotTipe = rotType[*(ch)];
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x12:  //Состояние очереди точек
  {
      char outdata[15];
      float * temp =(float*)(&outdata[3]);
      char * cntPoint  = (&outdata[0]);
      uint16_t * curPoint =  (uint16_t *)(&outdata[1]);
      *cntPoint= lastPoint;
      *curPoint = totalPointComplite;
      temp[0]= points[0].center[0];
      temp[1]= points[0].center[1];
      temp[2]= points[0].center[2];
      //char * str ="Ok";
      sendAnswer(cmd->command,outdata, 15);
  }
  break;

  case 0x13:  //отправить текущие координаты
  {

      sendAnswer(cmd->command,(char *)robotCoord, sizeof(robotCoord));
  }
  break;

  case 0x14:  //отправить текущую скорость
  {
      sendAnswer(cmd->command,(char *)robotSpeed, sizeof(robotCoord));
  }
  break;

  case 0x15:  //Задать скорость движения
  {
      float *(temp) ={(float*)(cmd->param)};
      char i;
      for (i = 0; i<=4; i++)
        normalVelFast[i]= temp[i];
      for (i = 0; i<=4; i++)
        stopVelFast[i]= temp[i];
      stopVelFast[2]=-0.2;

      char * str = "Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

   case 0x16:  //Установить режим ножки
   {

    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {
    pinType[ch] = *((char *)(cmd->param +1));
      if (pinType[ch] == ADC_ANALOG_PIN )
          conf_pin(GENERAL_PIN[ch], ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
      if (pinType[ch] == ADC_DIG_INPUT )
          conf_pin(GENERAL_PIN[ch], INPUT, PUSH_PULL, FAST_S, NO_PULL_UP);
      if (pinType[ch] == ADC_DIG_OUTPUT )
          conf_pin(GENERAL_PIN[ch], GENERAL, PUSH_PULL, FAST_S, NO_PULL_UP);

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
    }
  }
  break;

  case 0x17:  //отправить состояние выбранного входа АЦП
  {
      char ch = (*((char *)(cmd->param))) - 1;
    if (ch < 10)
      sendAnswer(cmd->command,(char *)&(adcData[ch]), sizeof(uint16_t));
  }
  break;

  case 0x18:  //отправить состояние всех АЦП
  {
      sendAnswer(cmd->command,(char *)adcData, sizeof(adcData));
  }
  break;

  case 0x19:  //отправить состояние входа
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {
      char temp =  (pin_val(GENERAL_PIN[ch])!=0);
      sendAnswer(cmd->command,&temp, sizeof(temp));
    }
  }
  break;

  case 0x1A:  //отправить состояние всех входов
  {
      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(GENERAL_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;

  case 0x1B:  //установить состояние выхода
  {
      char ch = (*((char *)(cmd->param))) -1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(GENERAL_PIN[ch]); else
                              set_pin(GENERAL_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x1C:  //отправить текущий режим ножки
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(pinType[ch]), sizeof(uint8_t));
  }
  break;

  case 0x1D:  //установить режим ножки EXTI
  {

    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {

      extiType[ch] = (*((char *)(cmd->param +1)));
      if (extiType[ch] == EXTI_BOTH )
      {
          conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch],  EXTI_BOTH_EDGES);
      }

      if (extiType[ch] == EXTI_RISE )
      {
          conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch], EXTI_RISING_EDGE);
      }
      if (extiType[ch] == EXTI_FALL )
      {
           conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch], EXTI_FALLING_EDGE) ;
      }
      if (extiType[ch] == EXTI_DIG_INPUT )
      {
        conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, NO_PULL_UP);
        clear_ext_interrupt(EXTI_PIN[ch]) ;
      }

      if (extiType[ch] == EXTI_DIG_OUTPUT )
      {
        conf_pin(EXTI_PIN[ch], GENERAL, PUSH_PULL, FAST_S, NO_PULL_UP);
        clear_ext_interrupt(EXTI_PIN[ch]) ;

      }

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
    }
  }
  break;

  case 0x1E:  //отправить состояние входа
  {
    char ch = *((char *)(cmd->param))-1;
    if ((ch)<10)
    {
      char temp = ch;
      if (pin_val(EXTI_PIN[ch])) temp |=0x80;
      sendAnswer(cmd->command,&temp, sizeof(temp));
    }
  }
  break;

  case 0x1F:  //отправить состояние всех входов
  {

      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(EXTI_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;

  case 0x20:  //установить состояние выхода
  {
       char ch = *((char *)(cmd->param))-1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(EXTI_PIN[ch]); else
                              set_pin(EXTI_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x21:  //отправить текущий режим ножки
  {
    char ch = *((char *)(cmd->param))-1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(extiType[ch]), sizeof(uint8_t));
  }
  break;

  case 0x22:  //установить состояние выхода +12В
  {
      char ch = (*((char *)(cmd->param)))-1;
      if (ch<6)
      {

       if (*(cmd->param+1)==0)
            reset_pin(V12_PIN[ch]); else
                              set_pin(V12_PIN[ch]);
      }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x23:  //Выключить ПИД регуляторы приводов
  {
      curState.pidEnabled=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x24:  //Включить ПИД регуляторы приводов
  {
      curState.pidEnabled=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x25:    // set current coordinate
  {
      float *(temp) ={(float*)cmd->param};
      robotCoord[0]= temp[0];
      robotCoord[1]= temp[1];
      robotCoord[2]= temp[2];
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }

  case 0x26:  //set dynamixel angle
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(angle) ={(uint16_t*)(cmd->param + 1)};
      if (setServoMovingSpeed(ID, angle))
      {
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x27:  //set CW angle limit
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(limit) ={(uint16_t*)(cmd->param + 1)};
      if (setServoCWAngleLimit(ID, limit))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x28:  //set CCW angle limit
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(limit) ={(uint16_t*)(cmd->param + 1)};
      if (setServoCCWAngleLimit(ID, limit))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x29:  //set servo moving speed
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(speed) ={(uint16_t*)(cmd->param + 1)};
      uint16_t *(direction) ={(uint16_t*)(cmd->param + 3)};
      if (setServoMovingSpeed(ID, speed, direction))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x2A:  //add a point to the beginning of Queue
  {
      float *(temp) = (float*)(cmd->param);
      char * ch = cmd->param + 12;
      addPointInFrontOfQueue(&points[0], &temp[0], &ch, &lastPoint);
      CreatePath(&points[0], &robotCoord[0], &curPath);

      char * str ="Ok";
      sendAnswer(cmd->command, str, 3);
  }
  break;

  case 0x2B:  //Open Cubes Catcher
  {
      openCubesCatcher();

      char * str ="Ok";
      sendAnswer(cmd->command, str, 3);
  }
  break;

  case 0x2C:  //Close Cubes Catcher
  {
      uint8_t numberOfCubesCatched;
      closeCubesCatcher(&numberOfCubesCatched);

      sendAnswer(cmd->command, numberOfCubesCatched, sizeof(uint8_t));
  }
  break;

//  case 0x2D:  // Open cubes movers
//  {
//      OpenCubesMovers();
//
//      char * str ="Ok";
//      sendAnswer(cmd->command,str, 3);
//  }
//  break;
//
//  case 0x2E:  // Close  cubes movers
//  {
//      CloseCubesMovers();
//
//      char * str ="Ok";
//      sendAnswer(cmd->command,str, 3);
//  }
//  break;

  case 0x2F:  // Switch On the vibration
  {
      switchOnVibration();

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x30:  // Switch Off the vibration
  {
      switchOffVibration();

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x31:  // Set angle of cubes catcher
  {
      float *(temp) = (float*)(cmd->param);
      cubesCatcherPID.target = *temp;

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

//  case 0x32:  // Switch On the belts
//  {
//      switchOnBelts();
//
//      char * str ="Ok";
//      sendAnswer(cmd->command,str, 3);
//  }
//  break;
//
//  case 0x33:  // Switch Off the belts
//  {
//      switchOffBelts();
//
//      char * str ="Ok";
//      sendAnswer(cmd->command,str, 3);
//  }
//  break;

  case 0x34:  // Starting command
  {
      if (pin_val (EXTI2_PIN))
      {
        char * str = "1";
        sendAnswer(cmd->command,str, 2);
        __enable_irq();

      }
      else
      {
        char * str = "0";
        sendAnswer(cmd->command,str, 2);
      }

  }
  break;

  default:
  break;
}

return 0;
}

