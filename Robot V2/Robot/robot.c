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
#include "init.h"


float robotCoordTarget[3] = {0,0,0}; // ������� ���������� ������ � ���� ���-�� ���������
float robotSpeedTarget[3] = {0,0,0}; // ������� �������� ������ � ���� ���-�� ���������
float motorSpeed[4];                // �������� �������
float motorCoord[4] = {0,0,0};      // ����� ���������� ������� ����
float robotCoord[3] = {0,0,0};       // ���������� ������ �� ���������� ������������� �������
float robotSpeed[3] = {0,0,0};       // �������� ������ �� ���������� ������������� �������
robStateStruct curState = {1,1,1};    // ��������� ����������� �������-1/�������� -0
encOutPackStruct outEnc;              //����� ������ ������������ ������������� �������

char param[30] ;                      //����� ���������� �������� ������
char inData[64];                      //������� ����� ������
char outData[30];                     //�������� ����� ������
char dataIndex;                       //������� ���������� ���� �� �������� ������
InPackStruct inCommand ={0xFA,0xAF,0x00,0x00,&param[0]}; //��������� ��������� ������

char answerPackLen[] = {23,8,8,8, // 01,02,03,04
                    8,8,0,8,    // 05,06,07,08
                    8,0,8,8,     // 09,0a,0b,0c
                    8,8,8,8,     // 0d,0e,0f,10
                    8,21,18,18,     // 11,12,13,14
                    8,8,8,26,     // 15,16,17,18
                    7,16,8,7,     // 19,1a,1b,1c
                    8,7,16,8,     // 1d,1e,1f,20
                    7,8,8,8           // 21,22
                    }; //������ ��������� ������� � ����������� �� ������ �������

uint32_t * PWM_CCR[10] ={BTN1_CCR,BTN2_CCR,BTN3_CCR,BTN4_CCR,BTN5_CCR,
                          BTN6_CCR,BTN7_CCR,BTN8_CCR,BTN9_CCR,BTN10_CCR};  //�������� ��������� ������� ���
uint32_t  PWM_DIR[10] ={BTN1_DIR_PIN,BTN2_DIR_PIN,
                          BTN3_DIR_PIN,BTN4_DIR_PIN,
                          BTN5_DIR_PIN,BTN6_DIR_PIN,
                          BTN7_DIR_PIN,BTN8_DIR_PIN,
                          BTN9_DIR_PIN,BTN10_DIR_PIN};
uint32_t  GENERAL_PIN[10] ={GENERAL_PIN_0,GENERAL_PIN_1,
                            GENERAL_PIN_2,GENERAL_PIN_3,
                            GENERAL_PIN_4,GENERAL_PIN_5,
                            GENERAL_PIN_6,GENERAL_PIN_7,
                            GENERAL_PIN_8,GENERAL_PIN_9};
uint32_t  EXTI_PIN[10] ={EXTI1_PIN,EXTI2_PIN,
                         EXTI3_PIN,EXTI4_PIN,
                         EXTI5_PIN,EXTI6_PIN,
                         EXTI7_PIN,EXTI8_PIN,
                         EXTI9_PIN,EXTI10_PIN};
uint32_t  V12_PIN[6] ={PIN5_12V,PIN6_12V,
                            PIN3_12V,PIN4_12V,
                            PIN5_12V,PIN6_12V};

uint32_t * encCnt[4] ={ENCODER4_CNT,ENCODER3_CNT, ENCODER1_CNT,ENCODER2_CNT};  //������ ���������� �� �������� ��������� �����

char  WHEELS[4]= {WHEEL1_CH,WHEEL2_CH,WHEEL3_CH,WHEEL4_CH}; //������ ���������� �����

uint16_t adcData[10];
uint8_t pinType[10];
uint8_t extiType[10];
uint16_t extiFlag;

 char * str ="Ok";

extern CDC_IF_Prop_TypeDef  APP_FOPS;

char setVoltage(char ch, float duty) // ���������� ���������� �� ������ ���������� ���������� -1,0 .. 1,0
{
    if (duty>1 )duty=1;
    if (duty<-1 )duty=-1;

    if (duty < 0)
    {
          *PWM_CCR[ch] = (int32_t)(MAX_PWM +  (duty*MAX_PWM));
          set_pin(PWM_DIR[ch]);
    }
  else
    {
          *PWM_CCR[ch] = (int32_t) (duty*MAX_PWM);
          reset_pin(PWM_DIR[ch]);
    }
    return 0;
}

char setPWM(char ch, float duty) // ���������� ���������� �� ������ ���  0 .. 1,0
{
    if (duty>1 )duty=1;
    if (duty<0 )duty=0;
    *PWM_CCR[ch] = (int32_t)((duty*MAX_PWM));
    return 0;
}

void pushByte(char inByte) // �����, ������������ � �������� ��������� ������ � ������ ������
{
  char j;
  uint16_t checkSum;
  uint16_t * test;
  inData[dataIndex++] = inByte;

  if((inData[0] == SYNC_BYTE) && (inData[1] == ADR_BYTE))  //����� ���������
  {
    if( (dataIndex >= inData[2]) && (dataIndex > 3) ) //�������� ������ ������
    {
      checkSum = packetCheck(&inData[0], inData[2] - CHECK_SIZE);
      test = ( uint16_t *) &inData[inData[2] - CHECK_SIZE];
      if (*test == checkSum) // �������� CRC
      {
        inCommand.packLen = inData[2];
        for (j=0; j < inCommand.packLen - CHECK_SIZE - HEADER_SIZE; j++)  //����������� ����������
                      *(inCommand.param + j) = inData[4 + j];
        inCommand.command = inData[3];
        execCommand(&inCommand);     //���������� �������
      }
      dataIndex = 0;
      inData[0] = 0;
      inData[1] = 0;
    }
  }
  else
  {
    if (dataIndex > 1)
    {
      inData[0] = inData[1];
      inData[1] = 0;
      dataIndex = 1;
    }
  }
}

extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

char sendAnswer(char cmd, char * param, int paramSize) // ��������� ����� �� USB
{
         //    __disable_irq();
         outData[0] = 0xFA;
         outData[1] = 0xFA;
         outData[2] = answerPackLen[cmd - 1];
         outData[3] = cmd;
         memcpy(&outData[4], param, paramSize);

         *((int16_t*)&outData[paramSize + HEADER_SIZE]) = (int16_t) packetCheck(&outData[0], paramSize + HEADER_SIZE);
         int _size = paramSize + HEADER_SIZE + CHECK_SIZE  ;
         int i;
         for (i=0; i < _size; i++) putchar(outData[i]);

         if (APP_Rx_ptr_in + _size < APP_RX_DATA_SIZE)
         {
            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], outData, _size);
            APP_Rx_ptr_in += _size;
         }
         else
         {
            int freeSpace = APP_RX_DATA_SIZE - APP_Rx_ptr_in;

            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], outData, freeSpace);
            APP_Rx_ptr_in = 0;
            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], &outData[freeSpace], _size - freeSpace);
            APP_Rx_ptr_in += _size - freeSpace;
         }
         //     APP_FOPS.pIf_DataTx((uint8_t*)outData,
         //             paramSize+HEADER_SIZE+CHECK_SIZE);

         // __enable_irq();
         return paramSize + HEADER_SIZE + CHECK_SIZE;
}

char execCommand(InPackStruct* cmd) //���������� �������� �������
{

switch(cmd->command)
{
  case 0x01: //���
    {
     char *key=  cmd->param;

      if ((key[0] =='E')&&(key[1] =='C')&&(key[2] =='H')&&(key[3] =='O') )
      {
        char * str ="mobile robot V1.0";
        sendAnswer(cmd->command, str, strlen(str)+1);
        }
      }
  break;
  case 0x02:  //���������� ������� ����������
  {
      float *(temp) ={(float*)cmd->param};
      robotCoord[0]= temp[0];
      robotCoord[1]= temp[1];
      robotCoord[2]= temp[2];
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
       outEnc.adress=0x02;
       outEnc.sync = 0xAA;
       outEnc.Command =  ENC_SET_CUR_POS;
       outEnc.robotCoord[0]=0;
       outEnc.robotCoord[1]=0;
       outEnc.robotCoord[2]=0;
       outEnc.checkSum = packetCheck((char *) &outEnc,sizeof(outEnc)-2);
       sendPacket((char *) &outEnc,sizeof(outEnc));

  }
  break;
  case 0x03: //���������� ���������� ���
  {
      char  ch = *cmd->param;
      float  temp =*((float*)(cmd->param+1));
      setPWM( ch-1, temp);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);

  }
  break;
  case 0x04:  //���������� ��� �����������
  {
      char * ch = cmd->param;
      set_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);

  }
  break;
  case 0x05:  //����� ��� �����������
  {
      char * ch = cmd->param;
      reset_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x06:  //���������� ���������� �� ���������
  {
      char  ch = *cmd->param;
      float temp = *((float*)(cmd->param+1));
      setVoltage( ch-1, temp);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x08:  //���������� ��������� ����������
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
  case 0x09:  //���������� ��������� �������� ����������
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
  case 0x0B:  //�������� ������� ����������
  {
      curState.kinemEn=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x0C:  //��������� ������� ����������
  {
      curState.kinemEn=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
    case 0x0D:  //������ �������� ��������
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

    case 0x0E:  //�������� ����������� ���������
  {
      curState.trackEn=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x0F:  //��������� ����������� ���������
  {
      curState.trackEn=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x10:  //�������� ���� �����
  {
      while(lastPoint>0) removePoint(&points[0],&lastPoint);
      points[0].center[0]= robotCoord[0];
      points[0].center[1]= robotCoord[1];
      points[0].center[2]= robotCoord[2];

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
    case 0x11:  //�������� ����� � ����
  {

      float *(temp) ={(float*)(cmd->param)};
      char * ch = cmd->param+12;
      char * str ="Ok";
      lastPoint++;
      points[lastPoint].center[0] = temp[0];
      points[lastPoint].center[1] = temp[1];
      points[lastPoint].center[2] = temp[2];
      points[lastPoint].speedVelTipe = speedType[*ch];
      points[lastPoint].speedRotTipe = speedType[*(ch)];
      points[lastPoint].endTask=NULL;
      points[lastPoint].movTask =NULL;
      sendAnswer(cmd->command,str, 3);
  }
  break;
    case 0x12:  //��������� ����� �����
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

  case 0x13:  //��������� ������� ����������
  {

      sendAnswer(cmd->command,(char *)robotCoord, sizeof(robotCoord) + 1);
  }
  break;
    case 0x14:  //��������� ������� ��������
  {


      sendAnswer(cmd->command,(char *)robotSpeed, sizeof(robotCoord) + 1);
  }
  break;
  case 0x15:  //������ �������� ��������
  {
      float *(temp) ={(float*)(cmd->param)};
      char i;
      for (i = 0; i<=4; i++)
        normalVel[i]= temp[i];
     for (i = 0; i<=4; i++)
        stopVel[i]= temp[i];
         stopVel[2]=-0.2;
      char * str ="Ok";


      sendAnswer(cmd->command,str, 3);
  }
   break;
     case 0x16:  //���������� ����� �����
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
     case 0x17:  //��������� ��������� ���������� ����� ���
  {
      char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
      sendAnswer(cmd->command,(char *)&(adcData[ch]), sizeof(uint16_t));
  }
  break;
   case 0x18:  //��������� ��������� ���� ���
  {
      sendAnswer(cmd->command,(char *)adcData, sizeof(adcData));
  }
  break;
  case 0x19:  //��������� ��������� �����
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {
      char temp =  (pin_val(GENERAL_PIN[ch])!=0);
      sendAnswer(cmd->command,&temp, sizeof(temp));
    }
  }
  break;
   case 0x1A:  //��������� ��������� ���� ������
  {
      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(GENERAL_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;
  case 0x1B:  //���������� ��������� ������
  {
      char ch = (*((char *)(cmd->param))) -1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(GENERAL_PIN[ch]); else
                              set_pin(GENERAL_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x1C:  //��������� ������� ����� �����
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(pinType[ch]), sizeof(uint8_t));
  }
  break;

  case 0x1D:  //���������� ����� ����� EXTI
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
  case 0x1E:  //��������� ��������� �����
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
   case 0x1F:  //��������� ��������� ���� ������
  {

      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(EXTI_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;
  case 0x20:  //���������� ��������� ������
  {
       char ch = *((char *)(cmd->param))-1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(EXTI_PIN[ch]); else
                              set_pin(EXTI_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x21:  //��������� ������� ����� �����
  {
    char ch = *((char *)(cmd->param))-1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(extiType[ch]), sizeof(uint8_t));
  }
  break;
  case 0x22:  //���������� ��������� ������ +12�
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
  case 0x23:  //��������� ��� ���������� ��������
  {
      curState.pidEnabled=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;
  case 0x24:  //�������� ��� ���������� ��������
  {
      curState.pidEnabled=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

default:

  break;
}
return 0;
}

