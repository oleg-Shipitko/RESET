#ifndef _ROBOTS_INCLUDED_
#define _ROBOTS_INCLUDED_

#include "gpio.h"

#define WHEEL1_PWM_PIN BTN1_PWM_PIN    //              //TIM3 ch 3  AF2//
#define WHEEL2_PWM_PIN BTN2_PWM_PIN                    //TIM3 ch 4  AF2//
#define WHEEL3_PWM_PIN BTN3_PWM_PIN                    //TIM3 ch 1  AF2//
#define WHEEL4_PWM_PIN BTN4_PWM_PIN                    //TIM3 ch 2  AF2//

#define WHEEL1_DIR_PIN BTN1_DIR_PIN    //              //TIM3 ch 3  AF2//
#define WHEEL2_DIR_PIN BTN2_DIR_PIN                    //TIM3 ch 4  AF2//
#define WHEEL3_DIR_PIN BTN3_DIR_PIN                    //TIM3 ch 1  AF2//
#define WHEEL4_DIR_PIN BTN4_DIR_PIN                    //TIM3 ch 2  AF2//

#define WHEEL1_CH   0
#define WHEEL2_CH   1
#define WHEEL3_CH   2
#define WHEEL4_CH   3

#define ENC1A_PIN ENCODER1A_PIN         //������� 1 � TIM1//AF1
#define ENC1B_PIN ENCODER1A_PIN         //������� 1 B TIM1//AF1
#define ENC2A_PIN ENCODER2A_PIN         //������� 2 � TIM8//AF3
#define ENC2B_PIN ENCODER2A_PIN         //������� 2 B TIM8//AF3
#define ENC3A_PIN ENCODER3A_PIN         //������� 3 � TIM5//AF2
#define ENC3B_PIN ENCODER3A_PIN         //������� 3 B TIM5//AF2
#define ENC4A_PIN ENCODER4A_PIN         //������� 4 � TIM4//AF2
#define ENC4B_PIN ENCODER4A_PIN         //������� 4 B TIM4//AF2

#define SYNC_BYTE 0xFA
#define ADR_BYTE  0xAF
#define HEADER_SIZE  3
#define CHECK_SIZE   2

#define ADC_ANALOG_PIN 0
#define ADC_DIG_INPUT  1
#define ADC_DIG_OUTPUT 2

#define EXTI_BOTH       0
#define EXTI_RISE       1
#define EXTI_FALL       2
#define EXTI_DIG_INPUT  3
#define EXTI_DIG_OUTPUT 4

//��������� ������ ������������ �������
#pragma pack(push,1)
typedef struct {
  char sync;
  char adress;
  float robotSpeed[3];
  float robotCoord[3];
  uint16_t checkSum;
} encInPackStruct;

typedef struct {
  char sync;
  char adress;
  char Command;
  float robotCoord[4];
  uint16_t checkSum;

} encOutPackStruct;
#pragma pack(pop)

//��������� ������ ������
#pragma pack(push,1)
typedef struct {
  char sync;
  char adress;
  char command ;
  char * param;
} InPackStruct;

typedef struct {
  char sync;
  char adress;
  char command;
  float robotCoord[4];
  uint16_t checkSum;

} OutPackStruct;
#pragma pack(pop)
typedef struct {
  char pidEnabled;
  char trackEn;
  char kinemEn;
} robStateStruct;

extern float robotCoordTarget[3];
extern float robotSpeedTarget[3] ;
extern float motorSpeed[4];
extern float motorCoord[4] ;
extern float robotCoord[3] ;
extern float robotSpeed[3] ;
extern robStateStruct curState;
extern encOutPackStruct outEnc;
extern float vTargetGlob[3];

extern uint32_t  PWM_DIR[10];
extern uint32_t * PWM_CCR[10];
extern uint32_t * encCnt[4];
extern char  WHEELS[4];
extern uint16_t adcData[10];

extern char packLen[0x29];
extern InPackStruct inCommand;
extern char inData[64];
extern char dataIndex;


char setVoltage(char ch, float duty);
char execCommand(InPackStruct* cmd);
void pushByte(char inByte);
char sendAnswer(char cmd,char * param,int paramSize);

#endif
