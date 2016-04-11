
#ifndef _COMMUNICATION_
#define _COMMUNICATION_

#include "usbd_conf.h"

char param[30] ;                      //буфер параметров входящих команд
char inData[64];                      //Входной буфер данных
char outData[30];                     //Выходной буфер данных
char dataIndex;                       //счетчик количества байт во входящем пакете

//структура данных робота
#pragma pack(push,1)
typedef struct {
  char sync;
  char adress;
  char packLen;
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

void pushByte(char inByte);
char sendAnswer(char cmd, char * param, int paramSize);


#endif
