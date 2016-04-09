
#ifndef _COMMUNICATION_
#define __COMMUNICATION_

#include "robot.h"
#include "usbd_conf.h"

char param[30] ;                      //буфер параметров входящих команд
char inData[64];                      //Входной буфер данных
char outData[30];                     //Выходной буфер данных
char dataIndex;                       //счетчик количества байт во входящем пакете
InPackStruct inCommand ={0xFA, 0xAF, 0x00, 0x00, &param[0]}; //структура входящего пакета

void pushByte(char inByte);
char sendAnswer(char cmd, char * param, int paramSize);

#endif
