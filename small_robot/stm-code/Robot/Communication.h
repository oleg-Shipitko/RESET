
#ifndef _COMMUNICATION_
#define _COMMUNICATION_

#include "robot.h"
#include "usbd_conf.h"

//структура данных робота

char param[30] ;                      //буфер параметров входящих команд
char inData[64];                      //Входной буфер данных
char outData[30];                     //Выходной буфер данных
char dataIndex;                       //счетчик количества байт во входящем пакете


void pushByte(char inByte);
char sendAnswer(char cmd, char * param, int paramSize);

#endif
