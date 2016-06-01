#ifndef _USART_INCLUDED_
#define _USART_INCLUDED_

#include "stm32f4xx.h"


#define BufSize 200


typedef struct {
  int dmaStartByte;
  int dmaEndByte;
  char stDmaBusy ;
  char stDmainit ;
  int temp1;
  long int allCount;
  long int allDMA;
  int dmaCount;
  char * dmaAdr;
  int curWorkAdr;
} dmaPackStruct;




int putchar(char ch);  //отправить байт
uint32_t packetCheck(char* dataToCheck, char size);  // Вычислить контрольную сумму CRC
char sendPacket(char* dataToCheck, char size);     // отправить пакет
void usartSendByte(USART_TypeDef *USART,uint8_t byte);  //отправить байт
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void uartInit(USART_TypeDef* USARTx, uint32_t USART_BaudRate);//Инициализировать USART
void configUsart2DMA(char * usart2Data);
void configUsart3RXDMA(char * usart3Data, int count);

#endif
