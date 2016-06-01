#include "usart.h"
#include "Dynamixel_control.h"
#include "robot.h"

dmaPackStruct dmaData= {-1,-1,0,0,0,0,0,0,0,0};
char Buffer[BufSize];
uint8_t usart1Data[6];
encInPackStruct encData = {0xAA,0x01,0.0,0.0,0.0,0.0,0.0,0.0}; //входящие данные с энкодера

////////////////////////////////////////////////////////////////////////////////
//_________________________________USART______________________________________//
////////////////////////////////////////////////////////////////////////////////

char sendPacket(char* data, char size) // отправить пакет
{
  char state = 0;
  char i=0;

   for ( i = 0; i < (size ); i++)
        putchar(*(data + i));

  return state;
}

uint32_t packetCheck(char* dataToCheck, char size) //проверить пакет
{

 CRC->CR=1;
 char i;
 for ( i = 0; i < (size ); i++)
 CRC->DR = *(dataToCheck + i);

return CRC->DR;
}

void usartSendByte(USART_TypeDef *USART,uint8_t byte)
{
	while(!(USART->SR & USART_SR_TC));
	USART->DR = byte;
}
////////////////////////////////////////////////////////////////////////////////
void DMA1_Stream5_IRQHandler(void)     //????????????
{
  DMA1->HIFCR = DMA_HIFCR_CTCIF5;
}
////////////////////////////////////////////////////////////////////////////////
//void DMA1_Stream1_IRQHandler(void)
//{
//  DMA1->LIFCR = DMA_LIFCR_CTCIF1;
//}
////////////////////////////////////////////////////////////////////////////////
void uartInit(USART_TypeDef* USARTx, uint32_t USART_BaudRate)   // инициализация USART
{
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  RCC_ClocksTypeDef RCC_ClocksStatus;
/*---------------------------- USART BRR Configuration -----------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if ((USARTx == USART1) || (USARTx == USART6))
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  else
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    integerdivider = ((25 * apbclock) / (2 * USART_BaudRate ));
  else
    integerdivider = ((25 * apbclock) / (4 * USART_BaudRate ));
  tmpreg = (integerdivider / 100) << 4;
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  else
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);

  //USARTx->CR3 |= 0x40;   // DMA enable receiver
  USARTx->BRR = (uint16_t)tmpreg;
  USARTx->CR1 = USART_CR1_RXNEIE | USART_CR1_TE |  USART_CR1_RE |  USART_CR1_UE;
}
////////////////////////////////////////////////////////////////////////////////
void configUsart1TXDMA(char * usart1Data, int count)
{
  DMA2_Stream7->CR &=~ DMA_SxCR_EN;
  DMA2_Stream7->CR |= 4 << 25;        //Выбираем channel 4
  DMA2_Stream7->PAR = (uint32_t) &USART1->DR;//Задаем адрес периферии - регистр результата преобразования АЦП для регулярных каналов.
  DMA2_Stream7->M0AR = (uint32_t) usart1Data; //Задаем адрес памяти - базовый адрес массива в RAM.
  DMA2_Stream7->CR &= ~DMA_SxCR_DIR; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream7->CR |= DMA_SxCR_DIR_0; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream7->NDTR = count;
  DMA2_Stream7->CR &= ~DMA_SxCR_PINC; //Адрес периферии не инкрементируется после каждой пересылки.
  DMA2_Stream7->CR |= DMA_SxCR_MINC; //Адрес памяти инкрементируется после каждой пересылки.
  DMA2_Stream7->CR |= DMA_SxCR_PL; //Приоритет
  DMA2_Stream7->CR |= DMA_SxCR_TCIE; // прерывание в конце передачи
  USART1->SR&=~(USART_SR_TC);
  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
  DMA2_Stream7->CR |= DMA_SxCR_EN;
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}
////////////////////////////////////////////////////////////////////////////////
void configUsart1RXDMA(char * usart1Data, int count)
{
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  DMA2_Stream2->CR |= 4 << 25;        //Выбираем channel 4
  DMA2_Stream2->PAR = (uint32_t) &USART1->DR;//Задаем адрес периферии - регистр результата преобразования АЦП для регулярных каналов.
  DMA2_Stream2->M0AR = (uint32_t) usart1Data; //Задаем адрес памяти - базовый адрес массива в RAM.
  DMA2_Stream2->CR &= ~DMA_SxCR_DIR; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream2->NDTR = count; //Количество пересылаемых значений
  DMA2_Stream2->CR &= ~DMA_SxCR_PINC; //Адрес периферии не инкрементируется после каждой пересылки.
  DMA2_Stream2->CR |= DMA_SxCR_MINC; //Адрес памяти инкрементируется после каждой пересылки.
  DMA2_Stream2->CR |= DMA_SxCR_PL; //Приоритет
  DMA2_Stream2->CR |= DMA_SxCR_TCIE; // прерывание в конце передачи
  DMA2->LIFCR = DMA_LIFCR_CTCIF1;
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

////////////////////////////////////////////////////////////////////////////////


//
//______PUTCHAR_________________________________________________________________
//
int putchar(char ch) //добавить байт в буфер вывода
{
return 0; // заглушка за ненадобностью DMA
//curWorkAdr - текущее записываемое в буфер
//dmaStartByte -  последнее отправленное
//dmaEndByte  - текущее отправляемое

while ((dmaData.curWorkAdr>=dmaData.dmaStartByte)&&(dmaData.curWorkAdr<=dmaData.dmaEndByte));//h догнал сзади cur

dmaData.allCount++;
Buffer[dmaData.curWorkAdr]=ch;

if (++dmaData.curWorkAdr== BufSize) dmaData.curWorkAdr=0;
if (!(dmaData.stDmaBusy)) //если DMA свободен
{

  if (dmaData.curWorkAdr>dmaData.dmaEndByte)//заполнение буфера опережает последнюю отправку (h впереди cur)
  {
    if (dmaData.dmaEndByte==BufSize-1) dmaData.dmaEndByte=-1;
    dmaData.dmaStartByte = dmaData.dmaEndByte+1;
    dmaData.dmaEndByte =dmaData.curWorkAdr-1;
    dmaData.dmaCount=dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
    dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера

    dmaData.stDmaBusy = 1;    //DMA занят
    configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
    dmaData.allDMA+=dmaData.dmaCount;
    dmaData.stDmainit =1;

  }
  else //при переполнении буфера (cur впереди h)
  {
    if (dmaData.dmaEndByte==BufSize-1)
    {
      dmaData.dmaStartByte =  0;
      dmaData.dmaEndByte =dmaData.curWorkAdr-1;
      dmaData.dmaCount= dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
      dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
    }
    else
    {
      dmaData.dmaStartByte =  dmaData.dmaEndByte+1;
      dmaData.dmaEndByte =BufSize-1;
      dmaData.dmaCount= BufSize - dmaData.dmaEndByte+1;       // размер передаваемого буфера
      dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
    }

    dmaData.stDmaBusy = 1;    //DMA занят
    configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
    dmaData.stDmainit =2;
    dmaData.allDMA+=dmaData.dmaCount;

  }
}
return 0;
}



////////////////////////////////////////////////////////////////////////////////
//__________Прерывание_DMA1_____________________________________________________
//
void DMA1_Stream3_IRQHandler(void) // DMA USART2-TX
{
  int temp1=0;
   dmaData.stDmaBusy =0;

  if (DMA1->LISR & DMA_LISR_TCIF3) // если обмен закончен
  {

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR |= DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3;//очистить флаг окончания обмена.
    if (dmaData.curWorkAdr>dmaData.dmaEndByte) temp1 =dmaData.curWorkAdr-dmaData.dmaEndByte-1;
    else                       temp1 = dmaData.curWorkAdr-1 + BufSize-dmaData.dmaEndByte-1;

    if ((temp1>0))
    {
      if (dmaData.curWorkAdr>dmaData.dmaEndByte)//заполнение буфера опережает отправку (h впереди cur)
      {
        if (dmaData.dmaEndByte==BufSize-1) dmaData.dmaEndByte=-1;
        dmaData.dmaStartByte = dmaData.dmaEndByte+1;
        dmaData.dmaEndByte =dmaData.curWorkAdr-1;
        dmaData.dmaCount=dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
        dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера

        dmaData.stDmaBusy = 1;    //DMA занят

        configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
        dmaData.stDmainit =3;
        dmaData.allDMA+=dmaData.dmaCount;
        return;

      }
      else //при переполнении буфера (cur впереди h)
      {
        if (dmaData.dmaEndByte==BufSize-1)//99
        {
          dmaData.dmaStartByte =  0;
          dmaData.dmaEndByte =dmaData.curWorkAdr-1;
          dmaData.dmaCount= dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
          dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
        }
        else
        {
          dmaData.dmaStartByte =  dmaData.dmaEndByte+1;
          dmaData.dmaEndByte =BufSize-1;
          dmaData.dmaCount= dmaData.dmaEndByte - dmaData.dmaStartByte+1;       // размер передаваемого буфера
          dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
        }
        dmaData.stDmaBusy = 1;    //DMA занят
        dmaData.allDMA+=dmaData.dmaCount;
        configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
        dmaData.stDmainit =4;
        return;
      }
    }
    else
    {
      dmaData.stDmaBusy =0;
     return ;
    }
  }
  dmaData.stDmaBusy =0;
  return;
}

////////////////////////////////////////////////////////////////////////////////
//_________________________________USART______________________________________//
////////////////////////////////////////////////////////////////////////////////

void USART1_IRQHandler(void)
{
     char state;
	 state =USART1->SR ;
     USART1->SR =0;//&=~USART_SR_RXNE;
     if (state & USART_SR_RXNE || state & USART_SR_ORE)  //получен байт
     {
	 usart1Data[1] = USART1->DR;
	 pushByte(usart1Data[1]);
     state = USART1->SR;
     if (state & USART_SR_ORE)
     {
          usart1Data[1] = USART1->DR;
          pushByte(usart1Data[1]);
          state = USART1->SR;
     }
     if ((usart1Data[0] == 0xAA) && (usart1Data[1] == 0x01)) //проверка начала пакета
     {
         encData.adress = usart1Data[1];
         configUsart1RXDMA(((char *)&encData)+1, sizeof(encData)-1); // запуск DMA для приема основной части пакета
         USART1->CR1&=~USART_CR1_RXNEIE;
	     USART1->CR3|=USART_CR3_DMAR;
    }
    else usart1Data[0] = usart1Data[1];
    }
}

void DMA2_Stream2_IRQHandler(void)
{
  uint16_t checkSum;
  DMA2_Stream2->CR &=~ DMA_SxCR_EN;
 DMA2->LIFCR |= DMA_LIFCR_CTCIF1;

  USART1->CR1 |= USART_CR1_RXNEIE;
  checkSum  = packetCheck((char * )&encData,sizeof(encData)-2);
  if  (checkSum == encData.checkSum )  //Проверка CRC принятого пакета
  {
    robotCoord[0] = encData.robotCoord[0];
    robotCoord[1] = encData.robotCoord[1];
    robotCoord[2] = encData.robotCoord[2];
    robotSpeed[0] = encData.robotSpeed[1];
    robotSpeed[1] = encData.robotSpeed[0];
    robotSpeed[2] = encData.robotSpeed[2];

  }

}
