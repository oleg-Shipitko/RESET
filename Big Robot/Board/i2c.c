#include "i2c.h"
#include "stm32f4xx.h"
#include "Extern_variables.h"

char i2cState = IDLE, i2cFlag, i2cCnt;
float encData[24];

////////////////////////////////////////////////////////////////////////////////
//_________________________________I2C________________________________________//
////////////////////////////////////////////////////////////////////////////////
void i2cStateMachine(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t count, char* st)
{
  char state = *st;
  
	switch (state)
	{
          case DO_START_TRANSFER:
            {
              state = WAIT_FOR_BUS; // Ждем автобус :)
              break;
            }
		
          case WAIT_FOR_BUS:
            {
              //if (((GPIOB->IDR)&0xC00) == (GPIO_IDR_IDR_10 | GPIO_IDR_IDR_11))
	      if (!((I2Cx->SR2)&I2C_SR2_BUSY))
                state = DO_START;
              break;
            }
          
          case DO_START:
            {
              // Send I2C1 START condition
              I2Cx->CR1 |= I2C_CR1_START;
	      	//if ((I2Cx->SR1)&I2C_SR1_SB)
              		state = WAIT_START_ACCEPTED;
//		else
//		  	state = IDLE;
              break;
            }
            
          case WAIT_START_ACCEPTED:
            {
              if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
                {
		  I2C_Send7bitAddress(I2Cx, address/*(address<<1)*/, direction);
                  state = WAIT_FOR_ADDR_ACK;
                }
//	      else
//	      	{
//			I2Cx->CR1 |= I2C_CR1_STOP;
//	      		state = IDLE;
//	      	}
              break;
            }
            
          case WAIT_FOR_ADDR_ACK:
            {
              if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
                {
                  state = RECEIVING_PACKET;
		  i2cCnt = 0;
		  I2Cx->CR1 |= I2C_CR1_ACK;
//                  break;
                }
              if (i2cFlag == AF)
                {
//                  I2Cx->CR1 |=  I2C_CR1_SWRST;//Сброс модуля и2ц
//                  I2Cx->CR1 &= ~I2C_CR1_SWRST;
//                  I2C_My_Init(I2Cx, 10000, I2C_Mode_I2C, I2C_DutyCycle_2, 0, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
                    I2Cx->CR1 |= I2C_CR1_STOP;
	      	    state = IDLE;
                  i2cFlag = 0;
                }
              break;
            }
            
          case RECEIVING_PACKET:
            {
              if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
                {
                  if (i2cCnt > count-1)
                    i2cCnt = count-1;
                  else
                    {
                      if (i2cCnt < count-2)
                        {
                          I2Cx->CR1 |= I2C_CR1_ACK; // enable acknowledge of received data
                          encData[i2cCnt] = I2C2->DR;//line5buf;
                        }
                      else
                        if (i2cCnt == count-2)
                          {
                            I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable the acknowledgement
                            encData[i2cCnt] = I2C2->DR;//line5buf;
                          }
                        else
                          if (i2cCnt == count-1)
                            {
                              I2Cx->CR1 |= I2C_CR1_STOP; // stop request
                              encData[i2cCnt] = I2C2->DR;//line5buf;
                              state = IDLE;
                              i2cCnt = 0;
                            }
//                          else
//                            line5Data[i2cCnt] = I2Cx->DR;
                    }
                  i2cCnt++; 
                }
              break;
            }
	}
	
  *st = state;
}
////////////////////////////////////////////////////////////////////////////////
void I2C_My_Init(I2C_TypeDef* I2Cx, uint32_t I2C_ClockSpeed, uint16_t I2C_Mode, 
		 uint16_t I2C_DutyCycle, uint16_t I2C_OwnAddress1, 
		 uint16_t I2C_Ack, uint16_t I2C_AcknowledgedAddress)
{
  uint16_t tmpreg = 0, freqrange = 0;
  uint16_t result = 0x04;
  uint32_t pclk1 = 8000000;
  RCC_ClocksTypeDef  rcc_clocks;
/*---------------------------- I2Cx CR2 Configuration ------------------------*/
  /* Get the I2Cx CR2 value */
  tmpreg = I2Cx->CR2;
  /* Clear frequency FREQ[5:0] bits */
  tmpreg &= (uint16_t)~((uint16_t)I2C_CR2_FREQ);
  /* Get pclk1 frequency value */
  RCC_GetClocksFreq(&rcc_clocks);
  pclk1 = rcc_clocks.PCLK1_Frequency;
  /* Set frequency bits depending on pclk1 value */
  freqrange = (uint16_t)(pclk1 / 1000000);
  tmpreg |= freqrange;
  /* Write to I2Cx CR2 */
  I2Cx->CR2 = tmpreg;
  
  //I2Cx->CR2 |= I2C_CR2_DMAEN;

/*---------------------------- I2Cx CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
  /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
  tmpreg = 0;
  
    /* Standard mode speed calculate */
    result = (uint16_t)(pclk1 / (I2C_ClockSpeed << 1));
    /* Test if CCR value is under 0x4*/
    if (result < 0x04)
    {
      /* Set minimum allowed value */
      result = 0x04;  
    }
    /* Set speed value for standard mode */
    tmpreg |= result;	  
    /* Set Maximum Rise Time for standard mode */
    I2Cx->TRISE = freqrange + 1; 
  

  /* Write to I2Cx CCR */
  I2Cx->CCR = tmpreg;
/*---------------------------- I2Cx CR1 Configuration ------------------------*/
  /* Get the I2Cx CR1 value */
  tmpreg = I2Cx->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
  tmpreg |= (uint16_t)((uint32_t)I2C_Mode | I2C_Ack);
  /* Write to I2Cx CR1 */
  I2Cx->CR1 = tmpreg;
/*---------------------------- I2Cx OAR1 Configuration -----------------------*/
  /* Set I2Cx Own Address1 and acknowledged address */
  I2Cx->OAR1 = (I2C_AcknowledgedAddress | I2C_OwnAddress1);
/*---------------------- Enable the selected I2C peripheral ------------------*/
  I2Cx->CR1 |= I2C_CR1_PE;
  
  // Interrupts
  I2Cx->CR2 |= I2C_IT_BUF | I2C_IT_AF | I2C_IT_BERR | I2C_IT_ERR /*| I2C_IT_EVT | I2C_IT_ADDR | I2C_IT_SB */| I2C_IT_BTF;
}
////////////////////////////////////////////////////////////////////////////////
char I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
char status = 0;
uint32_t lastFlag = 0;
  
  address = address << 1;
	// wait until I2C1 is not busy anymore
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  	while((GPIOB->IDR)&0xC00 == (GPIO_IDR_IDR_10 | GPIO_IDR_IDR_11));

	// Send I2C1 START condition
	I2Cx->CR1 |= I2C_CR1_START;

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	//while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	while (lastFlag != I2C_EVENT_MASTER_MODE_SELECT)
		{
		  	lastFlag = I2C_GetLastEvent(I2C2);
			if ((lastFlag & I2C_FLAG_AF) || (lastFlag & I2C_FLAG_BERR))
				{
			  		status = 0;
					break;
				}
			else
			  	status = 1;
		}
	  	

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
//	if(direction == I2C_Direction_Transmitter)
//		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	else 
//		if(direction == I2C_Direction_Receiver)
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			{
			  	if (((I2C2->SR1)&I2C_SR1_AF) || ((I2C2->SR1)&I2C_SR1_BERR))
			  		{
			    			status = 0;
					  	break;
			  		}
				else
				  	status = 1;
			}
return status;
}
////////////////////////////////////////////////////////////////////////////////
void I2C_SendByte(I2C_TypeDef* I2Cx, uint8_t Data)
{
  I2Cx->DR = Data;
}
////////////////////////////////////////////////////////////////////////////////
//void I2C_ReceivePacket(I2C_TypeDef* I2Cx, uint8_t count, uint8_t* buf)
//{  
//
//char i;
//    for (i = 0; i < count; i++) // 0-4
//    	{
//		// wait until one byte has been received
//		while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
//		if (i < 3) // 0,1,2
//			{
//				I2Cx->CR1 |= I2C_CR1_ACK; // enable acknowledge of received data
//				*(buf+i) = I2Cx->DR;
//			}
//		else
//			if (i == 3)
//			{
//				*(buf+i) = I2Cx->DR;	
//				I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);// disabe acknowledge of received data
//			}
//			else
//				if (i == 4)
//					{
//						I2Cx->CR1 |= I2C_CR1_STOP; // stop request
//						*(buf+i) = I2Cx->DR;
//					}
//				else
//					*(buf+i) = I2Cx->DR;
//	}
//}
////////////////////////////////////////////////////////////////////////////////
//void configI2C2_DMA(void)
//{
//  DMA1_Stream0->CR |= 1 << 25;        //Выбираем channel 4 
//  DMA1_Stream0->PAR |= (uint32_t) &(I2C2->DR);//Задаем адрес периферии - регистр результата преобразования АЦП для регулярных каналов.
//  DMA1_Stream0->M0AR |= (uint32_t) &I2C_buf; //Задаем адрес памяти - базовый адрес массива в RAM.
//  DMA1_Stream0->CR &= ~DMA_SxCR_DIR; //Направление передачи данных - чтение из периферии, запись в память.
//  DMA1_Stream0->NDTR = 1; //Количество пересылаемых значений
//  DMA1_Stream0->CR &= ~DMA_SxCR_PINC; //Адрес периферии не инкрементируется после каждой пересылки.
//  DMA1_Stream0->CR &= ~DMA_SxCR_MINC; //Адрес памяти не инкрементируется после каждой пересылки.
//  DMA1_Stream0->CR |= DMA_SxCR_CIRC; //Circular mode
//  DMA1_Stream0->CR |= DMA_SxCR_PL; //Приоритет
//  DMA1_Stream0->CR |= DMA_SxCR_TCIE; // прерывание в конце передачи
//
//  DMA1->LIFCR = DMA_LIFCR_CTCIF0;
//  DMA1_Stream0->CR |= DMA_SxCR_EN;
//  //NVIC_EnableIRQ(DMA1_Stream0_IRQn);   
//}
////////////////////////////////////////////////////////////////////////////////
//void GetI2CPacket(I2C_TypeDef* I2Cx, uint8_t address, uint8_t count, uint8_t* buf)
//{
//  I2Cx->CR1 |= I2C_CR1_ACK;  
//  		if (I2C_start(I2Cx, address, I2C_Direction_Receiver))
//			I2C_ReceivePacket(I2Cx, count, buf);
//		else	
//			{
//				I2Cx->CR1 |=  I2C_CR1_SWRST;//Сброс модуля и2ц
//				I2Cx->CR1 &= ~I2C_CR1_SWRST;
//			  	I2C_My_Init(I2Cx, 10000, I2C_Mode_I2C, I2C_DutyCycle_2, 0, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
//			}
//}
////////////////////////////////////////////////////////////////////////////////
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx)
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_MASK;

  /* Return status */
  return lastevent;
}
////////////////////////////////////////////////////////////////////////////////
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;
  ErrorStatus status = ERROR;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_EVENT(I2C_EVENT));

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_MASK;

  /* Check whether the last event contains the I2C_EVENT */
  if ((lastevent & I2C_EVENT) == I2C_EVENT)
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }
  /* Return status */
  return status;
}
////////////////////////////////////////////////////////////////////////////////
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_DIRECTION(I2C_Direction));
  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= I2C_OAR1_ADD0;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
  }
  /* Send the address */
  I2Cx->DR = Address;
}
////////////////////////////////////////////////////////////////////////////////
