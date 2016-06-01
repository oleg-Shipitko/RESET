#ifndef _I2C_INCLUDED_
#define _I2C_INCLUDED_

#include "stm32f4xx.h"

#define CR1_CLEAR_MASK    ((uint16_t)0xFBF5)      /*<! I2C registers Masks */
#define FLAG_MASK         ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
#define ITEN_MASK         ((uint32_t)0x07000000)  /*<! I2C Interrupt Enable mask */

#define I2C_IT_SMBALERT                 ((uint32_t)0x01008000)
#define I2C_IT_TIMEOUT                  ((uint32_t)0x01004000)
#define I2C_IT_PECERR                   ((uint32_t)0x01001000)
#define I2C_IT_OVR                      ((uint32_t)0x01000800)
#define I2C_IT_AF                       ((uint32_t)0x01000400)
#define I2C_IT_ARLO                     ((uint32_t)0x01000200)
#define I2C_IT_BERR                     ((uint32_t)0x01000100)
#define I2C_IT_TXE                      ((uint32_t)0x06000080)
#define I2C_IT_RXNE                     ((uint32_t)0x06000040)
#define I2C_IT_STOPF                    ((uint32_t)0x02000010)
#define I2C_IT_ADD10                    ((uint32_t)0x02000008)
#define I2C_IT_BTF                      ((uint32_t)0x02000004)
#define I2C_IT_ADDR                     ((uint32_t)0x02000002)
#define I2C_IT_SB                       ((uint32_t)0x02000001)

#define I2C_IT_BUF                      ((uint16_t)0x0400)
#define I2C_IT_EVT                      ((uint16_t)0x0200)
#define I2C_IT_ERR                      ((uint16_t)0x0100)

// States
#define IDLE                            0x01
#define DO_START_TRANSFER               0x02
#define WAIT_FOR_BUS                    0x04
#define DO_START                        0x08
#define WAIT_START_ACCEPTED             0x10
#define WAIT_FOR_ADDR_ACK               0x20
#define RECEIVING_PACKET                0x40

// Flags
#define AF                              0x02

void I2C_My_Init(I2C_TypeDef* I2Cx, uint32_t I2C_ClockSpeed, uint16_t I2C_Mode, uint16_t I2C_DutyCycle, 
              uint16_t I2C_OwnAddress1, uint16_t I2C_Ack, uint16_t I2C_AcknowledgedAddress);
void I2C_SendByte(I2C_TypeDef* I2Cx, uint8_t Data);
void configI2C2_DMA(void);
void I2C_ReceivePacket(I2C_TypeDef* I2Cx, uint8_t count, uint8_t* buf);
char I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C2_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void GetI2CPacket(I2C_TypeDef* I2Cx, uint8_t address, uint8_t count, uint8_t* buf);
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void i2cStateMachine(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t count, char* st);

#endif
