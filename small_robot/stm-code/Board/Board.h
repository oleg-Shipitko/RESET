#ifndef _BOARD_INCLUDED_
#define _BOARD_INCLUDED_

#include "stdint.h"

char setVoltage(char ch, float duty); // установить напряжение на выходе управления двигателем -1,0 .. 1,0

extern uint32_t * PWM_CCR[10];  //регистры сравнения каналов ШИМ
extern uint32_t  PWM_DIR[10];
extern uint32_t  GENERAL_PIN[10];
extern uint32_t  EXTI_PIN[10];
extern uint32_t  V12_PIN[6];

extern uint16_t adcData[10];
extern uint8_t pinType[10];
extern uint8_t extiType[10];
extern uint16_t extiFlag;

void initAll(void); // That's all!
void clear_ext_interrupt(unsigned char pin);
void add_ext_interrupt(unsigned char pin, char edge);
#endif
