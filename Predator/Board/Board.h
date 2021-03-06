#ifndef _BOARD_INCLUDED_
#define _BOARD_INCLUDED_

#include "stdint.h"

#define MAX_RPM 12400 // Maxon max rpm
#define REDUCTION 26 // Gearbox reduction
#define MAX_MAXON_PWM 0.9
#define MIN_MAXON_PWM 0.1

////////////////////ADC SONAR CHANNELS/////////////////////////////////
#define ADC_SONAR_RIGHT 2
#define ADC_SONAR_LEFT 3
#define ADC_SONAR_FRONT_1 4
#define ADC_SONAR_FRONT_2 5
#define ADC_SONAR_BACK 6

////////////////////SONAR INDEXES IN ARRAY////////////////////////////
#define SONAR_RIGHT 0
#define SONAR_LEFT 1
#define SONAR_FRONT_1 2
#define SONAR_FRONT_2 3
#define SONAR_BACK 4

////////////////////ADC IR CHANNELS/////////////////////////////////
#define ADC_IR_RIGHT 7
#define ADC_IR_LEFT 8
#define ADC_IR_FRONT 9
#define ADC_IR_BACK 10

////////////////////IR INDEXES IN ARRAY////////////////////////////
#define IR_RIGHT 0
#define IR_LEFT 1
#define IR_FRONT 2
#define IR_BACK 3


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
char setVoltage(char, float);
char setVoltageMaxon(char, int8_t, float);
char setSpeedMaxon(char, float);
char setPWM(char, float);

#endif
