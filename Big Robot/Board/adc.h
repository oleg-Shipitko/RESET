#ifndef _ADC_INCLUDED_
#define _ADC_INCLUDED_



// Settings

#define ADC1_NUMB                       9 
#define ADC_SCAN_MODE                   ((uint32_t)0x0100)
#define ADC_CONT_MODE                   ((uint32_t)0x0002)
#define ADC_ON                          ((uint32_t)0x0001)
#define ADC_DATA_LEFT                   ((uint32_t)0x0800)
#define ADC_DMA_ENABLED                 ((uint32_t)0x0100)
#define VOLTS_PER_UNIT			0.012941

// Pins



void adcConfig();
float getLowDist(int value);

#endif
