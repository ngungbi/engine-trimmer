#ifndef _ADC_
#define _ADC_

#include <stdint.h>

void ADC_Init(void);
uint16_t ADC_Read(uint8_t ch);

#endif