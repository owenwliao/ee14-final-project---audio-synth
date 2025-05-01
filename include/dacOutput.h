#ifndef DAC_OUTPUT_H
#define DAC_OUTPUT_H

#include "stm32l432xx.h"

// DAC initialization
void DAC_init(void);

// set the DAC channels with a 12 bit value
void DAC_setValue1(uint16_t value);
void DAC_setValue2(uint16_t value);

#endif
