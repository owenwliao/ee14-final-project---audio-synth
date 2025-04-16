// complementary file for dacOutput.c
#ifndef DAC_OUTPUT_H
#define DAC_OUTPUT_H

#include "stm32l432xx.h"

// DAC initialization
void DAC_init(void);

// set the Dac with a 12 bit value
void DAC_setValue(uint16_t value);

#endif
