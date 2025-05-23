#include "dacOutput.h"
#include "stm32l432xx.h"

// initialize the dac peripheral
void DAC_init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // enable clock for GPIOA
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;   // enable clock for DAC1

    volatile uint32_t tmp = RCC->APB1ENR1; // synchronizes clock and prevents compiler from removing register read

    // dac channel 1 - set up pin PA4 (A3)
    GPIOA->MODER &= ~(3UL << (4 * 2));           // clear mode bits for pa4
    GPIOA->MODER |= (3UL << (4 * 2));            // set to (11)
    GPIOA->PUPDR &= ~(3U << (4 * 2));       // No pull resistor

    DAC->MCR &= ~DAC_MCR_MODE1; // Clear mode bits
    DAC->MCR |= (0b010 << DAC_MCR_MODE1_Pos);
    
    // dac channel 2 - set up pin PA5 (A4)
    GPIOA->MODER &= ~(3UL << (5 * 2)); // clear mode bits for pa5
    GPIOA->MODER |= (3UL << (5 * 2));            // set to (11)
    GPIOA->PUPDR &= ~(3U << (5 * 2));       // No pull resistor

    DAC->MCR &= ~DAC_MCR_MODE2; // Clear mode bits for channel 2
    DAC->MCR |= (0b010 << DAC_MCR_MODE2_Pos);
    
    DAC->CR &= ~(0b111 << 3); // set DAC channel 1 trigger to TIM6
    DAC->CR &= ~(0b111 << 19); // set DAC channel 2 trigger to TIM6

    // enable DAC channels
    DAC->CR |= DAC_CR_EN1;  
    DAC->CR |= DAC_CR_EN2;
}

// write a 12-bit value to the dac channel 1 and 2
void DAC_setValue1(uint16_t value) {
    DAC->DHR12R1 = value;
}

void DAC_setValue2(uint16_t value) {
    DAC->DHR12R2 = value;
}