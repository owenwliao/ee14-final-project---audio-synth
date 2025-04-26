#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include "waveforms.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

#define OSCA 0
#define OSCB 1

#define SQUARE_WAVE 2
#define TRIANGLE_WAVE 3
#define SAWTOOTH_WAVE 4
#define OSCILLATOR_OFF 5

#define BLINK_INTERVAL_MS 500

// red = sine
// blue = square
// yellow = triangle
// green = saw
// white = off
// solid = OSCA
// blinking = OSCB

// can change
#define RED_PIN   A2
#define GREEN_PIN A0
#define BLUE_PIN  A1
#define TIMER     TIM2

// global for set_color()
int RED; // sine
int GREEN; // square
int BLUE; // triangle
int YELLOW; // saw
int WHITE;
int OSC = 1; // 0 for A, 1 for B
int CURRENT_WAVEFORM = 0;

void set_color(uint16_t red, uint16_t green, uint16_t blue) {
    timer_config_channel_pwm(TIMER, RED_PIN, red);
    timer_config_channel_pwm(TIMER, GREEN_PIN, green);
    timer_config_channel_pwm(TIMER, BLUE_PIN, blue);
}

void LEDOutput(int oscillator, int waveform) {
    int color;

    switch (waveform) {
        case SQUARE_WAVE:
            color = BLUE;
            RED = 1023;
            GREEN = 1023;
            BLUE = 0;
            break;
        case TRIANGLE_WAVE:
            color = GREEN;
            RED = 1023;
            GREEN = 0;
            BLUE = 1023;
            break;
        case SAWTOOTH_WAVE:
            color = YELLOW;
            RED = 0;
            GREEN = 0;
            BLUE = 1023;
            break;
        case OSCILLATOR_OFF:
            color = WHITE;
            RED = 0;
            GREEN = 0;
            BLUE = 0;
            break;
        default:
            color = WHITE; // default to white
            RED = 0;
            GREEN = 0;
            BLUE = 0;
            break;
    }

    OSC = oscillator;
    set_color(RED, GREEN, BLUE); 
}

volatile unsigned int tick = 0;
volatile int ledState = 0;

void SysTick_Handler(void) {
    // DAC_setValue(currentTablePointer[tick]); 
    tick++;
    // tick %= WAVE_TABLE_SIZE; // Wrap around the tick value
}

// retargeting USART2 -> printf
int _write(int file, char *ptr, int len) {
    serial_write(USART2, ptr, len);
    return len;
}

void SysTick_initialize(void) {
    // Disable the counter and reset all the settings
    SysTick->CTRL = 0;

    // Set how often the interrupt should fire
    // Clock runs at 4MHz
    SysTick->LOAD = 39;  

    // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
    // largest supported value (aka lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0; // Reset the current value
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Use the processor clock
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enable the interrupt
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable the counter
}

static void delay_us(int us) {
    int start = tick;
    while ((tick - start) < us) {
        // wait
    }
}

void TIM6_initialize(void) {
    // Enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    __DSB();  // Ensure clock is enabled
    
    // Reset timer
    TIM6->CR1 = 0;
    TIM6->ARR = 39;
    
    // Clear any pending interrupts
    TIM6->SR = 0;
    
    // Enable interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    
    // Configure NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    
    // Start timer
    TIM6->CR1 |= TIM_CR1_CEN;
}


void TIM7_initialize(void) {
    // Enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    __DSB();  // Ensure clock is enabled
    
    // Reset timer
    TIM7->CR1 = 0;
    TIM7->PSC = 3999;  // Assuming 4MHz clock: 4000 ticks = 1ms
    TIM7->ARR = BLINK_INTERVAL_MS;
    
    // Clear any pending interrupts
    TIM7->SR = 0;
    
    // Enable interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    
    // Configure NVIC
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    // Start timer
    TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        // printf("TIM7 interrupt!\r\n");
        // printf("OSC value: %d\r\n", OSC);
        
        if (OSC == OSCB) {  // Only blink if OSCB is active
            ledState = !ledState;
            if (ledState) {
                set_color(RED, GREEN, BLUE);  // On
            } else {
                set_color(1023, 1023, 1023);  // Off
            }
        }
    }
}

void EXTI0_IRQHandler(void) {
    printf("EXTI0 interrupt!\r\n");
    if (EXTI->PR1 & 1) {
        OSC = !OSC;  // Toggle OSC
        if (OSC == OSCA) {
            set_color(RED, GREEN, BLUE);
        }
        EXTI->PR1 |= 1;  // Clear interrupt flag
    }

}

void EXTI1_IRQHandler(void) {   // for switching waveforms and oscillators 
    
}

void TIM6_IRQHandler(void) {
    // DAC_setValue(currentTablePointer[tick]); // Set DAC value from the current waveform table
    // tick++;
    // tick %= WAVE_TABLE_SIZE; 
    // TIM6->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
}

void config_gpio_interrupt(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;
    SYSCFG->EXTICR[0] |= 1<<0;
    
    EXTI->FTSR1 |= (1<<0);
    EXTI->RTSR1 &= ~1;
    EXTI->IMR1 |= (1<<0);
    NVIC_SetPriority(EXTI0_IRQn, 2); // (IRQ number, priority)
    NVIC_EnableIRQ(EXTI0_IRQn);

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1; // Clear the EXTI1 bits
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; // Set EXTI1 to use port B (PB[1] = 001)

    EXTI->FTSR1 |= EXTI_FTSR1_FT1; // Set EXTI1 to trigger on falling edge
    EXTI->RTSR1 &= ~EXTI_RTSR1_RT1; // Disable rising edge trigger for EXTI1

    EXTI->IMR1 |= EXTI_IMR1_IM1; // Unmask EXTI1 

    NVIC_SetPriority(EXTI1_IRQn, 0); // Set the priority to 0
    NVIC_EnableIRQ(EXTI1_IRQn); // Enable the interrupt
}

void playNote(int note_delay) {
    DAC_setValue(0);
    delay_us(note_delay);
    DAC_setValue(4095);
    delay_us(note_delay);
}

int main(void) {
    // initializing UART
    host_serial_init();

    // Initialize TIM7
    TIM7_initialize();

    // Configure the timer for PWM
    timer_config_pwm(TIMER, 1000);

    // Configure the pins for PWM output
    gpio_config_alternate_function(RED_PIN, 1);
    gpio_config_alternate_function(GREEN_PIN, 1);
    gpio_config_alternate_function(BLUE_PIN, 1);

    // Configure the pins for the oscillator button
    gpio_config_mode(D3, 0b00); // switch
    gpio_config_pullup(D3, 0b01); // switch

    // Configure the pins for the waveform button
    gpio_config_mode(D6, INPUT); // Button
    gpio_config_pullup(D6, PULL_UP);

    config_gpio_interrupt();

    gpio_config_mode(D9, 0b00);
    gpio_config_mode(D10, 0b00);
    gpio_config_mode(D11, 0b00);

    gpio_config_pullup(D9, 0b10); // pulldown
    gpio_config_pullup(D10, 0b10);
    gpio_config_pullup(D11, 0b10);
          
    // initializing DAC using dacOutput
    DAC_init();
    
    LEDOutput(OSC, CURRENT_WAVEFORM); // Set LED color
    
    // initializing SysTick
    SysTick_initialize();

    while (1) {

        for (int i = 0; i < 128/2; i += 1) {
            int dacValue = (i*4095) / (128/2); // Scale to 12-bit value
            DAC_setValue(dacValue); // Set DAC value
            delay_us(1);
        }

        for (int i = 128/2; i >=0; i -= 1) {
            int dacValue = (i*4095) / (128/2); // Scale to 12-bit value
            DAC_setValue(dacValue); // Set DAC value
            delay_us(1);
        }

        // DAC_setValue(0);
        // delay_us(188/3);
        // DAC_setValue(2048/2);
        // delay_us(188/3);
        // DAC_setValue(2048);
        // delay_us(188/3);
        // DAC_setValue(4095);
        // delay_us(188/3);
        // DAC_setValue(2048);
        // delay_us(188/3);
        // DAC_setValue(2048/2);
        // delay_us(188/3);
        // playNote(C4_DELAY);
    }
}