#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include "waveforms.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

#define C4 261.6256
#define Db4 277.1826
#define D4 293.6648
#define Eb4 311.1270
#define E4 329.6276
#define F4 349.2282
#define Gb4 369.9944
#define G4 391.9954 
#define Ab4 415.3047 
#define A4 440.0000
#define Bb4 466.1638
#define B4 493.8833

#define OSCA 0
#define OSCB 1

#define SINE_WAVE 0
#define SQUARE_WAVE 1
#define TRIANGLE_WAVE 2
#define SAWTOOTH_WAVE 3
#define OSCILLATOR_OFF 4

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
        case SINE_WAVE:
            RED = 0;
            GREEN = 1023;
            BLUE = 1023;
            break;
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

volatile int tick = 0;
volatile int ledState = 0;
void SysTick_Handler(void) {
    // static int blinkCounter = 0;
    tick++;
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
    // We want a 1 kHz interrupt frequency and the clock is running at 4 MHz
    SysTick->LOAD = 39; // Set to for 1 us interval

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

static void delay10us(int delay10us) {
    int start = tick;
    while ((tick - start) < (delay10us * 10)) {
        // wait
    }
}

void TIM7_initialize(void) {
    // 1. Enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    __DSB();  // Ensure clock is enabled
    
    // 2. Reset timer
    TIM7->CR1 = 0;
    TIM7->PSC = 3999;  // Assuming 4MHz clock: 4000 ticks = 1ms
    TIM7->ARR = BLINK_INTERVAL_MS;
    
    // 3. Clear any pending interrupts
    TIM7->SR = 0;
    
    // 4. Enable interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    
    // 5. Configure NVIC
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    // 6. Start timer
    TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        printf("TIM7 interrupt!\r\n");
        printf("OSC value: %d\r\n", OSC);
        
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

void EXTI1_IRQHandler(void) {    
    printf("EXTI1 interrupt!\r\n");

    if(EXTI->PR1 & EXTI_PR1_PIF1) {
        
        CURRENT_WAVEFORM++;
        if (CURRENT_WAVEFORM > OSCILLATOR_OFF) {
            CURRENT_WAVEFORM = SINE_WAVE; // Reset to SINE_WAVE
        }
        LEDOutput(OSC, CURRENT_WAVEFORM); // Set LED color
        printf("Current waveform: %d\r\n", CURRENT_WAVEFORM);

        EXTI->PR1 |= EXTI_PR1_PIF1; // Clear interrupt flag

    }
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

int main(void) {
    // initializing SysTick
    SysTick_initialize();

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
    gpio_config_direction(D3, 0b00); // switch
    gpio_config_pullup(D3, 0b01); // switch

    // Configure the pins for the waveform button
    gpio_config_direction(D6, INPUT); // Button
    gpio_config_pullup(D6, PULL_UP);

    config_gpio_interrupt();
    
    // wait for serial port readiness
    // for (volatile int i = 0; i < 1000000; i++) { }
    printf("Hello, Audio World!\r\n");
      
    // initializing DAC using dacOutput
    DAC_init();
    int index = 0;
    float sineTable[WAVE_TABLE_SIZE];
    float squareTable[WAVE_TABLE_SIZE];
    float triangleTable[WAVE_TABLE_SIZE];
    float sawtoothTable[WAVE_TABLE_SIZE];

    buildWaveTable(sineTable);
    buildSquareWaveTable(squareTable);
    buildTriangleWaveTable(triangleTable);
    buildSawtoothWaveTable(sawtoothTable);
    
    float frequency = A4; // Set desired frequency
    float delay_us_value = (1.0 / frequency) / WAVE_TABLE_SIZE * 1e6; // Delay in microseconds
    int delay10us_value = (int)(delay_us_value / 10.0 + 0.5); // Convert to multiples of 10 us (round to nearest)

    printf("Frequency: %f Hz, Delay (us): %f, Delay10us: %d\r\n", frequency, delay_us_value, delay10us_value);
    LEDOutput(OSC, CURRENT_WAVEFORM); // Set LED color

    while (1) {
        if (CURRENT_WAVEFORM == SINE_WAVE) {
            DAC_setValue(sineTable[index]);
        }
        else if (CURRENT_WAVEFORM == SQUARE_WAVE) {
            DAC_setValue(squareTable[index]);
        }
        else if (CURRENT_WAVEFORM == TRIANGLE_WAVE) {
            DAC_setValue(triangleTable[index]);
        }
        else if (CURRENT_WAVEFORM == SAWTOOTH_WAVE) {
            DAC_setValue(sawtoothTable[index]);
        }
        else { // oscillator off
            DAC_setValue(0);
        }
        
        index++;
        if (index >= WAVE_TABLE_SIZE) {
            index = 0;  // Reset index to loop through the wave table
        }
        delay10us((int)delay_us_value / 10);  // Use the calculated delay
    }
}