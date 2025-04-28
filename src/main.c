#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include "waveforms.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

#define OSCA 0
#define OSCB 1

#define SQUARE_WAVE 0
#define TRIANGLE_WAVE 1
#define SAWTOOTH_WAVE 2
#define OSCILLATOR_OFF 3

#define BLINK_INTERVAL_MS 500

#define RED_PIN   A2
#define GREEN_PIN A0
#define BLUE_PIN  A1
#define TIMER     TIM2

#define C       D1
#define Db      D0
#define D       D2
#define Eb      D4
#define E       D9
#define F       D10
#define Gb      D11
#define G       D12
#define Ab      A7
#define A       A6
#define Bb      A5
#define B       D13

volatile int PREV_OSCA = SQUARE_WAVE;
volatile int PREV_OSCB = SQUARE_WAVE;

int C4_DELAY_ARRAY[3] = {C4_SQUARE_DELAY, C4_TRIANGLE_DELAY, C4_SAW_DELAY};
int Db4_DELAY_ARRAY[3] = {Db4_SQUARE_DELAY, Db4_TRIANGLE_DELAY, Db4_SAW_DELAY};
int D4_DELAY_ARRAY[3] = {D4_SQUARE_DELAY, D4_TRIANGLE_DELAY, D4_SAW_DELAY};
int Eb4_DELAY_ARRAY[3] = {Eb4_SQUARE_DELAY, Eb4_TRIANGLE_DELAY, Eb4_SAW_DELAY};
int E4_DELAY_ARRAY[3] = {E4_SQUARE_DELAY, E4_TRIANGLE_DELAY, E4_SAW_DELAY};
int F4_DELAY_ARRAY[3] = {F4_SQUARE_DELAY, F4_TRIANGLE_DELAY, F4_SAW_DELAY};
int Gb4_DELAY_ARRAY[3] = {Gb4_SQUARE_DELAY, Gb4_TRIANGLE_DELAY, Gb4_SAW_DELAY};
int G4_DELAY_ARRAY[3] = {G4_SQUARE_DELAY, G4_TRIANGLE_DELAY, G4_SAW_DELAY};
int Ab4_DELAY_ARRAY[3] = {Ab4_SQUARE_DELAY, Ab4_TRIANGLE_DELAY, Ab4_SAW_DELAY};
int A4_DELAY_ARRAY[3] = {A4_SQUARE_DELAY, A4_TRIANGLE_DELAY, A4_SAW_DELAY};
int Bb4_DELAY_ARRAY[3] = {Bb4_SQUARE_DELAY, Bb4_TRIANGLE_DELAY, Bb4_SAW_DELAY};
int B4_DELAY_ARRAY[3] = {B4_SQUARE_DELAY, B4_TRIANGLE_DELAY, B4_SAW_DELAY};

int RED;
int GREEN;
int BLUE;
int OSC = 0; // 0 for A, 1 for B
int CURRENT_WAVEFORM = 0;

volatile int waveform_flag = 0;
volatile int osc_flag = 0;
volatile int buttonPressStartTick = 0;

void set_color(uint16_t red, uint16_t green, uint16_t blue) {
    timer_config_channel_pwm(TIMER, RED_PIN, red);
    timer_config_channel_pwm(TIMER, GREEN_PIN, green);
    timer_config_channel_pwm(TIMER, BLUE_PIN, blue);
}

void LEDOutput(int oscillator, int waveform) {

    switch (waveform) {
        case SQUARE_WAVE:
            // RED
            RED = 0;
            GREEN = 1023;
            BLUE = 1023;
            break;
        case TRIANGLE_WAVE:
            // GREEN
            RED = 1023;
            GREEN = 0;
            BLUE = 1023;
            break;
        case SAWTOOTH_WAVE:
            // BLUE
            RED = 1023;
            GREEN = 1023;
            BLUE = 0;
            break;
        case OSCILLATOR_OFF:
            // WHITE
            RED = 0;
            GREEN = 0;
            BLUE = 0;
            break;
        default:
            // WHITE
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
    if (EXTI->PR1 & 1) {
        osc_flag = 1; // Set the flag to indicate a switch press
        buttonPressStartTick = tick; // Record the tick count when the button was pressed
        EXTI->PR1 |= 1;  // Clear interrupt flag
    }
}

void EXTI1_IRQHandler(void) {   // for switching waveforms and oscillators
    if (EXTI->PR1 & EXTI_PR1_PIF1) {
        waveform_flag = 1; // Set the flag to indicate a waveform change
        EXTI->PR1 |= EXTI_PR1_PIF1;  // Clear interrupt flag
    }
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

void PlaySquareNote(int note_delay){
    DAC_setValue(0);
    delay_us(note_delay);
    DAC_setValue(4095);
    delay_us(note_delay);
}

void PlayTriangleNote(int note_delay){
    for (int i = 0; i < note_delay/2; i += 1) {
        int dacValue = (i*4095) / (note_delay/2); // Scale to 12-bit value
        DAC_setValue(dacValue); // Set DAC value
        delay_us(1);
    }

    for (int i = note_delay/2; i >=0; i -= 1) {
        int dacValue = (i*4095) / (note_delay/2); // Scale to 12-bit value
        DAC_setValue(dacValue); // Set DAC value
        delay_us(1);
    }
}

void PlaySawNote(int note_delay){
    for (int i = 0; i < note_delay; i += 1) {
        int dacValue = (i*4095) / (note_delay); // Scale to 12-bit value
        DAC_setValue(dacValue); // Set DAC value
        delay_us(1);
    }
}

void PlayNote(int note_array[]) {
    if (CURRENT_WAVEFORM == SQUARE_WAVE){
        PlaySquareNote(note_array[0]);
    }
    if (CURRENT_WAVEFORM == TRIANGLE_WAVE){
        PlayTriangleNote(note_array[1]);
    }
    if (CURRENT_WAVEFORM == SAWTOOTH_WAVE){
        PlaySawNote(note_array[2]);
    }
}

void initialize() {
    // Initialize UART
    host_serial_init();
    for (volatile int i = 0; i < 1000000; i++) {}

    // Initialize TIM7
    TIM7_initialize();

    // Configure the timer for PWM
    timer_config_pwm(TIMER, 1000);

    // Configure the pins for PWM output
    gpio_config_alternate_function(RED_PIN, 1);
    gpio_config_alternate_function(GREEN_PIN, 1);
    gpio_config_alternate_function(BLUE_PIN, 1);

    // Configure the pins for the oscillator button
    gpio_config_mode(D3, INPUT);
    gpio_config_pullup(D3, PULL_UP);

    // Configure the pins for the waveform button
    gpio_config_mode(D6, INPUT); // Button
    gpio_config_pullup(D6, PULL_UP);

    config_gpio_interrupt();

    gpio_config_mode(C, INPUT);
    gpio_config_mode(Db, INPUT);
    gpio_config_mode(D, INPUT);
    gpio_config_mode(Eb, INPUT);
    gpio_config_mode(E, INPUT);
    gpio_config_mode(F, INPUT);
    gpio_config_mode(Gb, INPUT);
    gpio_config_mode(G, INPUT);
    gpio_config_mode(Ab, INPUT);
    gpio_config_mode(A, INPUT);
    gpio_config_mode(Bb, INPUT);
    gpio_config_mode(B, INPUT);

    gpio_config_pullup(C, PULL_DOWN);
    gpio_config_pullup(Db, PULL_DOWN);
    gpio_config_pullup(D, PULL_DOWN);
    gpio_config_pullup(Eb, PULL_DOWN);
    gpio_config_pullup(E, PULL_DOWN);
    gpio_config_pullup(F, PULL_DOWN);
    gpio_config_pullup(Gb, PULL_DOWN);
    gpio_config_pullup(G, PULL_DOWN);
    gpio_config_pullup(Ab, PULL_DOWN);
    gpio_config_pullup(A, PULL_DOWN);
    gpio_config_pullup(Bb, PULL_DOWN);
    gpio_config_pullup(B, PULL_DOWN);
          
    // initializing DAC using dacOutput
    DAC_init();
    
    LEDOutput(OSC, CURRENT_WAVEFORM); // Set LED color
    
    // initializing SysTick
    SysTick_initialize();
}

int main(void) {
    // Initialize UART
    host_serial_init();
    for (volatile int i = 0; i < 1000000; i++) {}

    // Initialize TIM7
    TIM7_initialize();

    // Configure the timer for PWM
    timer_config_pwm(TIMER, 1000);

    // Configure the pins for PWM output
    gpio_config_alternate_function(RED_PIN, 1);
    gpio_config_alternate_function(GREEN_PIN, 1);
    gpio_config_alternate_function(BLUE_PIN, 1);

    // Configure the pins for the oscillator button
    gpio_config_mode(D3, INPUT);
    gpio_config_pullup(D3, PULL_UP);

    // Configure the pins for the waveform button
    gpio_config_mode(D6, INPUT); // Button
    gpio_config_pullup(D6, PULL_UP);

    config_gpio_interrupt();

    gpio_config_mode(C, INPUT);
    gpio_config_mode(Db, INPUT);
    gpio_config_mode(D, INPUT);
    gpio_config_mode(Eb, INPUT);
    gpio_config_mode(E, INPUT);
    gpio_config_mode(F, INPUT);
    gpio_config_mode(Gb, INPUT);
    gpio_config_mode(G, INPUT);
    gpio_config_mode(Ab, INPUT);
    gpio_config_mode(A, INPUT);
    gpio_config_mode(Bb, INPUT);
    gpio_config_mode(B, INPUT);

    gpio_config_pullup(C, PULL_DOWN);
    gpio_config_pullup(Db, PULL_DOWN);
    gpio_config_pullup(D, PULL_DOWN);
    gpio_config_pullup(Eb, PULL_DOWN);
    gpio_config_pullup(E, PULL_DOWN);
    gpio_config_pullup(F, PULL_DOWN);
    gpio_config_pullup(Gb, PULL_DOWN);
    gpio_config_pullup(G, PULL_DOWN);
    gpio_config_pullup(Ab, PULL_DOWN);
    gpio_config_pullup(A, PULL_DOWN);
    gpio_config_pullup(Bb, PULL_DOWN);
    gpio_config_pullup(B, PULL_DOWN);
          
    // initializing DAC using dacOutput
    DAC_init();
    
    LEDOutput(OSC, CURRENT_WAVEFORM); // Set LED color
    
    // initializing SysTick
    SysTick_initialize();
    
    // printf("System initialized.\r\n");
    while (1) {
        if (waveform_flag) {
            delay_us(500); // Debounce delay
            
            if (!gpio_read(D6)) {
                CURRENT_WAVEFORM++;
                if (CURRENT_WAVEFORM > OSCILLATOR_OFF){
                    CURRENT_WAVEFORM = SQUARE_WAVE;
                }
                LEDOutput(OSC, CURRENT_WAVEFORM);
            }
            waveform_flag = 0;
        }

        if (osc_flag) {
            delay_us(500);
            
            if (!gpio_read(D3)) {
                delay_us(30000);
                if(!gpio_read(D3)){
                    if (OSC == OSCA) {
                        PREV_OSCA = CURRENT_WAVEFORM;
                    } else if (OSC == OSCB) {
                        PREV_OSCB = CURRENT_WAVEFORM;
                    }
        
                    OSC = !OSC;  // Toggle OSC
                    
                    if (OSC == OSCA) {
                        CURRENT_WAVEFORM = PREV_OSCA;
                        LEDOutput(OSC, CURRENT_WAVEFORM);
                    } else if (OSC == OSCB) {
                        CURRENT_WAVEFORM = PREV_OSCB;
                        ledState = 0;
                        set_color(1023, 1023, 1023); 
                        LEDOutput(OSC, CURRENT_WAVEFORM);
                    }
                }
                else {
                    CURRENT_WAVEFORM++;
                    if (CURRENT_WAVEFORM > OSCILLATOR_OFF){
                        CURRENT_WAVEFORM = SQUARE_WAVE;
                    }
                    LEDOutput(OSC, CURRENT_WAVEFORM);
                }
            }
            osc_flag = 0;
        }
        
        if (gpio_read(C)) {
            PlayNote(C4_DELAY_ARRAY);
        }
        else if (gpio_read(Db)) {
            PlayNote(Db4_DELAY_ARRAY);
        }
        else if (gpio_read(D)) {
            PlayNote(D4_DELAY_ARRAY);
        }
        else if (gpio_read(Eb)) {
            PlayNote(Eb4_DELAY_ARRAY);
        }
        else if (gpio_read(E)) {
            PlayNote(E4_DELAY_ARRAY);
        }
        else if (gpio_read(F)) {
            PlayNote(F4_DELAY_ARRAY);
        }
        else if (gpio_read(Gb)) {
            PlayNote(Gb4_DELAY_ARRAY);
        }
        else if (gpio_read(G)) {
            PlayNote(G4_DELAY_ARRAY);
        }
        else if (gpio_read(Ab)) {
            PlayNote(Ab4_DELAY_ARRAY);
        }
        else if (gpio_read(A)) {
            PlayNote(A4_DELAY_ARRAY);
        }
        else if (gpio_read(Bb)) {
            PlayNote(Bb4_DELAY_ARRAY);
        }
        else if (gpio_read(B)) {
            PlayNote(B4_DELAY_ARRAY);
        }
        else {
            DAC_setValue(0);
        }
    }
}