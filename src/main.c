#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include "waveforms.h"
#include <stdio.h>
#include <math.h>

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
#define Eb      D5
#define E       D9
#define F       D10
#define Gb      D11
#define G       D12
#define Ab      A7
#define A       A6
#define Bb      D6
#define B       D13

// keeping the oscillators in memory when we switch 
volatile int PREV_OSCA = SQUARE_WAVE;
volatile int PREV_OSCB = OSCILLATOR_OFF;

// flags and ticks
volatile int waveform_flag = 0;
volatile int osc_flag = 0;
volatile int buttonPressStartTick = 0;

// arrays for knowing what waveform to switch to
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

// set color of LED based on waveform
void set_color(uint16_t red, uint16_t green, uint16_t blue) {
    timer_config_channel_pwm(TIMER, RED_PIN, red);
    timer_config_channel_pwm(TIMER, GREEN_PIN, green);
    timer_config_channel_pwm(TIMER, BLUE_PIN, blue);
}

// logic for switching between different LED states
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

void SysTick_initialize(void) {
    // disabling counter and resetting all settings
    SysTick->CTRL = 0;

    // interrupt fires every 10us
    SysTick->LOAD = 39;  

    // sets the priority of the interrupt to 15 (lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0; // reset current value
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // using processor clock
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // enable interrupt
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // enable counter
}

// delay function using systick
static void delay_us(int us) {
    int start = tick;
    while ((tick - start) < us) {
    }
}

// tim6 initialization
void TIM6_initialize(void) {
    // enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    __DSB();  // ensure clock is enabled
    
    // reset timer
    TIM6->CR1 = 0;
    TIM6->ARR = 39;
    
    // clear pending interrupts
    TIM6->SR = 0;
    
    // enable interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    
    // configure NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    
    // start timer
    TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM7_initialize(void) {
    // enable clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    __DSB();  // ensure clock is enabled
    
    // reset timer
    TIM7->CR1 = 0;
    TIM7->PSC = 3999;  // 1ms
    TIM7->ARR = BLINK_INTERVAL_MS;
    
    // clear pending interrupts
    TIM7->SR = 0;
    
    // enable interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    
    // configure NVIC
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    // start timer
    TIM7->CR1 |= TIM_CR1_CEN;
}


void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;  // clear interrupt flag
        
        if (OSC == OSCB) {  // only blink if OSCB is active
            ledState = !ledState;
            if (ledState) {
                set_color(RED, GREEN, BLUE);  // on
            } else {
                set_color(1023, 1023, 1023);  // off
            }
        }
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR1 & 1) {
        osc_flag = 1; // set flag to indicate a switch press
        EXTI->PR1 |= 1;  // clear interrupt flag
    }
}

void TIM6_IRQHandler(void) { // can i get rid of this?
}

void config_gpio_interrupt(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;
    SYSCFG->EXTICR[0] |= 1<<0;
    
    EXTI->FTSR1 |= (1<<0);
    EXTI->RTSR1 &= ~1;
    EXTI->IMR1 |= (1<<0);
    NVIC_SetPriority(EXTI0_IRQn, 2); // (IRQ number, priority)
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// square wave logic
void PlaySquareNote(int note_delay){
    DAC_setValue1(0);
    delay_us(note_delay);
    DAC_setValue1(4095);
    delay_us(note_delay);
}

// triangle wave logic (on channel 2 to let us fine tune the volume using a resistor)
void PlayTriangleNote(int note_delay){
    for (int i = 0; i < note_delay/2; i += 1) {
        int dacValue = (i*4095) / (note_delay/2); // scaling to 12-bit value
        DAC_setValue2(dacValue); 
        delay_us(1);
    }

    for (int i = note_delay/2; i >=0; i -= 1) {
        int dacValue = (i*4095) / (note_delay/2); // scaling to 12-bit value
        DAC_setValue2(dacValue);
        delay_us(1);
    }
}

// saw logic
void PlaySawNote(int note_delay){
    for (int i = 0; i < note_delay; i += 1) {
        int dacValue = (i*4095) / (note_delay); // scaling to 12-bit value
        DAC_setValue1(dacValue);
        delay_us(1);
    }
}

// play note on channel 1
void PlayNote1(int note_array[]) {
    if (OSC == OSCA){
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
    else {
        if (PREV_OSCA == SQUARE_WAVE){
            PlaySquareNote(note_array[0]);
        }
        if (PREV_OSCA == TRIANGLE_WAVE){
            PlayTriangleNote(note_array[1]);
        }
        if (PREV_OSCA == SAWTOOTH_WAVE){
            PlaySawNote(note_array[2]);
        }
    }
}

// play note on channel 2
void PlayNote2(int note_array[]) {
    if (OSC == OSCB) {
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
    else {
        if (PREV_OSCB == SQUARE_WAVE){
            PlaySquareNote(note_array[0]);
        }
        if (PREV_OSCB == TRIANGLE_WAVE){
            PlayTriangleNote(note_array[1]);
        }
        if (PREV_OSCB == SAWTOOTH_WAVE){
            PlaySawNote(note_array[2]);
        }
    }
}

// initialize everything
void initialize() {
    // initialize TIM7
    TIM7_initialize();

    // confiure timer for PWM
    timer_config_pwm(TIMER, 1000);

    // configure pins for PWM output
    gpio_config_alternate_function(RED_PIN, 1);
    gpio_config_alternate_function(GREEN_PIN, 1);
    gpio_config_alternate_function(BLUE_PIN, 1);

    // configure pins for the oscillator button
    gpio_config_mode(D3, INPUT);
    gpio_config_pullup(D3, PULL_UP);

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
    
    LEDOutput(OSC, CURRENT_WAVEFORM); // set LED color
    
    // initializing SysTick
    SysTick_initialize();
}

int main(void) {
    // initialize everything
    initialize();

    while (1) {
        if (waveform_flag) {
            delay_us(500); // debounce delay
            
            if (!gpio_read(D6)) { // cycle thru waveforms
                CURRENT_WAVEFORM++;
                if (CURRENT_WAVEFORM > OSCILLATOR_OFF){
                    CURRENT_WAVEFORM = SQUARE_WAVE; // loop back to square
                }
                LEDOutput(OSC, CURRENT_WAVEFORM);
            }
            waveform_flag = 0; // reset flag
        }

        if (osc_flag) {
            delay_us(500); // debounce
            
            if (!gpio_read(D3)) {
                delay_us(30000);
                if(!gpio_read(D3)){
                    if (OSC == OSCA) {
                        PREV_OSCA = CURRENT_WAVEFORM;
                    } else if (OSC == OSCB) {
                        PREV_OSCB = CURRENT_WAVEFORM;
                    }
        
                    OSC = !OSC;  // toggle OSC
                    
                    if (OSC == OSCA) {
                        CURRENT_WAVEFORM = PREV_OSCA;
                        LEDOutput(OSC, CURRENT_WAVEFORM);
                    } 
                    else if (OSC == OSCB) {
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
        
        // play notes
        if (gpio_read(C)) {
            PlayNote1(C4_DELAY_ARRAY);
            PlayNote2(C4_DELAY_ARRAY);
        }
        else if (gpio_read(Db)) {
            PlayNote1(Db4_DELAY_ARRAY);
            PlayNote2(Db4_DELAY_ARRAY);
        }
        else if (gpio_read(D)) {
            PlayNote1(D4_DELAY_ARRAY);
            PlayNote2(D4_DELAY_ARRAY);
        }
        else if (gpio_read(Eb)) {
            PlayNote1(Eb4_DELAY_ARRAY);
            PlayNote2(Eb4_DELAY_ARRAY);
        }
        else if (gpio_read(E)) {
            PlayNote1(E4_DELAY_ARRAY);
            PlayNote2(E4_DELAY_ARRAY);
        }
        else if (gpio_read(F)) {
            PlayNote1(F4_DELAY_ARRAY);
            PlayNote2(F4_DELAY_ARRAY);
        }
        else if (gpio_read(Gb)) {
            PlayNote1(Gb4_DELAY_ARRAY);
            PlayNote2(Gb4_DELAY_ARRAY);
        }
        else if (gpio_read(G)) {
            PlayNote1(G4_DELAY_ARRAY);
            PlayNote2(G4_DELAY_ARRAY);
        }
        else if (gpio_read(Ab)) {
            PlayNote1(Ab4_DELAY_ARRAY);
            PlayNote2(Ab4_DELAY_ARRAY);
        }
        else if (gpio_read(A)) {
            PlayNote1(A4_DELAY_ARRAY);
            PlayNote2(A4_DELAY_ARRAY);
        }
        else if (gpio_read(Bb)) {
            PlayNote1(Bb4_DELAY_ARRAY);
            PlayNote2(Bb4_DELAY_ARRAY);
        }
        else if (gpio_read(B)) {
            PlayNote1(B4_DELAY_ARRAY);
            PlayNote2(B4_DELAY_ARRAY);
        }
        else {
            DAC_setValue1(0); // silence
            DAC_setValue2(0);
        }
    }
}