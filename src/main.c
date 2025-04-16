#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include "waveforms.h"
#include <stdio.h>
#include <math.h>

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

volatile int tick = 0;
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

int main(void) {
    // initializing SysTick
    SysTick_initialize();

    // initializing UART
    host_serial_init();
    
    // wait for serial port readiness
    for (volatile int i = 0; i < 1000000; i++) { }
    printf("Hello, Audio World!\r\n");
      
    // initializing DAC using dacOutput
    DAC_init();
    int index = 0;
    float waveTable[WAVE_TABLE_SIZE];
    buildWaveTable(waveTable);  // Build the sine wave table
    
    float frequency = A4; // Set desired frequency (e.g., A4 = 440 Hz)
    float delay_us_value = (1.0 / frequency) / WAVE_TABLE_SIZE * 1e6; // Delay in microseconds
    int delay10us_value = (int)(delay_us_value / 10.0 + 0.5); // Convert to multiples of 10 us (round to nearest)

    printf("Frequency: %f Hz, Delay (us): %f, Delay10us: %d\r\n", frequency, delay_us_value, delay10us_value);

    while (1) {
        DAC_setValue(waveTable[index]);
        index++;
        if (index >= WAVE_TABLE_SIZE) {
            index = 0;  // Reset index to loop through the wave table
        }
        delay10us((int)delay_us_value / 10);  // Use the calculated delay
    }
}