#include <p32xxxx.h>
#include <plib.h>
#include "timer1.h"

void inline __attribute__((always_inline)) timer1_init()
{
    // configure TIMER1
    T1CON = 0x0;        // Stop timer and clear control register
                        // prescaler to 1:1 (50MHz) PBCLK
    TMR1 = 0x0;         // Clear timer register
    PR1 = 0xFFFF;       // Load period register
}

void inline __attribute__((always_inline)) timer1_start20n()
{
    // start TIMER1
    T1CONSET = BIT_15; // start timer
}

unsigned int inline __attribute__((always_inline)) timer1_end20n()
{
    unsigned int tmr = TMR1;
    T1CONCLR = BIT_15; // stop the tomer
    TMR1 = 0x0;
    return tmr;
}

// ~100ns error
void inline __attribute__((always_inline)) timer1_delay_ms(unsigned int ms)
{
    while(ms-- > 0)
    {
        T1CONSET = BIT_15; // start timer
        while(TMR1 < (49980)); // tuned
        T1CONCLR = BIT_15; // stop the tomer
        TMR1 = 0x0;
    }
}

// actual delay 1.02 us
void inline __attribute__((always_inline)) timer1_delay_us(unsigned int us)
{
    while(us-- > 0)
    {
        T1CONSET = BIT_15; // start timer
        while(TMR1 < (36)); // tuned
        T1CONCLR = BIT_15; // stop the tomer
        TMR1 = 0x0;
    }
}