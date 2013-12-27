/*
 * File:   main.c
 * Author: tylerjw
 *
 * Created on December 19, 2013, 11:59 AM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <p32xxxx.h>
#include <plib.h>
#include <stdio.h>
#include "U1.h"

#define SYS_CLK 100000000L // 100MHz

//	Function Prototypes
int main(void);
void delay(volatile unsigned int count);
void mem_test_16();

void inline __attribute__((always_inline)) timer_init();
void inline __attribute__((always_inline)) timer_start20n();
unsigned int inline __attribute__((always_inline)) timer_end20n();

int main(void) {
    char buffer[80];
    unsigned int pb_clock;
    float actual_baud;
    const int baud = 500000; // max baud rate using arduino interface 

    pb_clock = SYSTEMConfigPerformance(SYS_CLK); // if sys_clock > 100MHz, pb_clock = sys_clock/2 else pb_clock = sys_clock

    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); // led

    // setup UART
    PPSUnLock;
    PPSInput(1,U1RX,RPF4); // Rx - F4 (pin 49) 5V tolerent
    PPSOutput(2,RPF5,U1TX); // Tx - F5 (pin 50) 5V tolerent
    PPSLock;
    
    actual_baud = U1_init(pb_clock, baud);

    sprintf(buffer, "SYSCLK: %d\r\n", SYS_CLK);
    U1_write(buffer);
    sprintf(buffer, "PBCLK: %d\r\n", pb_clock);
    U1_write(buffer);
    sprintf(buffer, "U1BRG: %d\r\n", U1BRG);
    U1_write(buffer);
    sprintf(buffer, "target baud: %d\r\n", baud);
    U1_write(buffer);
    sprintf(buffer, "actual baud: %f\r\n", actual_baud);
    U1_write(buffer);

    timer_init();

    while (1) {
        // turn on LED
        mPORTEWrite(BIT_4);
        // start TIMER1
        T1CONSET = BIT_15; // start timer
        // when timer1 = 50 000 (1ms) turn off led
        while(TMR1 < 50000);
        mPORTEWrite(0);
        // wait some time
        delay(SYS_CLK/4);
        // reset timer
        T1CONCLR = BIT_15;  // turn off timer
        TMR1 = 0x0;         // reset timer value
    }
}

void delay(volatile unsigned int count)
{
    while(--count);
}

void inline __attribute__((always_inline)) timer_init()
{
    // configure TIMER1
    T1CON = 0x0;        // Stop timer and clear control register
                        // prescaler to 1:1 (50MHz) PBCLK
    TMR1 = 0x0;         // Clear timer register
    PR1 = 0xFFFF;       // Load period register
}

void inline __attribute__((always_inline)) timer_start20n()
{
    // start TIMER1
    T1CONSET = BIT_15; // start timer
}

unsigned int inline __attribute__((always_inline)) timer_end20n()
{
    unsigned int tmr = TMR1;
    T1CONCLR = BIT_15; // stop the tomer
    TMR1 = 0x0;
    return tmr;
}