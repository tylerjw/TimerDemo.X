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
#include "timer1.h"

#define SYS_CLK 100000000L // 100MHz

//	Function Prototypes
int main(void);
void inline __attribute__((always_inline)) delay(volatile unsigned int count);

void timer1ms_test();
void timer_delaymeasure_test();
void timer_delay_test();

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

    timer1_init();

    timer_delay_test();
}

void timer1ms_test()
{
    // test LED output (e4) on osciliscope for rising edge to falling edge of 1ms
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

void timer_delaymeasure_test()
{
    unsigned int tmr_value;
    char buffer[80];
    while(1)
    {
        mPORTEWrite(BIT_4);
        timer1_start20n();
        delay(SYS_CLK/4);
        tmr_value = timer1_end20n();
        sprintf(buffer, "delay(%d)\r\ntmr_value = %d\r\n", SYS_CLK/4, tmr_value);
        U1_write(buffer);
        mPORTEWrite(0);
        delay(SYS_CLK/4);
    }
}

void timer_delay_test()
{
    while(1)
    {
        mPORTEWrite(BIT_4);
        timer1_delay_ms(1000);
        mPORTEWrite(0);
        timer1_delay_ms(1000);
    }
}

void inline __attribute__((always_inline)) delay(volatile unsigned int count)
{
    while(--count) __asm("nop");
}