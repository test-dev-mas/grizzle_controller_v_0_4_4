#include <avr/io.h>

#include "timers.h"

/* OVERFLOW MODE */
// void init_timer0() {
//     // TCCR0B |= (1 << CS02) | (1 << CS00);
//     TCCR0B |= (1 << CS01);
//     TIMSK0 |= (1 << TOIE0);
// }

/* CTC MODE */
void init_timer0() {
    TCCR0A |= (1 << WGM01);                     // CTC mode
    TCCR0B |= (1 << CS01) | (1 << CS00);        // f=clk_io/64
    TIMSK0 |= (1 << OCIE0A);                    // enable COMPARE MATCH A INTERRUPT
    OCR0A = 249;                                // yields 1ms interrupt
}

/*  */
void init_timer1() {
    TCCR1B |= (1 << WGM12);                     // CTC MODE
    TCCR1B |= (1 << CS11) | (1 << CS10);        // f=clk_io/64
    // TIMSK1 |= (1 <<OCIE1A);                     // enable COMPARE MATCH A INTERRUPT
    OCR1A = 832;                                // yields 300.12 Hz
}

void start_timer1() {
    TIMSK1 |= (1 <<OCIE1A);                     // enable COMPARE MATCH A INTERRUPT
}

void stop_timer1() {
    TIMSK1 &= ~(1 <<OCIE1A);
}

void start_timer2() {
    TCCR2B |= (1 << CS22);                      // f=clk_io/64
}

void stop_timer2() {
    TCCR2B &= ~(1 << CS22);                     // no clock source
}

void init_timer3() {
    TCCR3B |= (1 << WGM32);                     // CTC MODE
    TCCR3B |= (1 << CS32) | (1 << CS30);        // f = clk_io/1024
    TIMSK3 |= (1 <<OCIE3A);
    OCR3A = 12499;                              // 0.8 second
}

void stop_timer3() {
    TIMSK3 &= ~(1 <<OCIE3A);
}