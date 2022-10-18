#include <avr/io.h>
#include "adc.h"

void init_adc() {
    ADMUX |= (1 << REFS0);                                                      // AVCC at AREF
    ADMUX |= (1 << ADLAR);                                                      // left algined
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);                       // f_adc = f / 128
    ADCSRA |= (1 << ADATE);
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADEN);
}

void select_adc2() {
    ADMUX |= (1 << MUX1);                                                       // ADC2
    ADMUX &= ~(1 << MUX0);
}

void select_adc3() {
    ADMUX |= (1 << MUX0) | (1 << MUX1);                                         // ADC3
}

void disable_adc() {
    // ADCSRA &= ~(1 << ADEN);
    ADCSRA &= ~(1 << ADIE);
}

// void start_adc() {
//     ADCSRA |= (1 << ADEN);                                                      // enable ADC
//     ADCSRA |= (1 << ADSC);
// }

/* looks like we need two types of start_adc(), one for single conversion, the other for free running mode */
void start_adc() {
    // ADCSRA |= (1 << ADEN);                                                      // enable ADC
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADSC);                                          // this is required to start converstion, at least for the very first time
}

void stop_adc() {
    // ADCSRA &= ~(1 << ADEN);
}

uint16_t read_adc() {
    ADCSRB |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC));           // ADSC is cleared by hardware when conversion is complete

    return ADC;
}