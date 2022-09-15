#ifndef _ADC_H
#define _ADC_H

void init_adc();
void select_adc2();
void select_adc3();
void disable_adc();
void start_adc();
void stop_adc();
uint16_t read_adc();

#endif