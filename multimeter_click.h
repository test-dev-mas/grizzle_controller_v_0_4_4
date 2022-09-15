#ifndef _MULTIMETER_CLICK_H
#define _MULTIMETER_CLICK_H

#include <stdint.h>
#include <avr/io.h>

#define CS                      PD1
#define CLK                     PB1
#define D_IN                    PB2
#define D_OUT                   PB3

#define MULTIMETER_I_CHANNEL    0x00
#define MULTIMETER_U_CHANNEL    0x01
#define MULTIMETER_R_CHANNEL    0x02
#define MULTIMETER_C_CHANNEL    0x03

void multimeter_init();
void multimeter_config(uint8_t pins);
static uint16_t multimeter_read_channel(uint8_t channel);
uint16_t multimeter_read_current();
uint16_t multimeter_read_voltage();
uint16_t multimeter_read_resistance();
uint16_t multimeter_read_capacitance();

#endif