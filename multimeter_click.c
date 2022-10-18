#include <avr/io.h>

#include "multimeter_click.h"

/*  the potential difference at two input terminals is compared to VGND, which is at half of Vref(2.048)
    0 potential difference should result in 2047/2048, mid-point of 12-bit conversion range
    smaller value than 2047/2048 means negative potential difference at input terminals
    larger value than 2047/2048 means positive potential difference at input terminals
    V_measured = (N - 2047)/4096 * 2.048 * 17
    2.048 is the V_ref to ADC, 17 is the constant intrinsic to the analog front-end (is empirically derived like 12V -> 470) 
    a calibration against a high precision reference voltage is needed to pin point this constant
    curve fitting using dps5005 power supply: y= 0.0083*x - 17.0091
*/

void multimeter_init() {
    DDRD |= (1 << CS);
    PORTD |= (1 << CS);

    DDRB |= (1 << CLK) | (1 << D_IN);
    // PORTB |= (1 << D_IN);
}

static uint16_t multimeter_read_channel(uint8_t channel) {
    uint8_t write_data;
    // uint8_t read_buf[2];
    uint16_t ret = 0;

    write_data = 0x60;                  // nNULL:NULL:START:SINGLE/DIFF:D2:D1:D0:S/H:NULL (0b00110000) everything is in initial state
    write_data |= (channel<<2);         // set channel bits:D2/D1/D0

    PORTD &= ~(1 << CS);                // bringing CS low initiates communication with MCP3204
    // PORTB |= (1 << CLK);                // first clock received with CS low and DIN high will constitute a start bit

    for (int i=6;i>=0;i--) {
        (write_data&(1<<i)) ? (PORTB |= (1 << D_IN)) : (PORTB &= ~(1 << D_IN));
        // if (write_data&(1<<i)) {
        //     PORTB |= (1 << D_IN);       // write 1
        // }
        // else {
        //     PORTB &= ~(1 << D_IN);      // write 0
        // }
        PORTB |= (1 << CLK);            // toggle clock
        PORTB &= ~(1 << CLK);
    }

    for (int i=11;i>=0;i--) {
        if (PINB & (1 << D_OUT)) {
            ret |= (1 << i);
        }
        PORTB |= (1 << CLK);            // toggle clock
        PORTB &= ~(1 << CLK);
    }

    PORTD |= (1 << CS);

    return ret;
}

uint16_t multimeter_read_voltage() {
    return multimeter_read_channel(MULTIMETER_U_CHANNEL);
}