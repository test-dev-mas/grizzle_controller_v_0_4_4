#include <avr/io.h>

#include "spi.h"

#define CS      PB0
#define SCK     PB1
#define MOSI    PB2
#define MISO    PB3

void spi_init() {
    DDRB |= (1<<CS) | (1<<SCK) | (1<<MOSI);
    /* pull CS high to disable any chip connected on spi bus */
    PORTB |= (1<<CS);
    /* leave SPI2X, SPR0, & SPR1 zero, fsck=fosc/4 
        SPR0/1 = 1, fsck=fosc/128
    */
    SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0);
}

uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    /* wait until transmission ends */
    while (!(SPSR & (1<<SPIF)));
    /* SPDR contains received data */
    return SPDR;
}