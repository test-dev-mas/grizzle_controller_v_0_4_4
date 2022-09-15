#include <avr/io.h>

#include "uart.h"

#define BAUD_0      115200
#define UBRR_0      ((F_CPU)/(BAUD_0*8UL)-1)    // see spec sheet: buad rate generator        
#define BAUD_1      115200
#define UBRR_1      ((F_CPU)/(BAUD_1*8UL)-1)    // see spec sheet: buad rate generator
#define BAUD_2      115200
#define UBRR_2      ((F_CPU)/(BAUD_2*8UL)-1)    // see spec sheet: buad rate generator
#define BAUD_3      9600
#define UBRR_3      ((F_CPU)/(BAUD_3*8UL)-1)    // see spec sheet: buad rate generator

void uart0_init() {
    UBRR0L = (uint8_t)UBRR_0;                   // write lower byte
    UBRR0H = (uint8_t)(UBRR_0 >> 8);            // write higher byte

    UCSR0A |= (1<<U2X0);                        // U2X0 bit seems to be 1 by default on Pro Mini 3.3V 8Mhz
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);      // enable transmitter & receiver
    UCSR0B |= (1 << RXCIE0);                    // enable RX Complete Interrupt
}

void uart0_transmit(uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0)));             // wait for empty transmit buffer
    UDR0 = data;                                // put data into buffer
}

void uart0_puts(char* message) {
    char* p = message;
    while (*p) {
        uart0_transmit(*p++);
    }
}

void uart1_init() {
    UBRR1L = (uint8_t)UBRR_1;                   // write lower byte
    UBRR1H = (uint8_t)(UBRR_1 >> 8);            // write higher byte

    UCSR1A |= (1<<U2X1);                        // U2X0 bit seems to be 1 by default on Pro Mini 3.3V 8Mhz
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1);      // enable transmitter & receiver
}

void uart1_transmit(uint8_t data) {
    while (!(UCSR1A & (1<<UDRE1)));             // wait for empty transmit buffer
    UDR1 = data;                                // put data into buffer
}

void uart2_init() {
    UBRR2L = (uint8_t)UBRR_2;                   // write lower byte
    UBRR2H = (uint8_t)(UBRR_2 >> 8);            // write higher byte

    UCSR2A |= (1<<U2X2);                        // U2X0 bit seems to be 1 by default on Pro Mini 3.3V 8Mhz
    UCSR2B |= (1 << RXEN2) | (1 << TXEN2);      // enable transmitter & receiver
}

void uart2_transmit(uint8_t data) {
    while (!(UCSR2A & (1<<UDRE2)));             // wait for empty transmit buffer
    UDR2 = data;                                // put data into buffer
}

void uart2_puts(char* message) {
    char* p = message;
    while (*p) {
        uart2_transmit(*p++);
    }
}

void uart3_init() {
    UBRR3L = (uint8_t)UBRR_3;                   // write lower byte
    UBRR3H = (uint8_t)(UBRR_3 >> 8);            // write higher byte

    UCSR3A |= (1<<U2X3);                        // U2X0 bit seems to be 1 by default on Pro Mini 3.3V 8Mhz
    UCSR3B |= (1 << RXEN3) | (1 << TXEN3);      // enable transmitter & receiver
}

void uart3_transmit(uint8_t data) {
    while (!(UCSR3A & (1<<UDRE3)));             // wait for empty transmit buffer
    UDR3 = data;                                // put data into buffer
}

void uart3_puts(char* message) {
    char* p = message;
    while (*p) {
        uart3_transmit(*p++);
    }
}