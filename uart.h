#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart0_init();
void uart0_transmit(uint8_t data);
void uart0_puts(char* message);

void uart1_init();
void uart1_transmit(uint8_t data);
void uart1_puts(char* message);

void uart2_init();
void uart2_transmit(uint8_t data);
void uart2_puts(char* message);

void uart3_init();
void uart3_transmit(uint8_t data);
void uart3_puts(char* message);

#endif