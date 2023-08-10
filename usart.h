#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED


#include <stdio.h>
struct FLAGS {
    volatile uint8_t rx_echo:1;
    volatile uint8_t rx_flush:1;  // Flush until next CR/LF seen
    volatile uint8_t rx_int:1;
    volatile uint8_t cmd_int:1;
};

extern struct FLAGS flags;

// Init the Software Uart
void USART_init(unsigned int);

// Writes a character to the serial port.
int USART_putchar(const char, FILE *);
int USART_getchar(FILE *);

// Write a NULL-terminated string from RAM to the serial port
void USART_puts(const char *s);

// Write a NULL-terminated string from program-space (flash) 
// to the serial port. example: USART_puts_p(PSTR("test"))
void USART_puts_p(const char *prg_s);

// Helper-Macro - "automatically" inserts PSTR
// when used: include avr/pgmspace.h before this include-file
#define USART_puts_P(s___) USART_puts_p(PSTR(s___))

uint8_t USART_rxready();

char buf_get();
char buf_ready();

extern char buf_full;

#endif // USART_H_INCLUDED
