/* 
 * File:   uart.h
 * Author: mwauras
 *
 * Created on September 20, 2018, 1:37 PM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <plib.h>
#include <p32xxxx.h>
    
void uart_init();
char uart_read();
uint8_t uart_read_ready();
uint8_t uart_write_ready();
char uart_read_nb();
void uart_write_nb(char arg);
void uart_write(char arg);
void uart_write_string(char * arg);
char * uart_read_line();
void uart_write8_nb(uint8_t arg);

char string[100];

#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

