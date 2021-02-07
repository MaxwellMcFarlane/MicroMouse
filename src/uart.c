#include "uart.h"

void uart_init(){
    //initialize U1RX & U1TX: w/o ANSELx set digital inputs TRISx
//    U1RXR = 0x0; //RPA2 RX
//    RPA0R = 0x1; //RPA0 TX
    U2RXR = 0x3; //RPB11 RX
    RPB10R = 0x2; //RPB10 TX       
    ANSELB = 0;
    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB11 = 1;
    
    
    U2BRG = 260;
    U2STAbits.URXEN = 1;
    U2STAbits.UTXEN = 1;
    U2MODEbits.ON = 1;
    U2MODEbits.BRGH = 0;
    
}

uint8_t uart_read_ready(){
    if(U2STAbits.URXDA) return 1;
    return 0;
}

uint8_t uart_write_ready(){
    if(!U2STAbits.UTXBF) return 1;
    return 0;
}

char uart_read_nb(){
    char arg;
    if(uart_read_ready()){//wait if UTXBF is clear
        arg = U2RXREG;
        return arg;
    } else {
        return '\0';
    }
}

void uart_write_nb(char arg){
    if(uart_write_ready()){//if UTXBF is clear
        U2TXREG = arg; 
    }
}

char uart_read(){
    char arg;
    while(uart_read_ready()){}//if UTXBF is clear
    arg = U2RXREG;
    return arg;
}

char * uart_read_line(){
    int i = 0;
    char bar = '\0';
    while(bar != '\r') {
        bar = uart_read_nb();
        if (bar != '\0') {
            if (bar == 8){
                i--;
                string[i] = ' ';
                uart_write_nb(bar);
                uart_write_nb(' ');
                uart_write_nb(bar);
            } else{
                uart_write_nb(bar);
                string[i] = bar;
                i++;
            }
        }
    }
    while(!uart_write_ready()){}
    U2TXREG = '\r';
    while(!uart_write_ready()){}
    U2TXREG = '\n';
    string[i] = '\0';
    return &string[0];
}

void uart_write(char arg){
    
    while(uart_write_ready()){}//wait if UTXBF is clear
    U2TXREG = arg; 
}

void uart_write8_nb(uint8_t arg){
    
   if(uart_write_ready()){//wait if UTXBF is clear
    U2TXREG = arg;
   }
}
   

void uart_write_string(char * arg){
    uint8_t i = 0;
    while(arg[i] != '\0'){   
        if(uart_write_ready()){//if UTXBF is clear
            U2TXREG = arg[i];
            i++;
        }
    }
    //possibly write new line after printing string
}