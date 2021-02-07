/* 
 * File:   analog_in.h
 * Author: mwauras
 *
 * Created on March 25, 2019, 10:09 PM
 */

#ifndef ANALOG_IN_H
#define	ANALOG_IN_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

//void delay_ms(uint32_t delay);
void analog_in_init();
int32_t analog_in_read(uint8_t analogPIN);

#endif	/* ANALOG_IN_H */

