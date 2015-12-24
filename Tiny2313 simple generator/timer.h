/*
 * timer.h
 *
 * Created: 03.12.2013 12:26:40
 *  Author: Вадим
 */ 


#ifndef TIMER_H_
#define TIMER_H_
#include <avr/io.h>
#include <avr/interrupt.h>

#define ST_CTC_HANDMADE 255-125

uint32_t st_millis(void);
ISR(TIMER0_OVF_vect);


#endif /* TIMER_H_ */