/*
 * timer.c
 *
 * Created: 03.12.2013 12:26:17
 *  Author: Вадим
 */ 
#include "timer.h"
#include <util/atomic.h>

volatile static uint32_t st_timer0_millis;

ISR(TIMER0_OVF_vect)
{
	st_timer0_millis++;
	TCNT0 = ST_CTC_HANDMADE;
}

uint32_t st_millis(void)
{
	uint32_t m;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		m = st_timer0_millis;
	}

	return m;
}
