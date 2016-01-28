/*Макрооредения*/
//Макроопределения входов-выходов
#ifndef __TINY2313_GEN_H__
#define __TINY2313_GEN_H__

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>
#include <usart.h>

#define OUT PB3
#define OUT_PORT PORTB
#define OUT_PORT_DDR DDRB
#define OUT_PORT_PIN PINB
#define BUT0 PD6 //левая кнопка
#define BUT0_PORT PORTD
#define BUT0_PORT_DDR DDRD
#define BUT0_PORT_PIN PIND
#define BUT1 PD5 //правая кнопка
#define BUT1_PORT PORTD
#define BUT1_PORT_DDR DDRD
#define BUT1_PORT_PIN PIND
#define LED0 PD3
#define LED0_PORT PORTD
#define LED0_PORT_DDR DDRD
#define LED0_PORT_PIN PIND
#define LED1 PD4
#define LED1_PORT PORTD
#define LED1_PORT_DDR DDRD
#define LED1_PORT_PIN PIND
#define PIN_(port)  PIN  ## port
#define PIN(port)  PIN_(port)
#define PRESCALER_1 1
#define PRESCALER_8 8
#define PRESCALER_64 64
#define PRESCALER_256 256 //PRESCALER для Timer1, задается в TCCR1B посдледними 3 битами
#define BUTTON_DELAY_200MSEC 20

//Макроопредения режимов
#define GEN_CTRL_MANUAL 0
#define GEN_CTRL_USART 1
#define GEN_CTRL_EXT 2
#define GEN_REGIME_ONESHOT 0
#define GEN_REGIME_PERIODIC 1
#define GEN_STATE_BUSY 0
#define GEN_STATE_READY 1
#define GEN_PRESCALER_1 1
#define GEN_PRESCALER_8 2
#define GEN_PRESCALER_64 3
#define GEN_PRESCALER_256 4
#define PERIOD_HZ1_TICKS (uint16_t)((F_CPU/PRESCALER_256))
#define PERIOD_HZ100_TICKS (uint16_t)((F_CPU/(PRESCALER_8*100)))
#define PERIOD_HZ1000_TICKS (uint16_t)((F_CPU/(PRESCALER_1*1000)))
#define PRESCALER_NO_MASK 0b00000111
#define PRESCALER_1_MASK 1
#define PRESCALER_8_MASK 2
#define PRESCALER_64_MASK 3
#define PRESCALER_256_MASK 4

//Макроопредления вспомогательные
#define B(bit_no)         (1 << (bit_no))
#define CB(reg, bit_no)   (reg) &= ~B(bit_no)
#define SB(reg, bit_no)   (reg) |= B(bit_no)
#define VB(reg, bit_no)   ( (reg) & B(bit_no) )
#define TB(reg, bit_no)   (reg) ^= B(bit_no)
//Макросы для работы со светодиодами индикации
#define LED0_ON LED0_PORT|=_BV(LED0)
#define LED0_OFF LED0_PORT&=~_BV(LED0)
#define LED1_ON LED1_PORT|=_BV(LED1)
#define LED1_OFF LED1_PORT&=~_BV(LED1)
//Макросы ручного управления выходом
#define OUT_ON OUT_PORT|=_BV(OUT)
#define OUT_OFF OUT_PORT&=~_BV(OUT)
//Макросы выключения, включения генератора
#define GENERATOR_ON TIMSK |= _BV(OCIE1A)
#define GENERATOR_OFF TIMSK &= ~_BV(OCIE1A)
//Макросы работы с таймером
#define CLEAR_TCCR1B TCCR1B &= (~PRESCALER_NO_MASK)
#define CONNECT_TIMER_TO_PIN TCCR1A |= _BV(COM1A1)
#define DISCONECT_TIMER_FROM_PIN TCCR1A &= ~_BV(COM1A1)
//Бит за номером, очистка, установка, проверка, переключение
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))

//Структура режима генератора
typedef struct {
	uint8_t control;
	uint8_t regime;
	uint8_t state;
	uint8_t prescaler;
	uint16_t duration;
	uint16_t period;
} generator_struct;

//Объявления функций
uint32_t st_millis(void);

#endif