/*
 * Tiny2313 simple generator.c
 * Ver 2.0
 * Created: 13.01.2016 12:37:18
 * Author : Brendel Vadim
  Суть программы:
  
  В качестве выхода генератора используется PD3
  PD5, PD6 - кнопки выбора режима
  PD4, PD2 - светодиоды режима работы
  PD1, PD0 - UART, возможно будет использовать в будущем
  Ядро крутится от внутреннего генератора на частоте 4МГц.
  Таймер Т0- системный настроен на прерывание каждую милисекунду, прокручивает протопотоки.
  Таймер T1 - используется как генератор импульсов.
  
  Структура режима таймера:
  Состояние - вкл/выкл
  Режим - ручной, периодический, UART
  Длительность - uint
  Период - uint
  
  Протопоток1 (1мсек) проверяет кнопки и устанавливает режим работы, записывая значения в структуру.
  Ппротопоток2 (10мсек) конфигурирует таймер T0 согласно режиму работы и запусает генерацию и делает индикацию режима.
  В ручном режиме вызывает дочерний протопоток который отключает таймер и дерагает выход просто по пину.
  Протопок3 (5мсек) общается по UART. Ждет букву S, если она пришла выставляет семафор гасит протопоток1 и
  протопоток 2. Получает значения длительности и периода, проверяет их на корректность, конфигурирует таймер,
  запускает. При приходе буквы P, гасит таймер, отдает управление в ручной режим.
 
 
  Протопоток2:
  Если кнопка0=0 - ручной режим, (светодиоды 11) выплевываем импльс 80мкс при нажатии кнопки1
  Если кнопка0= 1 и кнопка1=0 - частота 1Гц(светодиды 10),  длительность 80мкс, 62500тиков, 5 тиков - скважность 0,0001 (0,01%)
  Если кнопка0= 2 и кнопка1=1 - частота 1Гц(светодиды 10), длительность 0,5с, 62500тиков, 31500 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частота 1Гц(светодиды 10), длительность 0,9с, 62500тиков, 56250 тиков - скважность 0,9
  Если кнопка0= 1 и кнопка1=0 - частота 100Гц(светодиды 01), длительность 80мкс, 625тиков, 5 тиков - скважность 0,01 (1%)
  Если кнопка0= 2 и кнопка1=1 - частота 100Гц(светодиды 01), длительность 0,05с, 625тиков, 615 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частота 100ГЦ(светодиды 01), длительность 0,09с, 625тиков, 562 тиков - скважность 0,9
  Если кнопка0= 1 и кнопка1=0 - частоты 1000ГЦ(светодиды 00), длительность 80мкс, 62тиков, 5 тиков - скважность 0,1 (10%)
  Если кнопка0= 2 и кнопка1=1 - частоты 1000ГЦ(светодиды 00), длительность 496мкс, 62тиков, 31 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частоты 1000ГЦ(светодиды 00), длительность 896мкс, 62тиков, 56 тиков - скважность 0,9
 
  
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>


/*Макрооредения*/
//Макроопределения входов-выходов

#define OUT PD3
#define OUT_PORT PORTD
#define OUT_PORT_DDR DDRD
#define OUT_PORT_PIN PIND
#define BUT0 PD5
#define BUT0_PORT PORTD
#define BUT0_PORT_DDR DDRD
#define BUT0_PORT_PIN PIND
#define BUT1 PD6
#define BUT1_PORT PORTD
#define BUT1_PORT_DDR DDRD
#define BUT1_PORT_PIN PIND
#define LED0 PD2
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

//Макроопредения режимов
#define GEN_ON 1
#define GEN_OFF 0
#define GEN_MANUAL 0
#define GEN_PERIODIC 1
#define GEN_UART 2
#define PERIOD_HZ1_TICKS (uint16_t)((F_CPU/PRESCALER_256)) 
#define PERIOD_HZ100_TICKS (uint16_t)((F_CPU/(PRESCALER_8*100)))
#define PERIOD_HZ1000_TICKS (uint16_t)((F_CPU/(PRESCALER_1*1000)))
#define DURATION_US320 0
#define DURATION_50 1
#define DURATION_90 2
#define PERIOD_HZ1 0
#define PERIOD_HZ100 1
#define PERIOD_HZ1000 2
#define PRESCALER_1_MASK 1
#define PRESCALER_8_MASK 2
#define PRESCALER_64_MASK 3
#define PRESCALER_256_MASK 8


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
//Бит за номером, очистка, установка, проверка, переключение
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))

/*?Макрооредения*/

/*Объявление глобальных переменных*/
//Структура режима таймера 

typedef struct {
	uint8_t state;
	uint8_t regime;
	uint8_t duration;
	uint8_t period;
} generator_struct;
static generator_struct generator, *p_generator=&generator;

//Указатели на структуры протопотоков
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Leds_pt;
static struct pt Sync_pt;
//static struct pt_sem manual_pulse; //указатель на структуру семафора
//Статическая переменная системного таймера, хранит текущее время
volatile static uint32_t st_timer0_millis;
/*?Объявление глобальных переменных*/


/*Объявление функций*/
//Фукция выдачи текущего системного времени
uint32_t st_millis(void);
/*?Объявление функций*/


/*Функции*/
//Фукция выдачи текущего системного времени
uint32_t st_millis(void)
{
	uint32_t m;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		m = st_timer0_millis;
	}

	return m;
}
/*?Функции*/


/*Обработчики прерываний*/
//Обработка прерывания по совпадению от таймера0, увеличивает системный таймер
ISR(TIMER0_COMPA_vect)
{
	st_timer0_millis++;
}
//Обработка прерывания по совпадению от таймера1
ISR(TIMER1_COMPA_vect)
{
	
}
/*?Обработчики прерываний*/

/* Протопотоки */

//Дочерний протопоток, который отрабатывает ручной запуск
PT_THREAD(Sync(struct pt *pt))
{
	PT_BEGIN(pt);
	//PT_SEM_SIGNAL(pt, &manual_pulse); //устанавливает 1 в manual_pulse, сигнализируя что кнопки больше не опрашивались в другом пропотоке
	OUT_OFF; //устанавливаем 0 на выходе
	if ((BUT1_PORT_PIN&(_BV(BUT1)))==0)
	{
		LED0_ON;
		OUT_ON; //устанавливаем 1 на выходе
		_delay_us(5); //держим 1 на пине 80 микросекунд
		OUT_OFF; //сбрасываем выход в 0
		_delay_ms(10); //задержка перед следующим срабатыванием
		LED0_OFF;
	}
	PT_EXIT(pt);
	PT_END(pt);
}

//Протопоток 1 - обработка нажатия кнопок, настройка режима генератора
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=80);
	but_timer=st_millis();
	if ((BUT0_PORT_PIN&(_BV(BUT0)))==0)
	{
		if (p_generator->regime==GEN_MANUAL) 
		{
			p_generator->regime=GEN_PERIODIC; 
			p_generator->period=PERIOD_HZ1;
		} //при долгом нажатии кн0, проиходит смена периода
		else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period=PERIOD_HZ1))
		{
			p_generator->period=PERIOD_HZ100;
		}
		else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period=PERIOD_HZ100))
		{
			p_generator->period=PERIOD_HZ1000;
		}
		else
		{
			p_generator->regime=GEN_MANUAL;
		}
		p_generator->state=GEN_OFF;
		GENERATOR_OFF;
	}
	if ((BUT1_PORT_PIN&(_BV(BUT1)))==0)
	{
		if (p_generator->duration==DURATION_US320) 
		{
			p_generator->duration=DURATION_50;
		}
		else if (p_generator->duration==DURATION_50)
		{
			p_generator->duration=DURATION_90;
		}
		else
		{
			p_generator->duration=DURATION_US320;
		}
		p_generator->state=GEN_OFF;
		GENERATOR_OFF;
	}
	PT_END(pt);
}
//Протопоток2 - настрока таймера1 и индикация режима работы генератора
PT_THREAD(Switch(struct pt *pt))
{
	uint32_t dur=0;
	static volatile uint32_t switch_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//запуск протопотока каждые 10мсек
	if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->state==GEN_OFF))
	{
		if (p_generator->period==PERIOD_HZ1000)
		{
			TCCR1B|=PRESCALER_1_MASK;
			ICR1=PERIOD_HZ1000;
		}
		else if (p_generator->period==PERIOD_HZ100)
		{
			TCCR1B|=PRESCALER_8_MASK;
			ICR1=PERIOD_HZ100;
		}
		else 
		{
			TCCR1B|=PRESCALER_256_MASK;
			ICR1=PERIOD_HZ1;
		}
		if (p_generator->duration==DURATION_90)
		{
			dur=((7*ICR1)>>3);
			OCR1=(uint16_t)dur;
		}
		else if (p_generator->duration==DURATION_50)
		{
			OCR1=(ICR1>>1);
		}
		else
		{
			OCR1=(ICR1>>12);
		}
		p_generator->state=GEN_ON;
		GENERATOR_ON;
	}
	else if (p_generator->regime==GEN_UART)
	{
		//тут что-то будет :) можно прямо здесь написать работу от уарта, а можно в отдельно протопотоке
	}
	else 
	{
		GENERATOR_OFF;
		p_generator->regime=GEN_MANUAL; //на всякий пожарный, если режим генератора свалится в что-то неизвестное,
		//то попадет сюда и выставит ручной режим
		p_generator->state=GEN_OFF;
		PT_SPAWN(pt, &Sync_pt, Sync(&Sync_pt));//вызываем дочерний протопоток ручного или синхро запуска
		//Макс частота нажатия кнопки ~2Гц
		LED0_OFF;
		LED1_OFF;
	}
	switch_timer=st_millis();
	PT_END(pt);
}
PT_THREAD(Leds(struct pt *pt))
{
	static volatile uint32_t leds_timer=0;
	static volatile uint8_t counter=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-leds_timer)>=100);//запуск протопотока каждые 0.1мсек
	if (p_generator->regime==GEN_PERIODIC)
	{
		switch(p_generator->period)
		{
			case PERIOD_HZ1:
				if (((LED0_PORT_PIN&(_BV(LED0)))==0)&&(counter>10)) //моргает LED0 раз в секунду
				{
					LED0_OFF;
					counter=0;
				}
				else 
				{
					LED0_ON;
					counter++;
				}
				LED1_OFF;
				break;
			case PERIOD_HZ100:
				if (((LED0_PORT_PIN&(_BV(LED0)))==0)) //моргает LED0 10 раз в секунду
				{
					LED0_OFF;
				}
				else
				{
					LED0_ON;
				}
				LED1_OFF;
				break;
			case PERIOD_HZ1000: //оба светодиоа горят
				LED0_ON;
				LED1_ON;
				break;
		}
	}
	else if (p_generator->regime==GEN_UART) //моргаем обоими светодиодами
	{
		if ((LED0_PORT_PIN&(_BV(LED0)))==0)
		{
			LED0_OFF;
			LED1_OFF;
		}
		else 
		{
			LED0_ON;
			LED1_ON;
		}
	}
	PT_END(pt);
}
/*?Протопотоки*/


int main(void)
{
	//Инициализация струтуры генератора
	p_generator->state=GEN_OFF;
	p_generator->regime=GEN_MANUAL;
	p_generator->duration=DURATION_US320;
	p_generator->period=PERIOD_HZ1;
	//Настройка входов-выходов
	DDRD=0b00011110; //PD6 - button1, PD5 - button0, PD4 - OUT, PD3 - LED1, PD2- LED0, PD1 -TX, PD0 - RX
	DDRB=0b11111111; //all pins on portb are outputs
	PORTD=0b01100000;//100k pull-up PD6, PD5
	PORTB=0;
	
	// Настройка системного таймера
	TCCR0A=0b00000010;//CTC operation of timer
	TCCR0B=0b00000011;//Timer count from clk with 64 prescaler
	TCNT0 = 0; //обнуляем счетчик таймера
	OCR0A = 125;//прерывание каждые 100тиков, то есть каждые 1мсек
	//TIMSK |= _BV(OCIE0A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	
	//Начальная настройка таймера генератора, 1Гц, 100мкс
	TCCR1A=0b01000010;//Toggle OC1A on campare match
	TCCR1B=0b00011100; //FastPWM with ICR s TOP, prescaler 256 ->32us resolution
	TCNT1=0;
	ICR1=p_generator->period;
	OCR1=p_generator->duration;
	TIMSK |= _BV(OCIE0A)|_BV(OCIE1A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	
	//Настройка UART
	
	//Инициализация протопотоков
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	PT_INIT(&Leds_pt);
	PT_INIT(&Sync_pt);
	
	//Настройка собаки
	wdt_reset(); //сбрасываем собаку на всякий пожарный
	wdt_enable(WDTO_2S); //запускаем собаку с перидом 2с
	
	//Разрешаем прерывания, запускаем работу шедулера
	sei();

    while(1)
    {
		//Крутим протопотоки
		PT_SCHEDULE(Buttons(&Buttons_pt));
		PT_SCHEDULE(Switch(&Switch_pt));
		PT_SCHEDULE(Leds(&Leds_pt));
		wdt_reset(); //переодически сбрасываем собаку чтобы не улетететь в ресет
	 }
}
