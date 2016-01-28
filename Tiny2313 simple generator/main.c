/*
 * Tiny2313 simple generator.c
 * Ver 2.0
 * Created: 13.01.2016 12:37:18
 * Author : Brendel Vadim
  Суть программы:
  
  В качестве выхода генератора используется PB3 (OC1A)
  PD5, PD6 - кнопки выбора режима
  PD4, PD2 - светодиоды режима работы
  PD1, PD0 - UART, возможно будет использовать в будущем
  Ядро крутится от внутреннего генератора на частоте 8МГц.
  Таймер Т0- системный настроен на прерывание каждую милисекунду, прокручивает протопотоки.
  Таймер T1 - используется как генератор импульсов.
  Таймер1 настроен на режим PWM (mode14), OCR1 содержит длительность импульса, ICR используется как период (TOP).
  TCCRA: COM1A1=1, COM1A0=0 - Clear OC1A/OC1B on Compare Match (Set output to low level). 
  inverting Compare Output mode output is cleared on compare match and set at TOP
  TCNT считаем с тиками, делеными на прескейлер, до OCR1. Когда произошло сопаденеи дергает PB3 вниз.
  Затем TNCT считает до ICR, когда происходит совпадение, TCNT обнуляется и PB3 дергается вверх.
  Оказалось этот таймер работает как только в ICR загружено любое число больше 0. (по умолчанию OCR=0 дает
  длительность в 1 тик). При этом генерация возможна когда ICR>OCR (при ICR<OCR случае на пине висит
  высокий уровень). Так что 100% правильный способ отключения таймера - записать в ICR=0, отключить пин таймера 
  и через GPIO и положить его в 0.
  
  
  Структура режима таймера:
  Состояние - вкл, выкл, oneshot
  Режим - ручной, периодический, UART
  Длительность - uint8_t
  Период - uint8_t
  
  Протопоток1 (1мсек) проверяет кнопки и устанавливает режим работы, записывая значения в структуру.
  Протопоток2 (10мсек) конфигурирует таймер T0 согласно режиму работы и запускает генерацию
  Протопоток3 (100мсек) делает индикацию режима работы
  Протопоток4 (5мсек) общается по UART.  
 
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
 
Для версии 3.0
Можно сделать очень изящно, и универсально, но пользоваться тяжело будет и не влезет в 2313.
Текущая версия хороша именно как простой генератор с  возможностью ручного запуска.
Более универсальное решение будет неудобнее.
typedef struct {
	uint8_t state;
	uint8_t regime;
	uint16_t duration;
	uint8_t duration_multiplier;
	uint8_t duration_divider;
	uint16_t period;
	uint8_t period_multiplier;
	uint8_t period_divider;
	uint8_t prescaler_mask_id;
} generator_struct;

BUT0 - вкл/выкл генерация, ручной запуск - state
BUT1 - выбор режима - regime
BUT2 - выбор параметра для измения - duration, period, prescaler
(прескейлер, частота, делитель чатоты(сдвиг вправо на двойку, деление кратное двойке), множитель частоты, 
длительность, делитель длительности, множитель длительности)
BUT3 - увеличение параметра
BUT4 - уменьшение параметра
Суть программы - пользователь грузит в структуру таймера все значения в протопотоке кнопок.
Протопотоке настройки аппаратног таймера, исходя из настроек в структуре.
Протопток UART, пользователь шлет данные в структуру генератора, а протопоток настройки опять же все делает.
 
Но для данной реализации нужен скорее всего уже дисплей, а значит в этот камень не влезет,
да и лучше не кнопки, а энкодер. Да и тогда уж деление не на двойку, а просто выставляешь частоту и скважность 
и молотишь.
Так что в данной реализации все нормально, версию 3.0 при необходимости реализовывать уже на другом камне и плате.
  
 */ 

#include <Tiny2313_gen.h>

/*Объявление глобальных переменных*/
//Структура режима таймера 

static generator_struct generator, *p_generator=&generator;

//Указатели на структуры протопотоков
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Leds_pt;
static struct pt USART_pt;
//Статическая переменная системного таймера, хранит текущее время
volatile static uint32_t st_timer0_millis=0;
/*?Объявление глобальных переменных*/


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
ISR(TIMER1_COMPA_vect)
{
	if (p_generator->regime==GEN_REGIME_ONESHOT) 
	{
		p_generator->state=GEN_OFF;
	}
}

/* Протопотоки */
PT_THREAD(USART_listen(struct pt *pt))
{
	unsigned char sym;
	PT_BEGIN(pt);
	sym=USART_GetChar();
	if (sym!=0)
	{
		p_generator->state=GEN_STATE_BUSY;
		switch(sym)
		{
			case m:
			{
				p_generator->control=GEN_CTRL_MANUAL;
				break;
			}
			case u:
			{
				p_generator->control=GEN_CTRL_USART;
				break;
			}
			case e:
			{
				p_generator->control=GEN_CTRL_EXT;
				break;
			}
			case r:
			{
				if (p_generator->control==GEN_CTRL_USART)
				{
					p_generator->regime=GEN_REGIME_ONESHOT;
				}
				break;
			}
			case p:
			{
				if(p_generator->control==GEN_CTRL_USART)
				{
					p_generator->regime=GEN_REGIME_PERIODIC;
				}
				break;
			}
			case l:
			{
				p_generator->prescaler=USART_GetChar();
				if (p_generator->prescaler>=5) p_generator->prescaler=GEN_PRESCALER_1;
			}
			case d:
			{
				p_generator->duration=USART_GetChar();
				p_generator->duration|=(USART_GetChar()<<8);
			}
			case p:
			{
				
			}
			case s:
			{
				p_generator->state=GEN_STATE_READY;
			}
		}
	}
}

//Протопоток 1 - обработка нажатия кнопок, настройка режима генератора
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	static volatile uint8_t delay=0; //200мсек задержка между нажатиями
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=10);
	but_timer=st_millis();
	if (delay>0) 
	{
		delay--;
	}
	else 
	{
		if (!(BUT1_PORT_PIN&(_BV(BUT1))))//((BUT1_PORT_PIN&(_BV(BUT1)))==0)
		{
			if (p_generator->regime==GEN_MANUAL)
			{
				p_generator->regime=GEN_PERIODIC;
				p_generator->period=PERIOD_HZ1;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ1))
			{
				p_generator->period=PERIOD_HZ100;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ100))
			{
				p_generator->period=PERIOD_HZ1000;
			}
			else if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->period==PERIOD_HZ1000))
			{
				p_generator->regime=GEN_MANUAL;
			}
			p_generator->state=GEN_OFF;
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			delay=BUTTON_DELAY_200MSEC;
		}
		if (!(BUT0_PORT_PIN&(_BV(BUT0))))
		{
			if ((p_generator->regime==GEN_MANUAL))
			{
				p_generator->state=GEN_ONESHOT;
			}
			else if ((p_generator->regime==GEN_PERIODIC))
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
			}
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			delay=BUTTON_DELAY_200MSEC;
		}
	}
	PT_END(pt);
}
//Протопоток2 - настрока таймера1 и индикация режима работы генератора
PT_THREAD(Switch(struct pt *pt))
{
	//uint32_t dur=0;
	static volatile uint32_t switch_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//запуск протопотока каждые 10мсек
	switch_timer=st_millis();
	if (p_generator->state==GEN_OFF)
	{
		if(p_generator->regime==GEN_PERIODIC)
		{
			CLEAR_TCCR1B;
			//DISCONECT_TIMER_FROM_PIN;
			//OUT_OFF;
			if (p_generator->period==PERIOD_HZ1000)
			{
				TCCR1B|=PRESCALER_1_MASK;
				ICR1=PERIOD_HZ1000_TICKS;
			}
			else if (p_generator->period==PERIOD_HZ100)
			{
				TCCR1B|=PRESCALER_8_MASK;
				ICR1=PERIOD_HZ100_TICKS;
			}
			else
			{
				TCCR1B|=PRESCALER_256_MASK;
				ICR1=PERIOD_HZ1_TICKS;
			}
			if (p_generator->duration==DURATION_90)
			{
				OCR1=(uint16_t)((7*(uint32_t)ICR1)>>3);
			}
			else if (p_generator->duration==DURATION_50)
			{
				OCR1=(uint16_t)((uint32_t)ICR1>>1);
			}
			else
			{
				if (p_generator->period==PERIOD_HZ1000)
				{
					OCR1=(uint16_t)(5*(uint32_t)ICR1>>4);
				}
				else if (p_generator->period==PERIOD_HZ100)
				{
					OCR1=(uint16_t)((uint32_t)ICR1>>5);
				}
				else
				{
					OCR1=(uint16_t)(5*(uint32_t)ICR1>>14);
				}
			}
			p_generator->state=GEN_ON;
			CONNECT_TIMER_TO_PIN;//подключаем таймер к пину
			GENERATOR_ON;
		}
		if (p_generator->regime==GEN_MANUAL)
		{
			//TIMSK&=~_BV(TOIE1);
			GENERATOR_OFF;
			DISCONECT_TIMER_FROM_PIN;
			OUT_OFF;
			LED0_OFF;
			LED1_OFF;
		}
	}
	else if (p_generator->regime==GEN_UART)
	{
		//тут что-то будет :) можно прямо здесь написать работу от уарта, а можно в отдельно протопотоке
	}
	else if ((p_generator->regime==GEN_MANUAL)&&(p_generator->state==GEN_ONESHOT))
	{
		CLEAR_TCCR1B;
		TCCR1B|=PRESCALER_1_MASK;
		DISCONECT_TIMER_FROM_PIN;
		TCNT1=0;
		ICR1=60000;
		OCR1=2500;
		CONNECT_TIMER_TO_PIN;
		GENERATOR_ON;
		LED0_ON;
		LED1_ON;
	}
	PT_END(pt);
}
PT_THREAD(Leds(struct pt *pt))
{
	static volatile uint32_t leds_timer=0;
	static volatile uint8_t counter1, counter2=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-leds_timer)>=100);//запуск протопотока каждые 0.1мсек
	leds_timer=st_millis();
	if (p_generator->regime==GEN_PERIODIC)
	{
		switch(p_generator->period)
		{
			case PERIOD_HZ1:
			{
				if (counter1<10) //моргает LED0 раз в секунду
				{
					LED0_ON;
					counter1++;
					counter2=0;
				}
				else if (counter2<10)
				{
					LED0_OFF;
					counter2++;
					if (counter2>=9)
					{
						counter1=0;
					}
				}
				LED1_OFF;
				break;
			}
			case PERIOD_HZ100: //моргает LED0 - 10 ращ в секунду
			{
				if (((LED0_PORT_PIN&(_BV(LED0)))==0))
				{
					LED0_ON;
				}
				else
				{
					LED0_OFF;
				}
				LED1_OFF;
				break;
			}
			case PERIOD_HZ1000: //LED0 горит непрерывно
			{
				LED0_ON;
				LED1_OFF;
				break;
			}
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
	else
	{
		LED0_OFF;
		LED1_OFF;	
	}
	PT_END(pt);
}
/*?Протопотоки*/

int main(void)
{
	//volatile uint16_t test=1;
	//volatile uint32_t test2=0;
	//Инициализация струтуры генератора
	p_generator->state=GEN_OFF;
	p_generator->regime=GEN_MANUAL;
	p_generator->duration=DURATION_US320;
	p_generator->period=PERIOD_HZ1;
	//Настройка входов-выходов
	DDRD=0b00011110; //PD6 - button0, PD5 - button1, PD4 - LED1, PD3 -LED0, PD2- free, PD1 -TX, PD0 - RX
	DDRB=0b11111111; //PB3- output
	PORTD=0b01100000;//100k pull-up PD6, PD3
	PORTB=0;
	
	// Настройка системного таймера
	TCCR0A=0b00000010;//CTC operation of timer
	TCCR0B=0b00000011;//Timer count from clk with 64 prescaler
	TCNT0 = 0; //обнуляем счетчик таймера
	OCR0A = 125;//прерывание каждые 100тиков, то есть каждые 1мсек
	//TIMSK |= _BV(OCIE0A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	
	//Начальная настройка таймера генератора, 1Гц, 100мкс
	TCCR1A=0b10000010;//Toggle OC1A on campare match, PB3
	TCCR1B=0b00011100; //FastPWM with ICR s TOP, prescaler 256 ->32us resolution
	DISCONECT_TIMER_FROM_PIN;
	TCNT1=0;
	ICR1=0;
	OCR1=0;	
	TIMSK |= _BV(OCIE0A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	
	//Настройка UART
	
	 USART_Init();
	
	//Инициализация протопотоков
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	PT_INIT(&Leds_pt);
	//PT_INIT(&Sync_pt);
	//PT_INIT(&Manual_pt);
	//PT_SEM_INIT(&regime_sem, GEN_MANUAL);
	
	//Настройка собаки
	wdt_reset(); //сбрасываем собаку на всякий пожарный
	wdt_enable(WDTO_2S); //запускаем собаку с перидом 2с
	//Запускаем прерывани
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
