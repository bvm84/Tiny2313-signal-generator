/*
 * Tiny2313 simple generator.c
 * Ver 2.0
 * Created: 13.01.2016 12:37:18
 * Author : Brendel Vadim
  Суть программы:
  
  В качестве выхода генератора используется PD4
  PD5, PD6 - кнопки выбора режима
  PD3, PD2 - светодиоды режима работы
  PD1, PD0 - UART, возможно будет использовать в будущем
  Ядро крутится от внутреннего генератора на частоте 4МГц.
  Таймер Т1- системный настроен на прерывание каждую милисекунду, прокручивает протопотоки.
  Таймер T0 - используется как генератор импульсов.
  
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

#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port
#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)
#define OUT PD4
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
#define LED1 PD3
#define LED1_PORT PORTD
#define LED1_PORT_DDR DDRD
#define LED1_PORT_PIN PIND

//Макроопредения режимов
#define GEN_ON 1
#define GEN_OFF 0
#define GEN_MANUAL 0
#define GEN_PERIODIC 1
#define GEN_UART 2
#define PERIOD_HZ1 1 
#define PERIOD_HZ100 100 
#define PERIOD_HZ1000 1000 
#define DURATION_US100 1
#define DURATION_50 2
#define DURATION_90 3

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
//Бит за номером, очистка, установка, проверка, переключение
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))
/*Макрооредения*/

/*Объявление глобальных переменных*/
//Структура режима таймера 

typedef struct {
	uint8_t state;
	uint8_t regime;
	uint16_t duration;
	uint16_t period;
} generator_struct;
static generator_struct generator, *p_generator=&generator;

//структуры протопотоков
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Sync_pt;
//static struct pt_sem manual_pulse;
//статические переменные
volatile static uint32_t st_timer0_millis;
volatile static uint8_t Button0State=MANUAL,Button1State=uS100;
volatile uint32_t microsecond_timer=0, duration_timer=0, period_timer=0, duration_timer_count=0, period_timer_count=0;  
uint32_t st_millis(void);

PT_THREAD(Sync(struct pt *pt))
{
	PT_BEGIN(pt);
	//PT_SEM_SIGNAL(pt, &manual_pulse); //устанавливает 1 в manual_pulse, сигнализируя что кнопки больше не опрашивались в другом пропотоке
	PORTD&=~_BV(OUT_PIN); //устанавливаем 0 на выходе
	if (((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0))
	{
		PORTD|=_BV(OUT_PIN); //устанавливаем 1 на выходе
		_delay_us(5); //держим 1 на пине 80 микросекунд
		PORTD&=~_BV(OUT_PIN); //сбрасываем выход в 0
		_delay_ms(10); //задержка перед следующим срабатыванием
	}
	PT_EXIT(pt);
	PT_END(pt);
}
PT_THREAD(Switch(struct pt *pt))
{
	static volatile uint32_t switch_timer=0; 
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//запуск протопотока каждые 10мсек
	if (Button0State==MANUAL) 
	{
		
		PT_SPAWN(pt, &Sync_pt, Sync(&Sync_pt));//вызываем дочерний протопоток ручного или синхро запуска
		//Макс частота нажатия кнопки ~2Гц
		PORTD|=_BV(PD2);//зажигаем оба светодиода
		PORTD|=_BV(PD3);
	}
	if (Button0State==Hz1) 
	{
		if (Button1State==uS100)
		{
			duration_timer=5; period_timer=62500;
		}
		else if (Button1State==Duty_50)
		{
			duration_timer=31500; period_timer=62500;
		}
		else 
		{
			duration_timer=56250; period_timer=62500;
		}
		PORTD|=_BV(PD2);//диод 2 горит
		PORTD&=~_BV(PD3);//диод 3 не горит
	} 
	if (Button0State==Hz100)
	{
		if (Button1State==uS100)
		{
			duration_timer=5; period_timer=625;
		}
		else if (Button1State==Duty_50)
		{
			duration_timer=315; period_timer=625;
		}
		else 
		{
			duration_timer=562; period_timer=625;
		}
		PORTD&=~_BV(PD2);//диод 2 не горит
		PORTD|=_BV(PD3);//диод 3  горит
	}
	if (Button0State==Hz1000)
	{
		if (Button1State==uS100)
		{
			duration_timer=5; period_timer=62;
		}
		else if (Button1State==Duty_50)
		{
			duration_timer=31; period_timer=62;
		}
		else
		{
			duration_timer=56; period_timer=62;
		}
		PORTD&=~_BV(PD2);//гасим оба  светодиода
		PORTD&=~_BV(PD3);
	}
	switch_timer=st_millis();
	PT_END(pt);
}
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	//static uint16_t val0, val1=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=80);
	but_timer=st_millis();
	//if (((PIN(BUTTON0_PORT)&(_BV(BUTTON0_PIN)))==0)&&(val0<=10))
	if ((PIN(BUTTON0_PORT)&(_BV(BUTTON0_PIN)))==0)
	{
		if (Button0State==MANUAL) Button0State=Hz1; //при долгом нажатии кн0, проиходит смена периода
		else if (Button0State==Hz1) Button0State=Hz100;
		else if (Button0State==Hz100) Button0State=Hz1000;
		else if (Button0State==Hz1000) Button0State=MANUAL;
			//ButtonState=BUTTON_LONG_ON; - это сейчас не нужно, вдруг пригодится обрабатывать долгие нажатия
			//PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
	}
	/*if (((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0)&&(val1<=10))
	{
		val1++;
	}
	else
	{
		if (val1>9)
		{
			if (Button1State==uS100) Button1State=Duty_50; //долгое нажатие кн1 - смена длительности
			else if (Button1State==Duty_50) Button1State=Duty_90;
			else if (Button1State==Duty_90) Button1State=uS100;
			//ButtonState=BUTTON_LONG_ON; - это сейчас не нужно, вдруг пригодится обрабатывать долгие нажатия
			//PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
		}
		val1=0;
	}*/
	if ((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0)
	{
		if (Button1State==uS100) Button1State=Duty_50; //долгое нажатие кн1 - смена длительности
		else if (Button1State==Duty_50) Button1State=Duty_90;
		else if (Button1State==Duty_90) Button1State=uS100;
		//ButtonState=BUTTON_LONG_ON; - это сейчас не нужно, вдруг пригодится обрабатывать долгие нажатия
		//PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
	}
	PT_END(pt);
}
ISR(TIMER0_COMPA_vect)
{
	if (Button0State!=MANUAL)
	{
		if (duration_timer_count<duration_timer) duration_timer_count++;
		else (PORTD&=(~_BV(OUT_PIN))); //сбросить пин в 0
		if (period_timer_count<period_timer) period_timer_count++;
		else {
			PORTD|=(_BV(OUT_PIN)); //установить пин 1
			period_timer_count=0;
			duration_timer_count=0;
		}
	}
	microsecond_timer++;
	if (microsecond_timer>=Ms1) {
		st_timer0_millis++;
		microsecond_timer=0;
	}
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

int main(void)
{

	DDRD=0b00011110; //PD6 - button1, PD5 - button0, PD4 - OUT, PD3 - LED1, PD2- LED0, PD1 -TX, PD0 - RX
	DDRB=0b11111111; //all pins on portb are outputs
	PORTD=0b01100000;//100k pull-up PD6, PD5
	PORTB=0;
	// Set prescaler to 64
	//TCCR0 |= (_BV(CS01) | _BV(CS00));
	// Enable interrupt
	//TIMSK |= _BV(TOIE0) | _BV(OCIE1A);
	// Set default value
	TCCR0A=0b00000010;//CTC operation of timer
	TCCR0B=0b00000001;//Timer count from clk with no prescaler
	TCNT0 = 0; //обнуляем счетчик таймера
	OCR0A = 100;//прерывание каждые 100тиков, то есть каждые 25мкс
	TIMSK |= _BV(OCIE0A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	/*//set timer 1 for PWM
	TCCR1A=0b10000001; //переключение oc1A по событие на таймере, oc1b льключен, WGM10=1, 8 бит таймер
	TCCR1B=0b00001001; //clocked from CLK=8MHZ WGM12=1
	OCR1AH=0;
	OCR1AL=127;//50% ШИМ 
	TCNT1=0;
	TIMSK=0;
	*/
	//PT_SEM_INIT(&manual_pulse, NOPULSE);
	
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	//PT_INIT(&Sync_pt);

	wdt_reset(); //сбрасываем собаку на всякий пожарный
	wdt_enable(WDTO_2S); //запускаем собаку с перидом 2с
	
	sei();

    while(1)
    {
        PT_SCHEDULE(Buttons(&Buttons_pt));
		PT_SCHEDULE(Switch(&Switch_pt));
		wdt_reset(); //переодически сбрасываем собаку чтобы не улетететь в ресет
	 }
}


//В этой версии полностью работает периодический таймер, ручной все портит
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

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>


/*Макрооредения*/
//Макроопределения входов-выходов

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
#define DISCONETC_TIMER_FROM_PIN TCCR1A &= ~_BV(COM1A1)
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
volatile static uint32_t st_timer0_millis=0;
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
	if (!(BUT0_PORT_PIN&(_BV(BUT0))))//((BUT0_PORT_PIN&(_BV(BUT0)))==0)
	{
		LED0_ON;
		OUT_ON; //устанавливаем 1 на выходе
		_delay_us(5); //держим 1 на пине 80 микросекунд
		OUT_OFF; //сбрасываем выход в 0
		_delay_ms(500); //задержка перед следующим срабатыванием
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
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=120);
	but_timer=st_millis();
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
	}
	if (!(BUT0_PORT_PIN&(_BV(BUT0))))
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
	//uint32_t dur=0;
	static volatile uint32_t switch_timer=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//запуск протопотока каждые 10мсек
	switch_timer=st_millis();
	if ((p_generator->regime==GEN_PERIODIC)&&(p_generator->state==GEN_OFF))
	{
		CLEAR_TCCR1B;
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
	else if (p_generator->regime==GEN_UART)
	{
		//тут что-то будет :) можно прямо здесь написать работу от уарта, а можно в отдельно протопотоке
	}
	/*else 
	{
		GENERATOR_OFF;
		DISCONETC_TIMER_FROM_PIN;//Отключаем OC1A от PB3, включаем управление GPIO
		//p_generator->regime=GEN_MANUAL; //на всякий пожарный, если режим генератора свалится в что-то неизвестное,
		//то попадет сюда и выставит ручной режим
		p_generator->state=GEN_ON;
		PT_SPAWN(pt, &Sync_pt, Sync(&Sync_pt));//вызываем дочерний протопоток ручного или синхро запуска
		//Макс частота нажатия кнопки ~2Гц
		LED0_OFF;
		LED1_OFF;
	}*/
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
	//volatile uint16_t test=0;
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
	TCNT1=0;
	/*CONNECT_TIMER_TO_PIN;
	CLEAR_TCCR1B;
	TCCR1B|=PRESCALER_256_MASK;
	OCR1=100;
	ICR1=1000;
	GENERATOR_ON;*/
	
	TIMSK |= _BV(OCIE0A);//разрешаем прерывание по совпадению TCNT0 с OCR0A
	
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
	//test2=((7*(uint32_t)31250));
	//test=(uint16_t)test2;
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
