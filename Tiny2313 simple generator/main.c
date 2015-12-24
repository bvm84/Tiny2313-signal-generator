/*
 * Tiny2313 simple generator.c
 *
 * Created: 23.12.2015 12:37:18
 * Author : ВАдим
  Суть программы:
  
  В качестве выхода генератора используется PD4
  PD5, PD6 - кнопки выбора режима
  PD3, PD2 - светодиоды режима работы
  PD1, PD0 - UART, возможно будет использовать в будущем
  Ядро крутится от внутреннего генератора на частоте 4МГц.
  Прерывание на переполнение таймера происходит каждые 16мкс.
  Проверяется значение установленное длительности, если меньше, увеличивает счетчик скважности
  Если больше, то сбрасывает пин на ноль
  Проверяется значение установки частоты, если меньше увеличиваем счетчик чатоты
  Если больше устанавливаем на пине 1, сбрасываем счетчик частоты, сбрасываем счетчик скважности
  Увеличиваем счетчик микросекунд 
  если набралось на 1мсек, то увеличивает милисекундный счетчик, сбрасывает микросекундный
 
  Протопоток установки частоты и скважности запускается каждые 10мсек
  Если кнопка0=0 - ручной режим, (светодиоды 00) выплевываем импльс 80мкс при нажатии кнопки1
  Если кнопка0= 1 и кнопка1=0 - частота 1Гц(светодиды 01),  длительность 80мкс, 62500тиков, 5 тиков - скважность 0,0001 (0,01%)
  Если кнопка0= 2 и кнопка1=1 - частота 1Гц(светодиды 01), длительность 0,5с, 62500тиков, 31500 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частота 1Гц(светодиды 01), длительность 0,9с, 62500тиков, 56250 тиков - скважность 0,9
  Если кнопка0= 1 и кнопка1=0 - частота 100Гц(светодиды 10), длительность 80мкс, 625тиков, 5 тиков - скважность 0,01 (1%)
  Если кнопка0= 2 и кнопка1=1 - частота 100Гц(светодиды 10), длительность 0,05с, 625тиков, 615 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частота 100ГЦ(светодиды 10), длительность 0,09с, 625тиков, 562 тиков - скважность 0,9
  Если кнопка0= 1 и кнопка1=0 - частоты 1000ГЦ(светодиды 11), длительность 80мкс, 62тиков, 5 тиков - скважность 0,1 (10%)
  Если кнопка0= 2 и кнопка1=1 - частоты 1000ГЦ(светодиды 11), длительность 496мкс, 62тиков, 31 тиков - скважность 0,5
  Если кнопка0= 3 и кнопка1=2 - частоты 1000ГЦ(светодиды 11), длительность 896мкс, 62тиков, 56 тиков - скважность 0,9
  
  Протопоток считывания кнопок запускается каждую милисекунду
  
  Альтернативно напрашивается такое решение:
    Если кнопка0=0 - ручной режим, (светодиоды 00) выплевываем импльс 80мкс при нажатии кнопки1
    Если кнопка0= 1 и кнопка1=0 - частота 1Гц(светодиды 01),
	Настраивается делитель таймера на 1Гц и можно менять скважность (16бит - 1/2^16=15мкс). Но менять скважность будет трудно
	очень много крутить ручку пиридется. Такое решение удобно при управлении от UART, кидаешь два числа к порт и имеешь импульсы
	нужной частоты и длительности. Так как в данном приборе более всего нужны короткие импульсы при малой частоте, а все
	остальное просто опция, оставлю предыдущий алгоритм, он мне видится более подходящим под задачу, хотя и менее универсальным.
    Плюс такого решения - вся работа по генерации импульсов на ножке лежит на таймере, а протопотоки только готовят
	задержки к нему.
	В имеющемся решении ногодрыг обеспечивается доступом к порту, а частота и длительность не аппаратной сврекой регистров
	а счетом импульсов в прерывании. Ресурсов жрет больше и менее точно. 
  
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PIN_(port)  PIN  ## port
#define PIN(port)  PIN_(port)
#define BUTTON0_PORT D
#define BUTTON0_PIN 5
#define BUTTON1_PORT D
#define BUTTON1_PIN 6
#define OUT_PORT D
#define OUT_PIN 4
#define BOUNCE 3
#define BUTTON_LONG_ON 2
#define BUT_MASK 16

#define B(bit_no)         (1 << (bit_no))
#define CB(reg, bit_no)   (reg) &= ~B(bit_no)
#define SB(reg, bit_no)   (reg) |= B(bit_no)
#define VB(reg, bit_no)   ( (reg) & B(bit_no) )
#define TB(reg, bit_no)   (reg) ^= B(bit_no)
//Бит за номером, очистка, установка, проверка, переключение

#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))

#define MANUAL 0
#define Hz1 1
#define Hz100 2
#define Hz1000 3
#define uS100 1
#define Duty_50 2
#define Duty_90 3
#define Ms1 62

#define ST_CTC_HANDMADE 255-210 //45  tick at 3MHz with prescaler 64 gives 22.33us*45=1,0045ms
//структуры протопотоков
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Sync_pt;
//статические переменные
volatile static uint32_t st_timer0_millis;
volatile static uint8_t Button0State=MANUAL,Button1State=uS100;
volatile uint32_t microsecond_timer=0, duration_timer=0, period_timer=0, duration_timer_count=0, period_timer_count=0;  
uint32_t st_millis(void);
PT_THREAD(Sync(struct pt *pt))
{
	static uint16_t val2=0;
	PT_BEGIN(pt);
	TIMSK &= ~_BV(TOIE0); //отрубаем таймер
	PIN(OUT_PORT)&=~_BV(OUT_PIN); //устанавливаем 0 на выходе
	while (val2<=5)
	{
		if (((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0)) val2++;
		else break;
		_delay_ms(10);
	}
	if (val2>4)
	{
		PIN(OUT_PORT)|=_BV(OUT_PIN); //устанавливаем 1 на выходе
		_delay_us(80); //держим 1 на пине 80 микросекунд
		PIN(OUT_PORT)&=~_BV(OUT_PIN); //сбрасываем выход в 0
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
		TIMSK &= ~_BV(TOIE0);//отрубаем таймер
		PT_SPAWN(pt, &Sync_pt, (int)Sync(&Sync_pt));//вызываем дочерний протопоток ручного или синхро запуска
	}//запустить дочерний протопоток, который сканирует кнопку и выплевывает исмпульс
	if (Button0State==Hz1 && Button1State==uS100) 
	{
		duration_timer=5; period_timer=62500; TIMSK |=_BV(TOIE0); //запускаем таймер
	} 
	if (Button0State==Hz1 && Button1State==Duty_50)
	{
		duration_timer=31500; period_timer=62500;
	}
	if (Button0State==Hz1 && Button1State==Duty_90)
	{
		duration_timer=56250; period_timer=62500;
	}
	if (Button0State==Hz100 && Button1State==uS100)
	{
		duration_timer=5; period_timer=625;
	}
	if (Button0State==Hz100 && Button1State==Duty_50)
	{
		duration_timer=315; period_timer=625;
	}
	if (Button0State==Hz100 && Button1State==Duty_90)
	{
		duration_timer=562; period_timer=625;
	}
	if (Button0State==Hz1000 && Button1State==uS100)
	{
		duration_timer=5; period_timer=62;
	}
	if (Button0State==Hz1000 && Button1State==Duty_50)
	{
		duration_timer=31; period_timer=62;
	}
	if (Button0State==Hz1000 && Button1State==Duty_90)
	{
		duration_timer=56; period_timer=62;
	}
	switch_timer=st_millis();
	PT_END(pt);
}
PT_THREAD(Buttons(struct pt *pt))
{
	static uint32_t but_timer=0;
	static uint16_t val0, val1=0;
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt, (st_millis()-but_timer)>=1);
	but_timer=st_millis();
	if (((PIN(BUTTON0_PORT)&(_BV(BUTTON0_PIN)))==0)&&(val0<=1000))
	{
		val0++;
	}
	else
	{
		if (val0>900)
		{
			//ButtonState=BUTTON_LONG_ON; - это сейчас не нужно, вдруг пригодится обрабатывать долгие нажатия
			PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
		}
		else if (val0>=5)
		{
			if (Button0State==MANUAL) Button0State=Hz1;
			else if (Button0State==Hz1) Button0State=Hz100;
			else if (Button0State==Hz100) Button0State=Hz1000;
			else if (Button0State==Hz1000) Button0State=MANUAL;
			//button_change_state();
		}
		val0=0;
	}
	if (((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0)&&(val1<=1000))
	{
		val1++;
	}
	else
	{
		if (val1>900)
		{
			//ButtonState=BUTTON_LONG_ON; - это сейчас не нужно, вдруг пригодится обрабатывать долгие нажатия
			PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
		}
		else if (val1>=5)
		{
			if (Button1State==uS100) Button1State=Duty_50;
			else if (Button1State==Duty_50) Button1State=Duty_90;
			else if (Button0State==Duty_90) Button0State=uS100;
			//button_change_state();
		}
		val1=0;
	}
	PT_END(pt);
}
ISR(TIMER0_OVF_vect)
{
	if (duration_timer_count<duration_timer) duration_timer_count++;
	else (PIN(OUT_PORT)&=(~_BV(OUT_PIN))); //сбросить пин в 0
	if (period_timer_count<period_timer) period_timer_count++;
	else {
		PIN(OUT_PORT)|=(_BV(OUT_PIN)); //установить пин 1
		period_timer_count=0;
		duration_timer_count=0;
	}
	microsecond_timer++;
	if (microsecond_timer==Ms1) {
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

	DDRD=0b11111110; //all pins on portd are outputs, exept PD0 - RX
	DDRB=0b11111111; //all pins on portb are outputs
	PORTD=0;
	PORTB=0;
	// Set prescaler to 64
	//TCCR0 |= (_BV(CS01) | _BV(CS00));
	// Enable interrupt
	//TIMSK |= _BV(TOIE0) | _BV(OCIE1A);
	// Set default value
	TCNT0 = ST_CTC_HANDMADE; //1ms tiks on 3mhz CPU clock
	
	/*//set timer 1 for PWM
	TCCR1A=0b10000001; //переключение oc1A по событие на таймере, oc1b льключен, WGM10=1, 8 бит таймер
	TCCR1B=0b00001001; //clocked from CLK=8MHZ WGM12=1
	OCR1AH=0;
	OCR1AL=127;//50% ШИМ 
	TCNT1=0;
	TIMSK=0;
	*/
	TIMSK |= _BV(TOIE0) | _BV(OCIE1A);
	

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
