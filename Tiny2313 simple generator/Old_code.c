/*
 * Tiny2313 simple generator.c
 * Ver 2.0
 * Created: 13.01.2016 12:37:18
 * Author : Brendel Vadim
  ���� ���������:
  
  � �������� ������ ���������� ������������ PD4
  PD5, PD6 - ������ ������ ������
  PD3, PD2 - ���������� ������ ������
  PD1, PD0 - UART, �������� ����� ������������ � �������
  ���� �������� �� ����������� ���������� �� ������� 4���.
  ������ �1- ��������� �������� �� ���������� ������ �����������, ������������ �����������.
  ������ T0 - ������������ ��� ��������� ���������.
  
  ��������� ������ �������:
  ��������� - ���/����
  ����� - ������, �������������, UART
  ������������ - uint
  ������ - uint
  
  ����������1 (1����) ��������� ������ � ������������� ����� ������, ��������� �������� � ���������.
  �����������2 (10����) ������������� ������ T0 �������� ������ ������ � �������� ��������� � ������ ��������� ������.
  � ������ ������ �������� �������� ���������� ������� ��������� ������ � �������� ����� ������ �� ����.
  ��������3 (5����) �������� �� UART. ���� ����� S, ���� ��� ������ ���������� ������� ����� ����������1 �
  ���������� 2. �������� �������� ������������ � �������, ��������� �� �� ������������, ������������� ������,
  ���������. ��� ������� ����� P, ����� ������, ������ ���������� � ������ �����.
 
 
  ����������2:
  ���� ������0=0 - ������ �����, (���������� 11) ����������� ������ 80��� ��� ������� ������1
  ���� ������0= 1 � ������1=0 - ������� 1��(��������� 10),  ������������ 80���, 62500�����, 5 ����� - ���������� 0,0001 (0,01%)
  ���� ������0= 2 � ������1=1 - ������� 1��(��������� 10), ������������ 0,5�, 62500�����, 31500 ����� - ���������� 0,5
  ���� ������0= 3 � ������1=2 - ������� 1��(��������� 10), ������������ 0,9�, 62500�����, 56250 ����� - ���������� 0,9
  ���� ������0= 1 � ������1=0 - ������� 100��(��������� 01), ������������ 80���, 625�����, 5 ����� - ���������� 0,01 (1%)
  ���� ������0= 2 � ������1=1 - ������� 100��(��������� 01), ������������ 0,05�, 625�����, 615 ����� - ���������� 0,5
  ���� ������0= 3 � ������1=2 - ������� 100��(��������� 01), ������������ 0,09�, 625�����, 562 ����� - ���������� 0,9
  ���� ������0= 1 � ������1=0 - ������� 1000��(��������� 00), ������������ 80���, 62�����, 5 ����� - ���������� 0,1 (10%)
  ���� ������0= 2 � ������1=1 - ������� 1000��(��������� 00), ������������ 496���, 62�����, 31 ����� - ���������� 0,5
  ���� ������0= 3 � ������1=2 - ������� 1000��(��������� 00), ������������ 896���, 62�����, 56 ����� - ���������� 0,9
 
  
 */ 

#include <avr/io.h>
#include <util/atomic.h>
#include <pt.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <pt-sem.h>

/*�������������*/
//���������������� ������-�������

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

//�������������� �������
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

//��������������� ���������������
#define B(bit_no)         (1 << (bit_no))
#define CB(reg, bit_no)   (reg) &= ~B(bit_no)
#define SB(reg, bit_no)   (reg) |= B(bit_no)
#define VB(reg, bit_no)   ( (reg) & B(bit_no) )
#define TB(reg, bit_no)   (reg) ^= B(bit_no)
//������� ��� ������ �� ������������ ���������
#define LED0_ON LED0_PORT|=_BV(LED0)
#define LED0_OFF LED0_PORT&=~_BV(LED0)
#define LED1_ON LED1_PORT|=_BV(LED1)
#define LED1_OFF LED1_PORT&=~_BV(LED1)
//��� �� �������, �������, ���������, ��������, ������������
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define ABS(x) ((x) < 0 ? -(x) : (x))
/*�������������*/

/*���������� ���������� ����������*/
//��������� ������ ������� 

typedef struct {
	uint8_t state;
	uint8_t regime;
	uint16_t duration;
	uint16_t period;
} generator_struct;
static generator_struct generator, *p_generator=&generator;

//��������� ������������
static struct pt Buttons_pt;
static struct pt Switch_pt;
static struct pt Sync_pt;
//static struct pt_sem manual_pulse;
//����������� ����������
volatile static uint32_t st_timer0_millis;
volatile static uint8_t Button0State=MANUAL,Button1State=uS100;
volatile uint32_t microsecond_timer=0, duration_timer=0, period_timer=0, duration_timer_count=0, period_timer_count=0;  
uint32_t st_millis(void);

PT_THREAD(Sync(struct pt *pt))
{
	PT_BEGIN(pt);
	//PT_SEM_SIGNAL(pt, &manual_pulse); //������������� 1 � manual_pulse, ������������ ��� ������ ������ �� ������������ � ������ ���������
	PORTD&=~_BV(OUT_PIN); //������������� 0 �� ������
	if (((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0))
	{
		PORTD|=_BV(OUT_PIN); //������������� 1 �� ������
		_delay_us(5); //������ 1 �� ���� 80 �����������
		PORTD&=~_BV(OUT_PIN); //���������� ����� � 0
		_delay_ms(10); //�������� ����� ��������� �������������
	}
	PT_EXIT(pt);
	PT_END(pt);
}
PT_THREAD(Switch(struct pt *pt))
{
	static volatile uint32_t switch_timer=0; 
	PT_BEGIN(pt);
	PT_WAIT_UNTIL(pt,(st_millis()-switch_timer)>=10);//������ ����������� ������ 10����
	if (Button0State==MANUAL) 
	{
		
		PT_SPAWN(pt, &Sync_pt, Sync(&Sync_pt));//�������� �������� ���������� ������� ��� ������ �������
		//���� ������� ������� ������ ~2��
		PORTD|=_BV(PD2);//�������� ��� ����������
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
		PORTD|=_BV(PD2);//���� 2 �����
		PORTD&=~_BV(PD3);//���� 3 �� �����
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
		PORTD&=~_BV(PD2);//���� 2 �� �����
		PORTD|=_BV(PD3);//���� 3  �����
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
		PORTD&=~_BV(PD2);//����� ���  ����������
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
		if (Button0State==MANUAL) Button0State=Hz1; //��� ������ ������� ��0, ��������� ����� �������
		else if (Button0State==Hz1) Button0State=Hz100;
		else if (Button0State==Hz100) Button0State=Hz1000;
		else if (Button0State==Hz1000) Button0State=MANUAL;
			//ButtonState=BUTTON_LONG_ON; - ��� ������ �� �����, ����� ���������� ������������ ������ �������
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
			if (Button1State==uS100) Button1State=Duty_50; //������ ������� ��1 - ����� ������������
			else if (Button1State==Duty_50) Button1State=Duty_90;
			else if (Button1State==Duty_90) Button1State=uS100;
			//ButtonState=BUTTON_LONG_ON; - ��� ������ �� �����, ����� ���������� ������������ ������ �������
			//PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
		}
		val1=0;
	}*/
	if ((PIN(BUTTON1_PORT)&(_BV(BUTTON1_PIN)))==0)
	{
		if (Button1State==uS100) Button1State=Duty_50; //������ ������� ��1 - ����� ������������
		else if (Button1State==Duty_50) Button1State=Duty_90;
		else if (Button1State==Duty_90) Button1State=uS100;
		//ButtonState=BUTTON_LONG_ON; - ��� ������ �� �����, ����� ���������� ������������ ������ �������
		//PT_WAIT_UNTIL(pt,(st_millis()-but_timer)>=1000);
	}
	PT_END(pt);
}
ISR(TIMER0_COMPA_vect)
{
	if (Button0State!=MANUAL)
	{
		if (duration_timer_count<duration_timer) duration_timer_count++;
		else (PORTD&=(~_BV(OUT_PIN))); //�������� ��� � 0
		if (period_timer_count<period_timer) period_timer_count++;
		else {
			PORTD|=(_BV(OUT_PIN)); //���������� ��� 1
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
	TCNT0 = 0; //�������� ������� �������
	OCR0A = 100;//���������� ������ 100�����, �� ���� ������ 25���
	TIMSK |= _BV(OCIE0A);//��������� ���������� �� ���������� TCNT0 � OCR0A
	/*//set timer 1 for PWM
	TCCR1A=0b10000001; //������������ oc1A �� ������� �� �������, oc1b ��������, WGM10=1, 8 ��� ������
	TCCR1B=0b00001001; //clocked from CLK=8MHZ WGM12=1
	OCR1AH=0;
	OCR1AL=127;//50% ��� 
	TCNT1=0;
	TIMSK=0;
	*/
	//PT_SEM_INIT(&manual_pulse, NOPULSE);
	
	PT_INIT(&Buttons_pt);
	PT_INIT(&Switch_pt);
	//PT_INIT(&Sync_pt);

	wdt_reset(); //���������� ������ �� ������ ��������
	wdt_enable(WDTO_2S); //��������� ������ � ������� 2�
	
	sei();

    while(1)
    {
        PT_SCHEDULE(Buttons(&Buttons_pt));
		PT_SCHEDULE(Switch(&Switch_pt));
		wdt_reset(); //������������ ���������� ������ ����� �� ��������� � �����
	 }
}
