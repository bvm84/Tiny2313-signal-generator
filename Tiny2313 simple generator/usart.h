//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: WINAVR
//
//  Description.: драйвер USART/UART с кольцевым буфером
//
//  Data........: 11.01.10 
//
//***************************************************************************
#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
	#define F_CPU 8000000     //задаем частоту кварца
#endif
#define BAUD 9600         //требуемую скорость обмена
#include <util/setbaud.h> //здесь лежат макросы для расчета

#define SIZE_BUF 3      //и размер кольцевых буферов - <255

void USART_Init(void); //инициализация usart`a
unsigned char USART_GetTxCount(void); //взять число символов передающего буфера
void USART_FlushTxBuf(void); //очистить передающий буфер
void USART_PutChar(unsigned char sym); //положить символ в буфер
void USART_SendStr(char * data); //послать строку по usart`у
unsigned char USART_GetRxCount(void); //взять число символов в приемном буфере
void USART_FlushRxBuf(void); //очистить приемный буфер
unsigned char USART_GetChar(void); //прочитать приемный буфер usart`a 

#endif //USART_H