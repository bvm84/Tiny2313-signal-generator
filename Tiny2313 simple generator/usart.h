//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: WINAVR
//
//  Description.: ������� USART/UART � ��������� �������
//
//  Data........: 11.01.10 
//
//***************************************************************************
#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
	#define F_CPU 8000000     //������ ������� ������
#endif
#define BAUD 9600         //��������� �������� ������
#include <util/setbaud.h> //����� ����� ������� ��� �������

#define SIZE_BUF 3      //� ������ ��������� ������� - <255

void USART_Init(void); //������������� usart`a
unsigned char USART_GetTxCount(void); //����� ����� �������� ����������� ������
void USART_FlushTxBuf(void); //�������� ���������� �����
void USART_PutChar(unsigned char sym); //�������� ������ � �����
void USART_SendStr(char * data); //������� ������ �� usart`�
unsigned char USART_GetRxCount(void); //����� ����� �������� � �������� ������
void USART_FlushRxBuf(void); //�������� �������� �����
unsigned char USART_GetChar(void); //��������� �������� ����� usart`a 

#endif //USART_H