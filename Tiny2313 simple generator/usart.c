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
#include "usart.h"

//������� ��� ���������� � ���������� ���������� usart`a
#define EnableRxInt()   UCSRB |= (1<<RXCIE);
#define DisableRxInt()  UCSRB &= (~(1<<RXCIE));
#define EnableTxInt()   UCSRB |= (1<<TXCIE);
#define DisableTxInt()  UCSRB &= (~(1<<TXCIE));

//���������� �����
static volatile unsigned char usartTxBuf[SIZE_BUF];
static unsigned char txBufTail = 0;
static unsigned char txBufHead = 0;
static volatile unsigned char txCount = 0;

//�������� �����
static volatile unsigned char usartRxBuf[SIZE_BUF];
static unsigned char rxBufTail = 0;
static unsigned char rxBufHead = 0;
static volatile unsigned char rxCount = 0;

//������������� usart`a
void USART_Init(void)
{
  UBRRH = UBRRH_VALUE;
  UBRRL = UBRRL_VALUE;
  #if USE_2X
   UCSRA |= (1 << U2X);
  #else
   UCSRA &= (~(1 << U2X));
  #endif
  UCSRB = (1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN); //����. ������ ��� ������ � ��������, ���� ������, ���� ��������.
  UCSRC = (1<<UMSEL)|(1<<UCSZ1)|(1<<UCSZ0); //������ ����� 8 ��������
}

//______________________________________________________________________________
//���������� ����������� �������� ����������� ������
unsigned char USART_GetTxCount(void)
{
  return txCount;  
}

//"�������" ���������� �����
void USART_FlushTxBuf(void)
{
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
}

//�������� ������ � �����, ���������� ������ ��������
void USART_PutChar(unsigned char sym)
{
  //���� ������ usart �������� � ��� ������ ������
  //����� ��� ����� � ������� UDR
  if(((UCSRA & (1<<UDRE)) != 0) && (txCount == 0)) UDR = sym;
  else {
    if (txCount < SIZE_BUF){    //���� � ������ ��� ���� �����
      usartTxBuf[txBufTail] = sym; //�������� � ���� ������
      txCount++;                   //�������������� ������� ��������
      txBufTail++;                 //� ������ ������ ������
      if (txBufTail == SIZE_BUF) txBufTail = 0;
    }
  }
}

//������� ���������� ������ �� usart`�
void USART_SendStr(char * data)
{
  unsigned char sym;
  while(*data){
    sym = *data++;
    USART_PutChar(sym);
  }
}

//���������� ���������� �� ���������� �������� 
ISR(USART_TX_vect)
{
  if (txCount > 0){              //���� ����� �� ������
    UDR = usartTxBuf[txBufHead]; //���������� � UDR ������ �� ������
    txCount--;                   //��������� ������� ��������
    txBufHead++;                 //�������������� ������ ������ ������
    if (txBufHead == SIZE_BUF) txBufHead = 0;
  } 
} 

//______________________________________________________________________________
//���������� ����������� �������� ����������� � �������� ������
unsigned char USART_GetRxCount(void)
{
  return rxCount;  
}

//"�������" �������� �����
void USART_FlushRxBuf(void)
{
  DisableRxInt(); //��������� ���������� �� ���������� ������
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
  EnableRxInt();
}

//������ ������
unsigned char USART_GetChar(void)
{
  unsigned char sym;
  if (rxCount > 0){                     //���� �������� ����� �� ������  
    sym = usartRxBuf[rxBufHead];        //��������� �� ���� ������    
    rxCount--;                          //��������� ������� ��������
    rxBufHead++;                        //���������������� ������ ������ ������  
    if (rxBufHead == SIZE_BUF) rxBufHead = 0;
    return sym;                         //������� ����������� ������
  }
  return 0;
}


//���������� �� ���������� ������
ISR(USART_RX_vect) 
{
  if (rxCount < SIZE_BUF){                //���� � ������ ��� ���� �����                     
      usartRxBuf[rxBufTail] = UDR;        //������� ������ �� UDR � �����
      rxBufTail++;                             //��������� ������ ������ ��������� ������ 
      if (rxBufTail == SIZE_BUF) rxBufTail = 0;  
      rxCount++;                                 //��������� ������� �������� ��������
    }
} 

