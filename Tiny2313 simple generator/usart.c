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
#include "usart.h"

//макросы для разрешения и запрещения прерываний usart`a
#define EnableRxInt()   UCSRB |= (1<<RXCIE);
#define DisableRxInt()  UCSRB &= (~(1<<RXCIE));
#define EnableTxInt()   UCSRB |= (1<<TXCIE);
#define DisableTxInt()  UCSRB &= (~(1<<TXCIE));

//передающий буфер
static volatile unsigned char usartTxBuf[SIZE_BUF];
static unsigned char txBufTail = 0;
static unsigned char txBufHead = 0;
static volatile unsigned char txCount = 0;

//приемный буфер
static volatile unsigned char usartRxBuf[SIZE_BUF];
static unsigned char rxBufTail = 0;
static unsigned char rxBufHead = 0;
static volatile unsigned char rxCount = 0;

//инициализация usart`a
void USART_Init(void)
{
  UBRRH = UBRRH_VALUE;
  UBRRL = UBRRL_VALUE;
  #if USE_2X
   UCSRA |= (1 << U2X);
  #else
   UCSRA &= (~(1 << U2X));
  #endif
  UCSRB = (1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN); //разр. прерыв при приеме и передачи, разр приема, разр передачи.
  UCSRC = (1<<UMSEL)|(1<<UCSZ1)|(1<<UCSZ0); //размер слова 8 разрядов
}

//______________________________________________________________________________
//возвращает колличество символов передающего буфера
unsigned char USART_GetTxCount(void)
{
  return txCount;  
}

//"очищает" передающий буфер
void USART_FlushTxBuf(void)
{
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
}

//помещает символ в буфер, инициирует начало передачи
void USART_PutChar(unsigned char sym)
{
  //если модуль usart свободен и это первый символ
  //пишем его прямо в регистр UDR
  if(((UCSRA & (1<<UDRE)) != 0) && (txCount == 0)) UDR = sym;
  else {
    if (txCount < SIZE_BUF){    //если в буфере еще есть место
      usartTxBuf[txBufTail] = sym; //помещаем в него символ
      txCount++;                   //инкрементируем счетчик символов
      txBufTail++;                 //и индекс хвоста буфера
      if (txBufTail == SIZE_BUF) txBufTail = 0;
    }
  }
}

//функция посылающая строку по usart`у
void USART_SendStr(char * data)
{
  unsigned char sym;
  while(*data){
    sym = *data++;
    USART_PutChar(sym);
  }
}

//обработчик прерывания по завершению передачи 
ISR(USART_TX_vect)
{
  if (txCount > 0){              //если буфер не пустой
    UDR = usartTxBuf[txBufHead]; //записываем в UDR символ из буфера
    txCount--;                   //уменьшаем счетчик символов
    txBufHead++;                 //инкрементируем индекс головы буфера
    if (txBufHead == SIZE_BUF) txBufHead = 0;
  } 
} 

//______________________________________________________________________________
//возвращает колличество символов находящихся в приемном буфере
unsigned char USART_GetRxCount(void)
{
  return rxCount;  
}

//"очищает" приемный буфер
void USART_FlushRxBuf(void)
{
  DisableRxInt(); //запрещаем прерывание по заверщению приема
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
  EnableRxInt();
}

//чтение буфера
unsigned char USART_GetChar(void)
{
  unsigned char sym;
  if (rxCount > 0){                     //если приемный буфер не пустой  
    sym = usartRxBuf[rxBufHead];        //прочитать из него символ    
    rxCount--;                          //уменьшить счетчик символов
    rxBufHead++;                        //инкрементировать индекс головы буфера  
    if (rxBufHead == SIZE_BUF) rxBufHead = 0;
    return sym;                         //вернуть прочитанный символ
  }
  return 0;
}


//прерывание по завершению приема
ISR(USART_RX_vect) 
{
  if (rxCount < SIZE_BUF){                //если в буфере еще есть место                     
      usartRxBuf[rxBufTail] = UDR;        //считать символ из UDR в буфер
      rxBufTail++;                             //увеличить индекс хвоста приемного буфера 
      if (rxBufTail == SIZE_BUF) rxBufTail = 0;  
      rxCount++;                                 //увеличить счетчик принятых символов
    }
} 

