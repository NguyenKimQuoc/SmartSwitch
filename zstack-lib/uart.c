#include "ZComDef.h"
#include "onboard.h"
#include "uart.h"

void UART_Init(void)
{
  PERCFG |= BV(1); 
  U1CSR |= (1<<7);
  
  U1GCR = 11;	// 115200 Baud
  U1BAUD = 216;	
}

void UART_Transmit(char data)
{
  U1DBUF = data;
  while (U1CSR & (1<<0)); 
}

void UART_String(const char *s)
{
  while (*s)
  {
    UART_Transmit(*s++);
  }
  UART_Transmit('\r');
  UART_Transmit('\n');
}