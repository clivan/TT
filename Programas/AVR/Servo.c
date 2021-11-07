#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <math.h>

#ifndef BAUD
#define BAUD 115200
#endif
#ifndef F_CPU
#define F_CPU 16000000
#endif
#define USART_HAS_DATA bit_is_set(UCSR0A, RXC0)
#define USART_READY bit_is_set(UCSR0A, UDRE0)
#define PULSO_MIN 1000
#define PULSO_MAX 2000
#define PULSO_MID 1500

void initUSART(void);
uint8_t recibirByte(void);
void initTimer1Servo(void);
uint16_t map(uint16_t X, uint16_t X0, uint16_t X1, uint16_t Y0, uint16_t Y1);

void main(void)
{
  initUSART();
  initTimer1Servo();
  OCR1A=PULSO_MID;
  OCR1B=PULSO_MID;
  _delay_ms(5000);
  uint8_t i;
  uint16_t vel=PULSO_MID;
  uint16_t giro=PULSO_MID;
  uint16_t a=0;
  while(1)
    {
      for (i=0; i<4; i++)
	{
	  a=recibirByte()*pow(10, i)+a; 
	}
      if (a>=1000)
	{
	  vel=a;
	  giro=giro;
	}
      else
	{
	  vel=vel;
	  giro=(uint8_t)map(a, 0, 180, PULSO_MIN, PULSO_MAX);
	}
    }
  OCR1A=vel;
  OCR1B=giro;
}

void initUSART(void)
{
  UBRR0H=UBRRH_VALUE;
  UBRR0L=UBRRL_VALUE;
  UCSR0A|=(1<<U2X0);
  UCSR0B=(1<<TXEN0)|(1<<RXEN0);
  UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);
}

uint8_t recibirByte(void) 
{
  loop_until_bit_is_set(UCSR0A, RXC0);
  return UDR0;
}

void initTimer1Servo(void) 
{
  TCCR1A|=(1<<WGM11);
  TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS10);
  ICR1=20000;
  TCCR1A|=(1<<COM1A1);
  DDRB|=(1<<PB1);
}

uint16_t map(uint16_t X, uint16_t X0, uint16_t X1, uint16_t Y0, uint16_t Y1)
{
  return (X-X0)*(Y1-Y0)/(X1-X0)+Y0;
}
