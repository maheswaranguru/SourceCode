/*
 * timer.cpp
 *
 * Created: 14-10-2018 17:27:33
 *  Author: maaply
 */ 


#include "timer.h"

extern bool ledUpdate_f;

/**************************************************************************
Name    : TimerInit
para-1  :
return  :
Notes   : 
**************************************************************************/
 void TimerInit( void )
 {
     TCCR0A = 0x00;
     TCCR0B |= ( (1<<CS02) | (1<<CS00) );        //!< Prrescale for 1024
     TIMSK0 |= (1<< TOIE0);       //!< Interrupt Enable
     GTCCR = 0x00;
     
 
 }

 /**************************************************************************
Name    : TimerInit
para-1  :
return  :
Notes   : 
**************************************************************************/
void timer1Init( void )
{
  TCCR1A = 0x00;
  TCCR1B = (1<<CS12);
  TCCR1C  = 0x00;
  TCNT1L = 0xDC;
  TCNT1H = 0x0B;
  TIMSK1 |= (1<<TOIE1);

  DDRB |= (1<<NANO_LED);
  sei(); 
}

/**************************************************************************
Name : 
para-1 :
return :
Notes : 
 **************************************************************************/
 
 ISR(TIMER1_OVF_vect)
 {         
  PORTB ^= (1<<NANO_LED);
  TCNT1L = 0xDC;
  TCNT1H = 0x0B;
 
    
 }
