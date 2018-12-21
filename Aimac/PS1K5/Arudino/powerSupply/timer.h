/*
 * timer.h
 *
 * Created: 14-10-2018 17:28:01
 *  Author: maaply
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

#include "config.h"


void TimerInit( void );
void timer1Init( void );



#endif /* TIMER_H_ */
