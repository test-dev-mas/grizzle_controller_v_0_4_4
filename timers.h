#ifndef _TIMERS_H
#define _TIMERS_H

void init_timer0();
void init_timer1();
void start_timer1();
void init_timer2();         // don't need this, since no isr is needed, will only read/clear TNCT2
void start_timer2();        // set clock bits to start
void stop_timer2();         // clear clock bits to stop
void stop_timer1();
void init_timer3();
void stop_timer3();

#endif