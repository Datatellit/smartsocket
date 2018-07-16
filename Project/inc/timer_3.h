#ifndef __TIMER_THREE_H
#define __TIMER_THREE_H

typedef void (*TM3_CallBack_t)();

extern TM3_CallBack_t TIM3_500ms_handler;

// 500ms interupt
void Time3_Init(void);

#endif // __TIMER_TWO_H