#include <stm8s.h>
#include "timer_3.h"
#include "stdio.h"

uint8_t TIM3_Timer500ms = 4;

TM3_CallBack_t TIM3_100ms_handler = NULL;
TM3_CallBack_t TIM3_500ms_handler = NULL;

void Time3_Init(void) {
  // Interval = 8us*12500 = 100ms
  ITC_SetSoftwarePriority(15,ITC_PRIORITYLEVEL_3);
  TIM3_TimeBaseInit(TIM3_PRESCALER_128, 12499);
  TIM3_PrescalerConfig(TIM3_PRESCALER_128, TIM3_PSCRELOADMODE_IMMEDIATE);
  TIM3_ARRPreloadConfig(ENABLE);
  TIM3_ClearFlag(TIM3_FLAG_UPDATE);
  TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
  TIM3_Cmd(ENABLE);
  GPIO_Init(GPIOD , GPIO_PIN_4 , GPIO_MODE_OUT_PP_LOW_SLOW);
}

INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
  TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
  if( TIM3_100ms_handler ) (*TIM3_100ms_handler)();
  
  if(TIM3_Timer500ms > 0) {
    TIM3_Timer500ms--;
  } else {
    // Reset and go
    TIM3_Timer500ms = 4;
    if( TIM3_500ms_handler ) (*TIM3_500ms_handler)();
  }
}
