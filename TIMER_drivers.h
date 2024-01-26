#ifndef DEF_TIMER_DRIVERS
#define DEF_TIMER_DRIVERS

#include "stm32f10x.h"

typedef enum TIMER_CLOCK TIMER_CLOCK;
enum TIMER_CLOCK
{
    TIMER_CLOCK_ENABLE, TIMER_CLOCK_DISABLE
};

void Timer_Init(TIM_TypeDef* timer, uint16_t prescaler, uint16_t period);
void Timer_Start(TIM_TypeDef* timer);
void Timer_Clock(TIM_TypeDef* timer, TIMER_CLOCK state);
void Timer_Stop(TIM_TypeDef* timer);
void Timer_EnableInterrupt(TIM_TypeDef* timer, uint8_t priority);
void TIM2_IRQHandler(void);

#endif // DEF_TIMER_DRIVERS