#include "TIMER_drivers.h"
#include "GPIO_drivers.h"

/** \fn Timer_Clock
  * \brief Allumer ou éteindre l'horloge associée à un timer
	* \param TIM_TypeDef* timer : Le timer que l'on veut utiliser
	* \param TIMER_CLOCK state : Allumer ou eteindre (TIMER_CLOCK_ENABLE ou TIMER_CLOCK_DISABLE)
	**/
void Timer_Clock(TIM_TypeDef* timer, TIMER_CLOCK state)
{
	if(state != TIMER_CLOCK_ENABLE && state != TIMER_CLOCK_DISABLE) return;
	
	int d = ((unsigned int)timer - APB1PERIPH_BASE)/0x400;
	
	if(state == TIMER_CLOCK_ENABLE) RCC->APB1ENR |= (1 << d);
	else RCC->APB1ENR &= ~(1 << d);
}

/** \fn Timer_Init
  * \brief Configurer les valeurs d'autoreload et de prescaler du timer
	* \param TIM_TypeDef* timer : Le timer que l'on veut utiliser
	* \param uint16_t prescaler : Prescaler du timer (16 bits)
	* \param uint16_t autoreload : Autoreload du timer (16 bits)
	**/
void Timer_Init(TIM_TypeDef* timer, uint16_t prescaler, uint16_t autoreload)
{
	timer->ARR =  autoreload;
	timer->PSC = prescaler;
}

/** \fn Timer_Start
  * \brief Lancer le timer
	* \param TIM_TypeDef* timer : Le timer que l'on veut lancer
	**/
void Timer_Start(TIM_TypeDef* timer)
{
	timer->CR1 |= TIM_CR1_CEN;
}

/** \fn Timer_Stop
  * \brief Arreter le timer
	* \param TIM_TypeDef* timer : Le timer que l'on veut arreter
	**/
void Timer_Stop(TIM_TypeDef* timer)
{
	timer->CR1 &= ~TIM_CR1_CEN;
}

/** \fn Timer_EnableInterrupt
  * \brief Configurer l'interruption sur un timer
	* \param TIM_TypeDef* timer : Le timer que l'on veut utiliser en mode interruption
	* \param uint8_t priority : la priorité de l'interruption (4 bits)
	**/
void Timer_EnableInterrupt(TIM_TypeDef* timer, uint8_t priority)
{
	timer->DIER = TIM_DIER_UIE;
	
	int d = ((unsigned int)timer - APB1PERIPH_BASE)/0x400;
	
	NVIC->ISER[0] |= (1 << (d+28));
	NVIC->IP[d+28] |= (priority << 4);
}

/** \fn TIM2_IRQHandler
  * \brief Fontion d'interruption du timer 2
	**/
void TIM2_IRQHandler(void) { 
	TIM2->SR &= ~TIM_SR_UIF;
	GPIO_Toggle(GPIOA, 5);
}