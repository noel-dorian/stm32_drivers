#include "GPIO_drivers.h"
#include <stdlib.h>

/** \fn GPIO_Init
  * \brief Initialiser un Pin d'un port en mode Input ou Output
	* \param GPIO_TypeDef* gpioPort : Le port que l'on veut modifier (GPIOA -> GPIOD)
	* \param uint16_t gpioPin : Le Pin à initialiser
	* \param GPIO_MODE mode : Mode du Pin (Entrée ou Sortie)
	**/
void GPIO_Init(GPIO_TypeDef* gpioPort, uint16_t gpioPin, GPIO_MODE mode)
{
	if(gpioPort == NULL) return;
	if(gpioPin < 0 || gpioPin > 15) return;
	
	volatile uint32_t* reg = 0;

	// Sélection du bon registre de configuration (CRL ou CRH)
	if(gpioPin < 8) reg = &gpioPort->CRL;
	else reg = &gpioPort->CRH;
	
	*reg = *reg & ~((unsigned int)0xF << 4*(gpioPin%8)); // Remise à 0 de tous les bits
	
	*reg = *reg | ((unsigned int)mode << 4*(gpioPin%8)); // Configuration de la broche
}

/** \fn GPIO_Clock
  * \brief Allumer ou éteindre l'horloge associée à un port
	* \param GPIO_TypeDef* gpioPort : Le port sur lequel on travaille
	* \param GPIO_CLOCK state : Allumer ou Eteindre l'horloge (GPIO_CLOCK_ENABLE ou GPIO_CLOCK_DISABLE)
	**/
void GPIO_Clock(GPIO_TypeDef* gpioPort, GPIO_CLOCK state)
{
	if(gpioPort == NULL) return;
	
	int d = (((unsigned int)gpioPort) - APB2PERIPH_BASE)/0x400;
	
	if(state == GPIO_CLOCK_ENABLE) RCC->APB2ENR |=  (1<<d) ;
	else if(state == GPIO_CLOCK_DISABLE) RCC->APB2ENR &=  ~(1<<d) ;
}

/** \fn GPIO_Read
  * \brief Lire une broche d'un port
	* \param GPIO_TypeDef* gpioPort : Le port sur lequel on travaille
	* \param uint16_t gpioPin : La broche sur laquelle on veut récupérer la valeur
	**/
uint8_t GPIO_Read(GPIO_TypeDef* gpioPort, uint16_t gpioPin)
{
	if(gpioPin < 0 || gpioPin > 15) return;
	return ((uint8_t) gpioPort->IDR & (1 << gpioPin)) >> gpioPin;
}

/** \fn GPIO_Write
  * \brief Ecrire sur une broche d'un port
	* \param GPIO_TypeDef* gpioPort : Le port sur lequel on travaille
	* \param uint16_t gpioPin : La broche sur laquelle on veut écrire la valeur
	* \param uint8_t state : La valeur que l'on veut écrire
	**/
void GPIO_Write(GPIO_TypeDef* gpioPort, uint16_t gpioPin, uint8_t state)
{
	if(state != 0 && state != 1) return;
	if(gpioPin < 0 || gpioPin > 15) return;
	
	if(!state) gpioPort->ODR &= ~((unsigned int)1 << gpioPin);
	else if (state) gpioPort->ODR |= ((unsigned int)state << gpioPin);
}

/** \fn GPIO_Toggle
  * \brief Basculer la valeur sur une broche d'un port
	* \param GPIO_TypeDef* gpioPort : Le port sur lequel on travaille
	* \param uint16_t gpioPin : La broche sur laquelle on veut basculer la valeur
	**/
void GPIO_Toggle(GPIO_TypeDef* gpioPort, uint16_t gpioPin)
{
	if(gpioPin < 0 || gpioPin > 15) return;
	
	gpioPort->ODR ^= ((unsigned int)1 << gpioPin);
}

/** \fn GPIO_EnableInterrupt
  * \brief Créer une interruption sur une broche
	* \param GPIO_TypeDef* gpioPort : Le port sur lequel on travaille
	* \param uint16_t gpioPin : La broche sur laquelle on veut basculer la valeur
	* \param uint8_t priority : la priorité de l'interruption
	**/
void GPIO_EnableInterrupt(GPIO_TypeDef* gpioPort, uint16_t gpioPin, uint8_t priority)
{
	if(gpioPort == NULL) return;
	if(gpioPin < 0 || gpioPin > 4) return;
	
	RCC->APB2ENR |= (1<<0);
	
	int d = (((unsigned int)gpioPort) - APB2PERIPH_BASE)/0x400 - 2;
	
	AFIO->EXTICR[gpioPin/4] |= ( d << (gpioPin%4)*4);

	EXTI->IMR |= (1<<gpioPin);

	EXTI->RTSR |= (1<<gpioPin);

	EXTI->FTSR &= ~(1<<gpioPin);
	
	NVIC->ISER[0] |= (1 << 6+gpioPin);
	NVIC->IP[6+gpioPin] |= (priority << 4);
}

/** \fn EXTI1_IRQHandler
  * \brief Fonction d'interruption EXTI1
	**/
void EXTI1_IRQHandler(void)
{
	EXTI->PR |= (1 << 1);
	GPIO_Toggle(GPIOA, 5);
}


