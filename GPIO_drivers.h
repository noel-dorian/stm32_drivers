#ifndef DEF_GPIO_DRIVERS
#define DEF_GPIO_DRIVERS

#include "stm32f10x.h"

typedef enum GPIO_MODE GPIO_MODE;
enum GPIO_MODE
{
    GPIO_MODE_FLOATING_INPUT=0x4, GPIO_MODE_OUTPUT_PUSH_PULL=0x1
};

typedef enum GPIO_CLOCK GPIO_CLOCK;
enum GPIO_CLOCK
{
    GPIO_CLOCK_ENABLE, GPIO_CLOCK_DISABLE
};

void GPIO_Init(GPIO_TypeDef* gpioPort, uint16_t gpioPin, GPIO_MODE mode);
void GPIO_Clock(GPIO_TypeDef* gpioPort, GPIO_CLOCK state);
uint8_t GPIO_Read(GPIO_TypeDef* gpioPort, uint16_t gpioPin);
void GPIO_Write(GPIO_TypeDef* gpioPort, uint16_t gpioPin, uint8_t state);
void GPIO_Toggle(GPIO_TypeDef* gpioPort, uint16_t gpioPin);

void GPIO_EnableInterrupt(GPIO_TypeDef* gpioPort, uint16_t gpioPin, uint8_t priority);

void EXTI1_IRQHandler(void);

#endif // DEF_GPIO_DRIVERS
