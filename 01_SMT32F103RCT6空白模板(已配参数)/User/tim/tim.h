#ifndef __TIM_H
#define __TIM_H

#include "main.h"

/********************通用定时器TIM参数定义，只限TIM2************/  
#define            GENERAL_TIM                   TIM2
#define            GENERAL_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM_CLK               RCC_APB1Periph_TIM2
#define            GENERAL_TIM_Period            (1000-1)
#define            GENERAL_TIM_Prescaler         71
#define            GENERAL_TIM_IRQ               TIM2_IRQn
#define            GENERAL_TIM_IRQHandler        TIM2_IRQHandler


void BASIC_TIM_Config(void);
void BASIC_TIM_NVIC_Config(void);
void BASIC_TIM_Init(void);

#endif