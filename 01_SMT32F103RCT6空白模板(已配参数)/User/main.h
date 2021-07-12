#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "delay.h"
#include "sys.h"
#include <stdio.h>
#include "speedtext.h"
#include "led.h"
#include "bsp_GeneralTim.h" 

void TIM2_CaptureCallBack();
typedef struct{
	uint8_t TIM2_CH1;
	uint8_t TIM2_CH2;
	uint8_t TIM2_CH3;
	uint8_t TIM2_CH4;
	
	uint8_t TIM3_CH1;
	uint8_t TIM3_CH2;
	uint8_t TIM3_CH3;
	uint8_t TIM3_CH4;
}Flag;


#endif