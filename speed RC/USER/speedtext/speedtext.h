#ifndef __SPEEDTEXT_H
#define __SPEEDTEXT_H

#include "main.h"

//定义传感器的接口
#define GPIO_Senor1    GPIO_Pin_0
#define GPIO_Senor2    GPIO_Pin_1


#define GPIO_des    GPIOA

//时钟
#define RCC_APB2Periph_NEED     RCC_APB2Periph_GPIOA


uint8_t InputCheck(uint8_t senor_num);//输入检测
void Senor_GPIOInit(void);
void InitAll(void);//全部硬件初始化

typedef struct
{   
	uint32_t  first_time;     // 第一次时间
	uint8_t		first_start; //完成标志位
	uint8_t		first_finishing; //完成标志位
	
	uint32_t  second_time;    // 第二次时间
	uint8_t		second_start; //完成标志位
	uint8_t		second_finishing; //完成标志位
	
	uint32_t  Interval_time;    // 间隔时间
	
	uint32_t states;//1.2.3.4
}USER;


#endif