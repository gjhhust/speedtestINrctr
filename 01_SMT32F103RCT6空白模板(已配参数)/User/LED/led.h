#ifndef __LED_H
#define	__LED_H

#include "main.h"


/*****LED********/
#define LED1_GPIO_PORT    	GPIOA		              /* GPIO端口 */
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */
#define LED1_GPIO_PIN			  GPIO_Pin_8			        
/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
 /* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //输出为高电平		
#define digitalLo(p,i)		 {p->BRR=i;}	 //输出低电平
#define digitalToggle(p,i) {p->ODR ^=i;} //输出反转状态

 /* 定义控制IO的宏 */
#define LED1_TOGGLE		 digitalToggle(LED1_GPIO_PORT,LED1_GPIO_PIN)//取反
#define LED1_OFF		   digitalHi(LED1_GPIO_PORT,LED1_GPIO_PIN)//关闭
#define LED1_ON			   digitalLo(LED1_GPIO_PORT,LED1_GPIO_PIN)//打开

void LED_GPIO_Config(void);
///////////////////////////END//////////////////////////////////////////////





#define KEY_ON  1
#define KEY_OFF 0



					
void Key_GPIO_Config(void);
u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin);
#endif /* __LED_H */
