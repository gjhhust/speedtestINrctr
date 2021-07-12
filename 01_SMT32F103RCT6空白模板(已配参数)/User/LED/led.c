/******************* ********************
 * 文件名  ：led.c
 * 描述    ：led 应用函数库
 *          
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 硬件连接：-----------------
 *          |   PC13 - LED1   |
 *          |       
 *          |                 |
 *           ----------------- 
 * 库版本  ：ST3.0.0  																										  
*********************************************************/
#include "led.h"


 /***************  配置LED用到的I/O口 *******************/
void LED_GPIO_Config(void)	
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( LED1_GPIO_CLK, ENABLE); // 使能PC端口时钟  
  GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);  //初始化PC端口
  GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN );	 // 关闭所有LED
}



/** 函数名：Key_GPIO_Config 
* 描述 ：配置按键用到的I/O口 
* 输入 ：
* 输出 ：无 
*/ 
void Key_GPIO_Config(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	/*开启按键端口（PE5）的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
} 


 /** 函数名：Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin)
* 描述 ：检测是否有按键按下 
* 输入 ：GPIOx：x 可以是 A，B，C，D或者 E 
GPIO_Pin：待读取的端口位 
* 输出 ：KEY_OFF(没按下按键)、KEY_ON（按下按键） 
*/
u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin) 
{
	/*检测是否有按键按下 */ 
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON )
		{ 
			/*延时消抖*/ 
			delay_ms(10000);
			if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON ) 
				{ 
					/*等待按键释放 */ 
					while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON); 
					return KEY_ON;
				} 
			else return KEY_OFF; 
	} 
	else return KEY_OFF; 
} 
