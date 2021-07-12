#include "speedtext.h"
extern int time;
USER TIME_SAVE = {0,0,0,0,0,0,0,1};//保存两次捕获时间

/**********************************************************************************************************
*函 数 名: 相关初始化函数
*功能说明: 初始化
*形    参: rx_message0
*返 回 值: 无
**********************************************************************************************************/
void InitAll(void){
	//中断标志位需要用的时候再开启
	
	SystemInit(); //配置系统时钟为72M //待定

	
	GENERAL_TIM_Init();//初始化定时器并开始计时；
//	
//OLED_Init();			//初始化OLED 
	
	LED_GPIO_Config();
		
	Senor_GPIOInit();//引脚初始化
	

}	
/**********************************************************************************************************
*函 数 名: 传感器接口初始化函数
*功能说明: 初始化
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void Senor_GPIOInit(void){
			 /*定义GPIO初始化结构体*/
	 GPIO_InitTypeDef GPIO_InitStructure;
		
		/*开启GPIOA的时钟的代码*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_NEED, ENABLE);

	 //选择按键的引脚
	 GPIO_InitStructure.GPIO_Pin = GPIO_Senor1| GPIO_Senor2 ;//| GPIO_Senor2 | GPIO_Senor3 | GPIO_Senor4;
	 // 设置按键的引脚为浮空输入
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	
	
 //使用结构体初始化
		GPIO_Init(GPIO_des, &GPIO_InitStructure);
		GPIO_SetBits(GPIO_des,GPIO_Senor1| GPIO_Senor2);
}

/**********************************************************************************************************
*函 数 名: 输入检测函数
*功能说明: 检测传感器的输入
*形    参: rx_message0
*返 回 值: 0为有光，1为暗
**********************************************************************************************************/
uint8_t InputCheck(uint8_t senor_num){
		
}