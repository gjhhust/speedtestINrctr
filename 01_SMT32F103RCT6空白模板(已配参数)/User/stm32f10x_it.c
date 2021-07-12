/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "main.h" 
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure1;
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure2;
extern TIM_ICUserValueTypeDef showOLED;
extern USER TIME_SAVE;
extern USER temp;
extern u16 STATE;

void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 

void GENERAL_TIM_INT_FUN(void)
{
	// 当要被捕获的信号的周期大于定时器的最长定时时，定时器就会溢出，产生更新中断
	// 这个时候我们需要把这个最长的定时周期加到捕获信号的时间里面去
	if ( (TIM_GetITStatus ( GENERAL_TIM, TIM_IT_Update) != RESET) && STATE == 0) //传感器1溢出              
	{	
		TIM_ICUserValueStructure1.Capture_Period ++;
		TIM_ClearITPendingBit ( GENERAL_TIM, TIM_FLAG_Update ); 		
	}else if( (TIM_GetITStatus ( GENERAL_TIM, TIM_IT_Update) != RESET) && STATE == 1)//传感器2溢出
	{
		TIM_ICUserValueStructure2.Capture_Period ++;
		TIM_ClearITPendingBit ( GENERAL_TIM, TIM_FLAG_Update ); 
	}

	
	
	/*********************************传感器1捕获中断处理**************************************************/
	if ( TIM_GetITStatus (GENERAL_TIM, TIM_IT_CC1 ) != RESET  )//传感器1的捕获标志位
	{
		
		// 第一次捕获
		if ( TIM_ICUserValueStructure1.Capture_StartFlag == 0 && TIME_SAVE.states == 1)
		{
			// 计数器清0
			TIM_SetCounter ( GENERAL_TIM, 0 );
			// 自动重装载寄存器更新标志清0
			TIM_ICUserValueStructure1.Capture_Period = 0;
			// 捕获比较寄存器的值的变量的值清0			
			TIM_ICUserValueStructure1.Capture_CcrValue = 0;
						// 自动重装载寄存器更新标志清0
			TIM_ICUserValueStructure2.Capture_Period = 0;
			// 捕获比较寄存器的值的变量的值清0			
			TIM_ICUserValueStructure2.Capture_CcrValue = 0;
			
			
			//第一次捕获到上升沿代表车头第一次遇到传感器1，则下一次需要捕获传感器2的上升沿
			TIME_SAVE.first_start = 1;//开启第一次捕获过程
			TIM_OC1PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Falling);
				
			// 开始捕获标准置1			
			TIM_ICUserValueStructure1.Capture_StartFlag = 1;
			TIME_SAVE.states = 2;

			
		}else if(TIME_SAVE.first_finishing == 1 && TIME_SAVE.states == 3)
		{
			//获取捕获比较寄存器的值，这个值就是捕获到的 第一次传感器上升到第二个传感器下降 间隔高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture1 (GENERAL_TIM);//暂存
			TIME_SAVE.Interval_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + (TIM_ICUserValueStructure1.Capture_CcrValue+1);
			
			// 当第二次捕获到沿之后，就把捕获边沿配置为上升沿，一轮捕获
			TIM_OC1PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Rising);
			
			// 计数器清0
			TIM_SetCounter ( GENERAL_TIM, 0 );
			STATE = 1;//使用传感器
			// 自动重装载寄存器更新标志清0
			TIM_ICUserValueStructure2.Capture_Period = 0;
			// 存捕获比较寄存器的值的变量的值清0			
			TIM_ICUserValueStructure2.Capture_CcrValue = 0;
			//开启第二次捕获过程
			TIME_SAVE.second_start = 1;
			//用传感器2计数第二次过程
			
			TIM_ICUserValueStructure2.Capture_StartFlag = 1;
			TIME_SAVE.states = 4;
		}
		
		TIM_ClearITPendingBit (GENERAL_TIM,TIM_IT_CC1);
	}		
	/*********************************传感器2捕获中断处理**************************************************/
	if (TIM_GetITStatus (GENERAL_TIM, TIM_IT_CC2 ) != RESET)
	{
		
		// 传感器2捕获
		if ( TIM_ICUserValueStructure2.Capture_StartFlag == 0 && TIME_SAVE.states == 2)
		{

			//获取捕获比较寄存器的值，这个值就是捕获到的第一次高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture1 (GENERAL_TIM);//暂存
			TIME_SAVE.first_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure1.Capture_CcrValue+1); //printf ( "\r\n测得高电平脉宽时间：%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK ); uint32_t TIM_PscCLK = 72000000 / (GENERAL_TIM_PSC+1);
				
			//完成第一次
			TIME_SAVE.first_finishing = 1;
			TIME_SAVE.first_start = 0  ;
			TIME_SAVE.states= 3;
			
			//通道2捕获准备下降沿
			TIM_OC2PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Falling);
				
					
		}else if(TIM_ICUserValueStructure2.Capture_StartFlag == 1 && TIME_SAVE.states == 4)
		{
			//获取捕获2的比较寄存器的值，这个值就是捕获到的第二次低电平的时间的值
			TIM_ICUserValueStructure2.Capture_CcrValue = TIM_GetCapture2 (GENERAL_TIM);//暂存
			TIME_SAVE.second_time = TIM_ICUserValueStructure2.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure2.Capture_CcrValue+1); 
			
			
			 //完成第二次
			TIME_SAVE.second_finishing = 1;
			TIME_SAVE.second_start = 0  ;
			TIM_ICUserValueStructure1.Capture_StartFlag = 0;
			TIM_ICUserValueStructure2.Capture_StartFlag = 0;
			TIME_SAVE.states = 1;
			STATE = 0 ;
			
				//通道2捕获准备上降沿
			TIM_OC2PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Rising);
		}
		
		TIM_ClearITPendingBit (GENERAL_TIM,TIM_IT_CC2);
	}
	
	
			
	
}

/***********else if (TIM_GetITStatus (GENERAL_TIM, TIM_IT_CC2 ) != RESET )// 捕获传感器2的上升沿
		{
			// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
			TIME_SAVE.first_time = TIM_GetCapture1 (GENERAL_TIM);
			TIM_ICUserValueStructure1.Capture_CcrValue = 
			GENERAL_TIM_GetCapturex_FUN (GENERAL_TIM);

			// 当第二次捕获到上升沿之后，就把捕获边沿配置为下降沿，开启车尾一轮捕获
			GENERAL_TIM_OCxPolarityConfig_FUN(GENERAL_TIM, TIM_ICPolarity_Falling);
			//完成第一次捕获过程
			TIME_SAVE.first_finishing = 1;
			STATE = 1;
			
      // 传感器2开始捕获标志1	
			TIM_ICUserValueStructure2.Capture_StartFlag = 1;
      
			//调试
			showOLED = TIM_ICUserValueStructure1;
		}else if(TIM_GetITStatus (GENERAL_TIM, TIM_IT_CC1 ) != RESET && STATE == 1)
		{
		
		
		}

		TIM_ClearITPendingBit (GENERAL_TIM,GENERAL_TIM_IT_CCx);	  
		*////
		
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
