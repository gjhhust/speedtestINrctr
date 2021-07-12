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
extern USER TIME_SAVE;
extern u16 STATE;
extern Flag FlagS;
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
		FlagS.TIM2_CH1 = 1;
		TIM_ClearITPendingBit (GENERAL_TIM,TIM_IT_CC1);
	}		
	/*********************************传感器2捕获中断处理**************************************************/
	if (TIM_GetITStatus (GENERAL_TIM, TIM_IT_CC2 ) != RESET)
	{
		FlagS.TIM2_CH2 = 1;
		TIM_ClearITPendingBit (GENERAL_TIM,TIM_IT_CC2);
	}
	TIM2_CaptureCallBack();
	
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
