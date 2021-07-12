/*********************************************************************************
u8 we;
we=flag;
we=' ';
OLED_ShowNum(103,6,we,3,16);//显示ASCII字符的码值 
****************************************************************************************************/
#include "main.h"
#include "string.h"
#define car_length  55 //cm
#define senor_length  5 //cm
#define TIAOSHI
/***********************************声明变量*************************************/
Flag FlagS={0,0,0,0,0,0,0,0};

extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure1;//传感器1的结构体
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure2;//传感器2    



/******************显示定义**********************/
extern USER TIME_SAVE ;
TIM_ICUserValueTypeDef showOLED;
USER temp;
double first;
double second;
double Inval;
double speed1=0;
double speed2=0;
double a=0;
char v1[6];
char v2[6];
char a1[6];
/***********************************声明变量*************************************/

/***********************************标志位*************************************/
volatile uint16_t STATE_USE;//捕获通道标志位 0的时候是通道1捕获，1是通道2捕获
uint16_t time=0;
/***********************************END*************************************/

uint32_t get_capture_Time(void);
int flag=0;
int show_flag=0;

#ifdef TIAOSHI

int ceshi=0;

#endif

int main(void)
  {	
		uint32_t TIM_PscCLK = 72000 / (GENERAL_TIM_PSC+1);

		InitAll();
				

		//delay_ms(100);		
		
		

		
		while(1) 
		{	
			

			if(TIME_SAVE.first_finishing == 1){
				temp = TIME_SAVE;
		    
			}
			if(TIME_SAVE.second_finishing == 1 && flag == 0)
			{
				temp = TIME_SAVE;
				first=temp.first_time/TIM_PscCLK;//算出来ms
				temp.first_time/=TIM_PscCLK;
				second=temp.second_time/TIM_PscCLK;
				temp.second_time/=TIM_PscCLK;
				Inval=temp.Interval_time/TIM_PscCLK;
				TIME_SAVE.first_finishing=0;
				TIME_SAVE.second_finishing=0;
				
				speed1 = 100* senor_length / first ; //1000倍的m/s
				speed2 = 100* senor_length / second ;
				a = 1000 * (speed2 - speed1) / Inval;// 1000倍的m/s方
				
				flag =1;
			}
			
			if(flag == 1){
				sprintf(v1, "%.4f", speed1);
				sprintf(v2, "%.4f", speed2);
				sprintf(a1, "%.4f", a);
			}	
//			if(show_flag==0 && flag ==1){	OLED_Clear(0);
//				OLED_ShowString(6,0,"v1:",16);
//				//如果记录完一次保存到temp里面
//				OLED_ShowString(40,0,v1,16);
//				
//				OLED_ShowString(6,3,"v2:",16);
//				//如果记录完一次保存到temp里面
//				OLED_ShowString(40,3,v2,16);
//				
//				OLED_ShowString(6,6,"a:",16);
//				//如果记录完一次保存到temp里面
//				OLED_ShowString(40,6,a1,16);
//				
//				show_flag = 1;
//			}
			
//			zhengshu = (uint32_t)get_capture_Time()/TIM_PscCLK;
//			OLED_ShowNum(80,0,zhengshu,3,16);//显示全局时间
//			OLED_ShowString(55,2,".",16);
//			xiaoshu = (uint32_t)(get_capture_Time()%TIM_PscCLK * 1000);
//			OLED_ShowNum(80,2,xiaoshu,3,16);//显示全局时间
//			
//		 
//			OLED_ShowNum(103,6,flag,3,16);

			//delay_ms(50000);
			//delay_ms(500);
			
	//		if(w%20000 == 0){
	//			re();
	//		}
//			flag=InputCheck(1);
//			if(flag==1){
//				LED1_ON;//取反
//			}else{
//				LED1_OFF;
//			}
   	}	  
}

void TIM2_CaptureCallBack(void){
	// 第一次捕获
	if(FlagS.TIM2_CH1 == 1)
	{
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
			TIM_OC1PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Falling);
				
			// 开始捕获标准置1			
			TIM_ICUserValueStructure1.Capture_StartFlag = 1;
			TIME_SAVE.states = 2;

			
		}else if(TIME_SAVE.states == 5)
		{
			//获取捕获比较寄存器的值，这个值就是捕获到的 第一次传感器上升到第二个传感器下降 间隔高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture1 (GENERAL_TIM);//暂存
			TIME_SAVE.Interval_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + (TIM_ICUserValueStructure1.Capture_CcrValue+1);
			
			// 当第二次捕获到沿之后，就把捕获边沿配置为上升沿，一轮捕获
			TIM_OC1PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Rising);
			
			// 计数器清0
			TIM_SetCounter ( GENERAL_TIM, 0 );
			STATE_USE = 1;//使用传感器
			// 自动重装载寄存器更新标志清0
			TIM_ICUserValueStructure2.Capture_Period = 0;
			// 存捕获比较寄存器的值的变量的值清0			
			TIM_ICUserValueStructure2.Capture_CcrValue = 0;

			//用传感器2计数第二次过程
			
			TIM_ICUserValueStructure1.Capture_StartFlag = 0;
			TIM_ICUserValueStructure2.Capture_StartFlag = 1;
			TIME_SAVE.states = 6;
		}
		
		FlagS.TIM2_CH1=0;
		
	}
	
	
	
	/**********************************捕获通道二****************************************/
	// 传感器2捕获
	if(FlagS.TIM2_CH2 == 1){
		if (TIME_SAVE.states == 2)
		{

			//获取捕获比较寄存器的值，这个值就是捕获到的第一次高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture1 (GENERAL_TIM);//暂存
			TIME_SAVE.first_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure1.Capture_CcrValue+1); //printf ( "\r\n测得高电平脉宽时间：%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK ); uint32_t TIM_PscCLK = 72000000 / (GENERAL_TIM_PSC+1);
				
			//完成第一次
			TIME_SAVE.first_finishing = 1;
			TIME_SAVE.states= 3;
			
			//通道2捕获准备下降沿
			TIM_OC2PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Falling);
				
					
		}else if(TIM_ICUserValueStructure2.Capture_StartFlag == 1 && TIME_SAVE.states == 6)
		{
			//获取捕获2的比较寄存器的值，这个值就是捕获到的第二次低电平的时间的值
			TIM_ICUserValueStructure2.Capture_CcrValue = TIM_GetCapture2 (GENERAL_TIM);//暂存
			TIME_SAVE.forth_finishing = TIM_ICUserValueStructure2.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure2.Capture_CcrValue+1); 
			
			
			 //完成第二次
			TIME_SAVE.forth_finishing = 1;
			TIM_ICUserValueStructure1.Capture_StartFlag = 0;
			TIM_ICUserValueStructure2.Capture_StartFlag = 0;
			TIME_SAVE.states = 1;
			
			STATE_USE = 0 ;
			
				//通道2捕获准备上降沿
			TIM_OC2PolarityConfig(GENERAL_TIM, TIM_ICPolarity_Rising);
		}
		
			FlagS.TIM2_CH2 = 0;
	}
	
	
		/**********************************捕获通道三****************************************/
	if(FlagS.TIM2_CH3 == 1){
		if (TIME_SAVE.states == 3)
		{

			//获取捕获比较寄存器的值，这个值就是捕获到的第一次高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture3 (GENERAL_TIM);//暂存
			TIME_SAVE.second_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure1.Capture_CcrValue+1); //printf ( "\r\n测得高电平脉宽时间：%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK ); uint32_t TIM_PscCLK = 72000000 / (GENERAL_TIM_PSC+1);
			
			TIME_SAVE.second_time = TIME_SAVE.second_time - TIME_SAVE.first_time;//第二段时间
			//完成第一次
			TIME_SAVE.second_finishing = 1;
			TIME_SAVE.states= 4;
			
			//关闭通道
			TIM_ITConfig (GENERAL_TIM, TIM_IT_CC3 , DISABLE );
		}
		FlagS.TIM2_CH3 = 0;
	}
	
		/**********************************捕获通道三****************************************/
	if(FlagS.TIM2_CH4 == 1){
		if (TIME_SAVE.states == 4)
		{

			//获取捕获比较寄存器的值，这个值就是捕获到的第一次高电平的时间的值
			TIM_ICUserValueStructure1.Capture_CcrValue = TIM_GetCapture4 (GENERAL_TIM);//暂存
			TIME_SAVE.third_time = TIM_ICUserValueStructure1.Capture_Period * (GENERAL_TIM_PERIOD+1) + 
			       (TIM_ICUserValueStructure1.Capture_CcrValue+1); //printf ( "\r\n测得高电平脉宽时间：%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK ); uint32_t TIM_PscCLK = 72000000 / (GENERAL_TIM_PSC+1);
			
			TIME_SAVE.third_time = TIME_SAVE.third_time - TIME_SAVE.second_time - TIME_SAVE.first_time;//第二段时间
			//完成第一次
			TIME_SAVE.third_finishing = 1;
			TIME_SAVE.states= 5;
			
			//关闭通道
			TIM_ITConfig (GENERAL_TIM, TIM_IT_CC4 , DISABLE );
					
		}
		FlagS.TIM2_CH4 = 0;
	}
	
	
	
	
}
