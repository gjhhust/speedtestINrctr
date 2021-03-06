
/****************************************************************************************************
//=========================================电源接线================================================//
//      5V  接DC 5V电源
//     GND  接地
//======================================OLED屏数据线接线==========================================//
//本模块数据总线类型为IIC
//     SCL  接PB6    // IIC时钟信号
//     SDA  接PB7    // IIC数据信号
//======================================传感器接线==========================================//
      1模拟DO口  接PB10
			2模拟DO口  接PB11
//=========================================触摸屏接线=========================================//
//本模块本身不带触摸，不需要接触摸屏线
//============================================================================================//
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/	

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

/***********************************声明变量*************************************/
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure1;//传感器1的结构体
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure2;//传感器2    
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
u16 STATE=0;//捕获通道标志位 0的时候是通道1捕获，1是通道2捕获
/***********************************END*************************************/

uint32_t get_capture_Time(void);
int flag=0;
int show_flag=0;


int main(void)
  {	
		uint32_t TIM_PscCLK = 72000 / (GENERAL_TIM_PSC+1);

		InitAll();
		OLED_Clear(0); 
		
		delay_ms(10000);
		
		

		
		while(1) 
		{	
			
			
			
			if(show_flag == 0){
				OLED_Clear(0);
				OLED_ShowString(6,3,"loading.....",16);
			//如果记录完一次保存到temp里面
			}	
			
			

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
				Inval=temp.Interval_time/=TIM_PscCLK;
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
			if(show_flag==0 && flag ==1){	OLED_Clear(0);
				OLED_ShowString(6,0,"v1:",16);
				//如果记录完一次保存到temp里面
				OLED_ShowString(40,0,v1,16);
				
				OLED_ShowString(6,3,"v2:",16);
				//如果记录完一次保存到temp里面
				OLED_ShowString(40,3,v2,16);
				
				OLED_ShowString(6,6,"a:",16);
				//如果记录完一次保存到temp里面
				OLED_ShowString(40,6,a1,16);
				
				show_flag = 1;
			}
			
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



uint32_t get_capture_Time(void){
		// TIM 计数器的驱动时钟
		return showOLED.Capture_Period * (GENERAL_TIM_PERIOD+1) + (showOLED.Capture_CcrValue+1);
	
}

///////////////////////////////////////////////////////////////////////