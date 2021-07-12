#ifndef __MANAGE_F103_H
#define __MANAGE_F103_H	
/*==================================================================================================================
 *【文件名称】  manage_f103.h
 *【编写人员】  魔女开发板团队-老周
 *【更新分享】  如有更新，分享在Q群文件夹  262901124
 *【淘    宝】  魔女开发板  https://demoboard.taobao.com
 *【最后修改】  2020-09-12
 ===================================================================================================================
 *【 功  能 】  简化常用的系统函数、初始化函数
 *
 *              常用数据定义、缩写：
 *              ----数据类型缩写
 *              ----位带     
 * 
 *              常用函数声明：                
 *              ----初始化 USART1 作为调试信息输出 
 *              ----初始化 SysTick时钟， 使delay_ms、delay_us正常工作 
 *              ----定义 GPIOSet()、NVICSet()、EXTISet(), 减少大量代码工作 
 *                 
 *【移植使用】  本文件参考正点原子、野火等大神的代码后，综合工作实况改编。
 *              移植使用，会与正点原子原sys文件夹下3个文件sys.c、delay.c、usart.c有引用上的冲突；
 *              本文件已包含、完善上述三个文件的功能，使用时，可先尝试取消上述三个文件的引用，即可解决冲突问题。
 ========================================================== =======================================================*/   
#include <stm32f10x.h>    // 优先使用用户目录中文件，方便修改优化。这个文件是必须的， 各种地址和参数的宏定义
#include "stdio.h"        // C标准库头文件: 输入输出函数；getchar()、putchar()、scanf()、printf()、gets()、puts()、sprintf()
#include "stdlib.h"       // C标准库头文件: 通用工具函数：malloc()、calloc()、realloc()、free()、system()、atoi()、atol()、rand()、srand()、exit()
#include "stdbool.h"      // C标准库头文件: 定义布尔类型: bool、true、false
#include "string.h"       // C标准库头文件: 字符数组常用：strcpy()、strncpy()、strcmp()、strlen()、strnset()





// 移植配置区 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
// 说明1：如果外部已有USART1、SysTick的初始化，尽量使用外部的， 用#if关掉本文件中的。
// 说明2：GPIOSet()、NVICSet()、EXTISet()三大初始化函数，正常使用，不受说明1影响，

// USART
#define  USARTx                     USART1
#define  USART_CLOCK_EN             RCC->APB2ENR |= RCC_APB2ENR_USART1EN 
#define  USART_CLK 
// TX
#define  USART_TX_GPIO              GPIOA
#define  USART_TX_PIN               GPIO_Pin_9
// RX
#define  USART_RX_GPIO              GPIOA  
#define  USART_RX_PIN               GPIO_Pin_10
// 中断
#define  USART_IRQHANDLER           USART1_IRQHandler       // 中断服务函数名称
#define  USART_IRQN                 USART1_IRQn             // 中断向量
// DMA
#define  DMAx_CLOCK_EN
#define  USART_TX_DMAx_CHANNELx     DMA1_Channel4           // USART使用DMA通道发送数据
#define  USART_RX_DMAx_CHANNELx     DMA1_Channel5           // USART使用DMA通道接收数据
// 接收缓冲区大小
#define  USART_RX_MAX               512                     // USART1通过DMA+IDLE接收不定长数据时的数组大小
//end 移植  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 












// 全局状态标志结构体,  0=失败/没有, 1=成功/正常
typedef struct 
{
    uint8_t  allOK;                // 全部状态正常        0=失败   1=成功  
    // USART1 已连接到虚拟串口
    uint8_t  PrintfOK;             // 标记USART是否配置   0=失败   1=成功   防止在配置前调用printf卡死
    uint8_t  usartRecived;         // 调试所用的串口接收到数据后，等待处理标志， u8 RxBuffer[255]
    
    uint8_t  RTCOK;  
    // LCD
    uint8_t  LcdOK;                // lcd状态 
    uint8_t  LcdType;              // 型号
    uint8_t  LcdInfo[60];          // 状态信息  
    
}_flag; 
extern _flag xFlag;

////////////////////////////////////////////////////////////////////////////////// 
// 【 1 】 数据类型短关键字, 在stdint.h的基础上进行 
// 有符号                                // stdint.h 上的类型定义
typedef              int8_t    s8;      // typedef   signed          char int8_t; 
typedef             int16_t   s16;      // typedef   signed short     int int16_t;
typedef             int32_t   s32;      // typedef   signed           int int32_t;
typedef             int64_t   s64;      // typedef   signed       __INT64 int64_t;
// 无符号
typedef             uint8_t    u8;      // typedef unsigned          char uint8_t;
typedef            uint16_t   u16;      // typedef unsigned short     int uint16_t;
typedef            uint32_t   u32;      // typedef unsigned           int uint32_t;
typedef            uint64_t   u64;      // typedef unsigned       __INT64 uint64_t;
// 
typedef  volatile   uint8_t   vu8;      // volatile 每一次读取都要从内存读取，防止被错误地优化了
typedef  volatile  uint16_t  vu16;
typedef  volatile  uint32_t  vu32;  
//
#ifndef  bool
#define  bool      _Bool
#endif
//
#ifndef  true 
#define  true      1
#endif 
//
#ifndef  false
#define  false     0  
#endif
 
 
#ifndef USE_STDPERIPH_DRIVER
// GPIO引脚编号定义
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /*!< All pins selected */

// 
#define GPIO_Speed_10MHz	    1		// GPIO速度2Mhz
#define GPIO_Speed_2MHz		    2		// GPIO速度2Mhz
#define GPIO_Speed_50MHz	    3		// GPIO速度50Mhz
#endif

   
   
// 为兼容F4xx的书写风格 *************************************************************************
// 时钟使能
#define  RCC_APB2ENR_GPIOAEN   ((uint32_t)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2ENR_GPIOBEN   ((uint32_t)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2ENR_GPIOCEN   ((uint32_t)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2ENR_GPIODEN   ((uint32_t)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2ENR_GPIOEEN   ((uint32_t)0x00000040)         /*!< I/O port D clock enable */
// GPIOSet()专用参数
#define GPIO_Mode_AIN	        0	    // 模拟输入模式
#define GPIO_Mode_IN            1		// 普通输入模式
#define GPIO_Mode_OUT		    2		// 普通输出模式
#define GPIO_Mode_AF		    3		// AF功能模式
//
#define GPIO_OType_PP		    0		// 推挽输出
#define GPIO_OType_OD		    1		// 开漏输出 
//
#define GPIO_Speed_10M		    1		// GPIO速度2Mhz
#define GPIO_Speed_2M		    2		// GPIO速度2Mhz
#define GPIO_Speed_50M		    3		// GPIO速度50Mhz
//
#define GPIO_PuPd_NOPULL        0		// 不带上下拉
#define GPIO_PuPd_UP		    1		// 上拉
#define GPIO_PuPd_DOWN		    2		// 下拉
// EXTISet()专用参数
#define EXTI_FTIR   		    1  		// 下降沿触发
#define EXTI_RTIR   		    2  		// 上升沿触发
// System_MCOxInit()专用参数
// MCO1引脚输出时钟频率
#define RCC_MCO1Source_SYSCLK   4       // 0100
#define RCC_MCO1Source_HSI      5       // 0101
#define RCC_MCO1Source_HSE      6       // 0110
#define RCC_MCO1Source_NULL     0       // 0000，关闭

// 中量向量偏移值， This value must be a multiple of 0x200
#define VECT_TAB_OFFSET  0x0      

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入




/*****************************************************************************
 ** 声明  全局函数
 ** 数量  
 ** 更新  资料分享Q群  262901124
****************************************************************************/
// SysTick 
void  System_SysTickInit (void);                         // 配置SysTick时钟，配置后delay_ms、delay_us即可使用
void  delay_ms (u32);                                    // 毫秒延时  
void  delay_us (u32);                                    // 微秒延时
u64   System_GetTimeMs (void);                           // 获取 SysTick 计时数, 单位:ms
u32   System_GetTimeInterval (void);                     // 监察运行时间
void  System_SetTimesPoint (void);                       // printf打印监察运行时间 
// 简化初始化代码的3大函数
void  GPIOSet (GPIO_TypeDef* GPIOx,  u32 PINx,  u8 MODE,  u8 OTYPE,  u8 OSPEED,  u8 PUPD);  // GPIO初始化
void  NVICSet (u8 NVIC_Channel, u8 Preemption);          // NVIC优先级配置, 已分好组, 4位抢占级, 16级, 无子级
void  EXTISet (GPIO_TypeDef* GPIOx, u16 PINx, u8 TRIM);  // 外部中断配置函数
// 调试所用串口 
void  System_Usart1Init (u32 baudrate);                  // USART1初始化，通过PA9、PA10引脚进行通信，注意：已连接至CMSIS-DAP
void  printData (u8 *buf, u8 cnt);                       // USART1利用发送中断，发送指定长度字节，适合少量数据的频密发送，建议优先使用
void  printString (char* charTemp);                      // USART1使用DMA发送不定长字符串，适合UASRT很空闲时的大量数据的发送
// 辅助功能
void  System_SwdMode (void);                             // SWD调试模式，关闭JTAG只保留SWD，可释放引脚PB3、PB4、PA15，只需PA13、PA14
u32   System_GetSystemClock (void);                      // 获取系统时钟频率
void  System_Reset (void);                               // 系统软复位  
void  MCO1Init (uint32_t);                               // 可选参数,前辍 RCC_MCO1Source_ +++ HSI , LSE , HSE , PLLCLK
void  MCO2Init (uint32_t);                               // 可选参数,前辍 RCC_MCO2Source_ +++ SYSCLK , PLLI2SCLK , HSE , PLLCLK

#endif



