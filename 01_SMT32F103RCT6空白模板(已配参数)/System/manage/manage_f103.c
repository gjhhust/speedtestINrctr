/*==================================================================================================================
 *【文件名称】  manage_f103.c
 *【编写人员】  魔女开发板团队-老周
 *【更新分享】  如有更新，分享在Q群文件夹  262901124
 *【淘    宝】  魔女开发板  https://demoboard.taobao.com
 *【最后修改】  2020-09-12
 ===================================================================================================================
 *【 功  能 】  简化常用的系统函数、初始化函数
 *
 *              1- USART1  部分
 *              2- SysTick 部分
 *              3- 常用初始化函数 部分
 *              4- 辅助功能函数   部分
 *                 
 *【移植使用】  本文件参考正点原子、野火等大神的代码后，综合工作实况改编。
 *              移植使用，会与正点原子原sys文件夹下3个文件sys.c、delay.c、usart.c有引用上的冲突；
 *              本文件已包含、完善上述三个文件的功能，使用时，可先尝试取消上述三个文件的引用，即可解决冲突问题。
 ========================================================== =======================================================*/   
#include "manage_f103.h"





/*****************************************************************************
 ** 本地变量声明
 *****************************************************************************/
u64 sysTickCnt = 0;                    // 运行时长，单位：ms
u8  RxTemp [USART_RX_MAX];             // 串口USART空闲中断+DMA接收时的接收缓冲区
u8  RxBuffer [USART_RX_MAX];           // 串口USART接收到数据后，临时缓冲区RxTemp转存到接收存放区RxBuffer, 等待被处理 
_flag xFlag;                           // 全局状态标志













// ###########################     第1部分 -   USART1 初始化，用作串口调试输出     #############################################



#if 1     // 如果使用外部代码初始化USART1 , 就把1改为0， 失效本文件整片USART功能
/******************************************************************************
 * 函  数： System_Usart1Init
 * 功  能： 初始化USART1, 用于调试信息输出，重构put已可使用printf
 * 参  数： u32 nus : 延时的微秒数 
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
 ******************************************************************************/  
void  System_Usart1Init(u32 baudrate)
{
    // 重要： 外设时钟要在GPIO端口配置前使能
    USART_CLOCK_EN;   
    
    // gpio
    GPIOSet (USART_TX_GPIO, USART_TX_PIN ,GPIO_Mode_AF ,GPIO_OType_PP ,GPIO_Speed_50M ,GPIO_PuPd_UP );
    GPIOSet (USART_RX_GPIO ,USART_RX_PIN, GPIO_Mode_IN ,GPIO_OType_PP ,GPIO_Speed_50M ,GPIO_PuPd_NOPULL );
    
    // USART初始化     115200-8-N-1
    float temp   = (float)(System_GetSystemClock())/(baudrate*16); // 得到USARTDIV,OVER8设置为0
	u16 mantissa = temp;				                     // 得到整数部分
	u16 fraction = (temp-mantissa)*16;                       // 得到小数部分,OVER8设置为0, *16=左移4位	 
    mantissa   <<= 4;
	mantissa    += fraction;        
    USARTx ->BRR  = mantissa;                         // 设置波特率因子 
                                                      
	USARTx ->CR1  = 0;                                // 清0
    USARTx ->CR1 |= 1<<2;                             // 接收
    USARTx ->CR1 |= 1<<3;                             // 发送
    USARTx ->CR1 |= 1<<4;                             // 空闲中断
    USARTx ->CR1 |= 0<<10;                            // 校验
    USARTx ->CR1 |= 0<<12;                            // 8位字长       
    USARTx ->CR1 |= 0<<15;                            // 过采样 0=16     
    
    // DMA接收初始化
    USARTx ->CR3  = 1<<6;                             // 使能DMA接收  
    RCC->AHBENR  |= 1<<0;			                  // 开启DMA1时钟 [1]:DMA2   [0]:DMA1
    USART_RX_DMAx_CHANNELx ->CCR   = 0;	              // 复位
	USART_RX_DMAx_CHANNELx ->CPAR  = (u32)&USARTx->DR;// 外设地址 
	USART_RX_DMAx_CHANNELx ->CMAR  = (u32)RxTemp; 	  // 存储器地址
	USART_RX_DMAx_CHANNELx ->CNDTR = USART_RX_MAX ;   // 传输数据量

	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<4;  		  // 数据传输方向   0:从外设读   1:从存储器读
	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<5;  		  // 循环模式       0:不循环     1：循环
	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<6; 		      // 外设地址非增量模式
	USART_RX_DMAx_CHANNELx ->CCR  |= 1<<7; 	 	      // 存储器增量模式
	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<8; 	 	      // 外设数据宽度为8位
	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<10; 		  // 存储器数据宽度8位
	USART_RX_DMAx_CHANNELx ->CCR  |= 1<<12; 		  // 中等优先级
	USART_RX_DMAx_CHANNELx ->CCR  |= 0<<14; 		  // 非存储器到存储器模式	
    USART_RX_DMAx_CHANNELx ->CCR  |= 1<<0;            // 正式驱动DMA传输

    NVICSet(USART_IRQN, 6);                           // 使能中断通道，设置优先级  
    USARTx ->CR1 |= 1<<13;                            // 使能USART 
    
    xFlag.PrintfOK =1;                                // 标记USART已配置，防止在配置前调用printf卡死
    
    printf("\r\r\r========= 魔女开发板 STM32F103RCT6 外设初始报告 =========\r");      
    printf("USART1配置            空闲中断+DMA接收, 中断发送\r");       
}

/******************************************************************************
 * 函  数： USART_IRQHANDLER
 * 功  能： 空闲中断 + DMA 接收数据
 * 参  数： 
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月10日
 ******************************************************************************/
static u8 TxBuffer[512] ;
static u8 TxCounter = 0 ;
static u8 count = 0 ;

void USART_IRQHANDLER(void)           
{     
    // 用于接收数据	
    // DMA接收+空闲中断
    if(USARTx->SR & 1<<4)                             // 检查IDLE中断标志
    {   
        USARTx ->SR;                                  // !!! 这两句很重要, 作用是清理IDLE中断, 序列清零，顺序不参错!!
        USARTx ->DR;                                  // !!!            
        USART_RX_DMAx_CHANNELx ->CCR &= ~(1<<0);      // 失能 DMA, 关闭方能修改参数
        
        u8 num = USART_RX_MAX - USART_RX_DMAx_CHANNELx ->CNDTR;  // 计算接收到多少个字节
        
        printf("\rUSARTx 接收到信息: ");              // 把接收到的数据打印出来, 看正确不
        for(u8 i=0; i<num; i++)
            printf("%c", RxTemp[i]);
        printf("\r");
        
        
        memcpy(RxBuffer, RxTemp , USART_RX_MAX);      // 临时数据转存为待处理数据, 等待处理
        memset(RxTemp ,0, USART_RX_MAX);              // 临时数据空零,准备下一次的接收
		
        // 注意： 外部程序通过检查此标志位得知是否有新数据
        xFlag.usartRecived =1;                        // 调试所用的串口接收到数据后，等待处理标志， u8 RxBuffer[]

        USART_RX_DMAx_CHANNELx ->CNDTR = USART_RX_MAX;// 重新设置DMA的传输数目, 必须大于要传输的数目,否则超出部分会履盖开始部份
        USART_RX_DMAx_CHANNELx ->CCR  |= 1<<0;        // 使能 DMA                
     }     

    // 发送中断
    if (USARTx->SR & 1<<7)                            // 检查中断来源 TXEIE
    {
        USARTx->DR = TxBuffer[TxCounter++];
        
        if(TxCounter == count )
            USARTx->CR1 &= ~(1<<7);                   // 已发送完成，关闭发送缓冲区空置中断 TXEIE
    }  
}  

/******************************************************************************
 * 函  数： print
 * 功  能： UART通过中断发送数据，需要函数外三个变量、中断配合
 *         【特别注意】本函数可发送各种数据，而不限于字符串
 *         【特别注意】注意缓冲区为512字节，如果发送频率太高，注意波特率
 * 参  数： u8* buf   需发送数据的首地址
 *         u8  cnt   发送的字节数  
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年07月10日
 ******************************************************************************/  
void printData(u8* buf, u8 cnt)
{
    for(u8 i=0; i<cnt; i++) 
        TxBuffer[count++] = buf[i];
     
    if((USARTx->CR1 & 1<<7) == 0 )         // 检查发送缓冲区空置中断(TXEIE)是否已打开
        USARTx->CR1 |= 1<<7;             
}

/******************************************************************************
 * 函  数： printString
 * 功  能： UART通过DMA发送数据
 *         【特别注意】通过DMA发送，更适合字节数比较大的字符串，而不用占用中断
 *         【特别注意】本函数以0判断字符串的结尾，只适合发送字符串，不适合发送可能含0的数值类数据
 * 参  数： u8* charTemp  要发送的字符串首地址
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月10日
 ******************************************************************************/  
void printString(char* charTemp) 
{
    u32 num = 0;                                // 发送的数量，注意发送的单位不是必须8位的    
    char* t=charTemp ;                          // 用于配合计算发送的数量    
    while(*t++ !=0)  num++;                     // 计算要发送的数目，这步比较耗时，测试发现每多6个字节，增加1us，单位：8位   
        
#if 1                                           // 使用发送中断，好处：发送频率较高，但字节数较少，使用发送中断更合理
    printData((u8*)charTemp , num); 
#else                                           // 使用DMA发送数据, 好处:在发送频率不高，但单次发送字节数量很多时，如几十上百的，DMA好用
    static u8 Flag_DmaTxInit=0;                 // 用于标记是否已配置DMA发送
    while(DMA1_Channel4->CNDTR > 0);            // 重要：如果DMA还在进行上次发送，就等待; 得进完成中断清标志，F4不用这么麻烦，发送完后EN自动清零
    if( Flag_DmaTxInit == 0)                    // 是否已进行过USAART_TX的DMA传输配置
    {   
        Flag_DmaTxInit  = 1;                    // 设置标记，下次调用本函数就不再进行配置了
        USARTx ->CR3   |= 1<<7;                 // 使能DMA发送
        RCC->AHBENR    |= 1<<0;			        // 开启DMA1时钟  [0]DMA1   [1]DMA2        
 
        DMA1_Channel4->CCR   = 0;               // 失能， 清0整个寄存器, DMA必须失能才能配置
        DMA1_Channel4->CNDTR = num;    	        // 传输数据量   
        DMA1_Channel4->CMAR  = (u32)charTemp;   // 存储器地址 
        DMA1_Channel4->CPAR  = (u32)&USARTx->DR;// 外设地址      

        DMA1_Channel4->CCR |= 1<<4;  		    // 数据传输方向   0:从外设读   1:从存储器读
        DMA1_Channel4->CCR |= 0<<5;  		    // 循环模式       0:不循环     1：循环
        DMA1_Channel4->CCR |= 0<<6; 		    // 外设地址非增量模式
        DMA1_Channel4->CCR |= 1<<7; 	 	    // 存储器增量模式
        DMA1_Channel4->CCR |= 0<<8; 	 	    // 外设数据宽度为8位
        DMA1_Channel4->CCR |= 0<<10; 		    // 存储器数据宽度8位
        DMA1_Channel4->CCR |= 0<<12; 		    // 中等优先级
        DMA1_Channel4->CCR |= 0<<14; 		    // 非存储器到存储器模式	
    }    
    DMA1_Channel4->CCR  &= ~((u32)(1<<0));      // 失能，DMA必须失能才能配置
    DMA1_Channel4->CNDTR = num;    	            // 传输数据量
    DMA1_Channel4->CMAR  = (u32)charTemp;       // 存储器地址      
	DMA1_Channel4->CCR  |= 1<<0;                // 开启DMA传输   
#endif    
} 


/******************************************************************************
 * 功  能： printf函数支持代码
 *         【特别注意】加入以下代码, 使用printf函数时, 不再需要选择use MicroLIB	 
 * 参  数： 
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年07月15日
 ******************************************************************************/  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	 
#pragma import(__use_no_semihosting)                
struct __FILE         { int handle; }; 
FILE __stdout; 
void _sys_exit(int x) {	x = x; } 


#endif








// #######################     第2部分 - SysTick初始化，用作系统延时时基，1000Hz   ###########################################



/*****************************************************************************
 * 函  数： SysTick_Init
 * 功  能： 配置systick定时器， 1ms中断一次， 用于任务调度器、delay_ms、delay_us
 * 参  数：
 * 返回值： 
 * 重  要： SysTick 的时钟源自 HCLK 的 8 分频
 * 备  注： 魔女开发板团队  资料存放Q群：262901124  2020年04月21日
*****************************************************************************/
void System_SysTickInit(void)
{    
    // 改了几次位置， 在这里调用最合适了
    RCC->APB2ENR |= 1<< 0;                 // 打开辅助功能AFIO时钟
    AFIO->MAPR   |= 0x02 << 24;            // 为了把PB3\PB4作为普通引脚，关闭JATG功能，但保留SWD； 如果同时关闭，就无法再写芯片了    
    
    SystemCoreClock = 5201314;             // 用于存放系统时钟频率，先随便设个值
    SystemCoreClockUpdate();               // 获取当前时钟频率， 更新全局变量 SystemCoreClock值 
    printf("系统运行时钟          %d Hz\r", SystemCoreClock);  // 系统时钟频率信息 , SystemCoreClock在system_stm32f4xx.c中定义   
    
    u32 msTick= SystemCoreClock /8 /1000;  // 计算重载值，全局变量SystemCoreClock的值 ， 定义在system_stm32f10x.c    
    SysTick -> LOAD  = msTick -1;          // 自动重载
    SysTick -> VAL   = 0;                  // 清空计数器
    SysTick -> CTRL  = 0;                  // 清0
    SysTick -> CTRL |= 0<<2;               // 0: 时钟=HCLK/8, 1:时钟=HCLK
    SysTick -> CTRL |= 1<<1;               // 使能中断
    SysTick -> CTRL |= 1<<0;               // 使能SysTick	
    
    printf("SysTick时钟配置       1ms中断1次\r");
} 

/*****************************************************************************
 * 函  数：SysTick_Handler
 * 功  能：SysTick中断函数，必须注释掉stm32f10x_it.c中的SysTick_Handler()
 * 参  数：
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124                 2020年04月21日
*****************************************************************************/
void SysTick_Handler(void)
{
    sysTickCnt++;	         // 1ms 加1次	
    
    #ifdef __SCHEDULER_H     // 任务调度器；如果引用了scheduler.h文件，就调用下面的函数
    Scheduler_TickCnt();     // 
    #endif
}

/*****************************************************************************
 * 函  数： System_GetRunTimes
 * 功  能： 获取当前的运行时间，单位：毫秒
 * 参  数：
 * 返回值： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124                 2020年04月21日
*****************************************************************************/
u64 System_GetTimeMs(void)
{    
    return sysTickCnt  ;
}

/*****************************************************************************
 * 函  数： System_GetTimeUs
 * 功  能： 获取系统上电后运行时间数：us
 * 参  数：
 * 返回值： u32 us
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年04月21日
*****************************************************************************/
u32 System_GetTimeUs(void)
{
    u32 ms;
    u32 us;
    do{
        ms = System_GetTimeMs() ;
        us = (float)ms *1000 + (SysTick ->LOAD - SysTick ->VAL )*1000/SysTick->LOAD ;
    }while(ms != System_GetTimeMs() );
    return us;        
}

/*****************************************************************************
 * 函  数： delay_ms
 * 功  能： 毫秒延时;  直接调用无需提前初始化      
 * 参  数：
 * 返回值： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年04月21日
*****************************************************************************/
void delay_ms(u32 ms)
{	
    static u64 _startTime=0;
    
    _startTime = System_GetTimeMs() ;
    while( System_GetTimeMs() - _startTime < ms );    	    
} 

/*****************************************************************************
 * 函  数： delay_us
 * 功  能： 微秒延时; 直接调用无需提前初始化 
 * 参  数： 
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年04月21日
*****************************************************************************/
void delay_us(u32 us)
{
    u64 nowUs = System_GetTimeUs ();
    while(System_GetTimeUs() - nowUs < us);    
}

/*****************************************************************************
 * 函  数： System_GetTimeInterval
 * 功  能： 获取时间间隔，用于测试代码片段运行时间
 * 参  数： 
 * 返回值： 第一次调用返回0, 之后每次调用返回与上次调用的间隔时间(单位:us)
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年04月21日
 *****************************************************************************/ 
u32 System_GetTimeInterval(void)
{
    static u32  lastTime=0, nowTime=0;
    
    lastTime = nowTime ;        
    nowTime = System_GetTimeUs ();               
    
    if(lastTime !=0 )                      // 不是第一次调用 
        return (nowTime-lastTime) ;          
   
    return 0;                              // 第1次调用   
}

/*****************************************************************************
 * 函  数： System_SetTimesPoint
 * 功  能： 调试时使用，获取代码段的运行时长，并输出到串口 
 * 参  数： 
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年04月21日
 *****************************************************************************/ 
void System_SetTimesPoint(void)
{
    static u8 CNT=0;
    
    u32 intervalTimes = System_GetTimeInterval ();
    if(intervalTimes == 0)
    {
        printf("【运行时长 原点位 】\r");
        CNT++;
        System_GetTimeInterval ();
        return;
    }
    
    printf("【运行时长-监察点-%d:%9u us】\r", CNT++, intervalTimes);
    System_GetTimeInterval ();
} 










// ###########################     3 - 三大基本初始化函数   ########################################################



/******************************************************************************
 * 函  数： GPIOSet
 * 功  能： 使能相应时钟、配置引脚
 *          外设时钟使能，必须在GPIO配置前，否则会出现问题
 * 参  数：
 * 返回值： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
******************************************************************************/
void GPIOSet( GPIO_TypeDef* GPIOx,  u32 allPin,  u8 mode,  u8 otype,  u8 ospeed,  u8 pupd)
{
    u32 reg = 0; 
    u32 nowPin=0;    
    
    if(GPIOx==GPIOA )   RCC->APB2ENR |= RCC_APB2ENR_GPIOAEN ;
    if(GPIOx==GPIOB )   RCC->APB2ENR |= RCC_APB2ENR_GPIOBEN ;
    if(GPIOx==GPIOC )   RCC->APB2ENR |= RCC_APB2ENR_GPIOCEN ;
    if(GPIOx==GPIOD )   RCC->APB2ENR |= RCC_APB2ENR_GPIODEN ;
    if(GPIOx==GPIOE )   RCC->APB2ENR |= RCC_APB2ENR_GPIOEEN ;    
     
    // 模拟输入    
    if(mode == GPIO_Mode_AIN){
        reg |= 0;       
    }      
    // 普通输入    
    if(mode == GPIO_Mode_IN) {   
        if(pupd==0)  reg |= 0x01<<2;
        else         reg |= 0x02<<2;
    }
    
    if((ospeed &0x03)==0) ospeed= 0x03;       // 输出速度，
    // 普通输出
    if(mode==GPIO_Mode_OUT){
        reg = ospeed & 0x03;                  // 引脚速度
        reg |= (otype & 0x01)<<2;             // 普通推挽、开漏
    }    
    // 复用输出
    if(mode ==GPIO_Mode_AF){
        reg = ospeed & 0x03;                  // 引脚速度
        reg |= ((otype | 0x02) & 0x03) <<2;   // 复用推挽、开漏
    }      
    
    // CHL, pin 0~7          
    for(u32 i=0; i<8; i++) 
    {      
        nowPin = (u32) 0x01 << i;         // 当前要判断的引脚号     
        if((allPin & nowPin) != 0) {      // 当前引脚要配置
           GPIOx->CRL &= ~(0x0F<<(i*4));  // 清0
           GPIOx->CRL |= reg<<(i*4);      // 写入新配置                
        }          
    }          
    
    // CRH, pin 8~15         
    for(u32 i=0; i<8; i++)     {      
        nowPin = (u32) 0x01 << (i+8);     // 当前要判断的引脚号     
        if((allPin & nowPin) != 0) {      // 当前引脚要配置
           GPIOx->CRH &= ~(0x0F<<(i*4));  // 清0
           GPIOx->CRH |= reg<<(i*4);      // 写入新配置                
        }          
    }             
    
    if(pupd== GPIO_PuPd_UP )   GPIOx->BSRR |= allPin ;
    if(pupd== GPIO_PuPd_DOWN)  GPIOx->BSRR |= allPin << 16;      
}

/******************************************************************************
 * 函  数： NVICSet
 * 功  能： 优先级设置，为方便管理及使用FreeRTOS，统一使用4位抢占级(16级),0位子优先级(0级)
 *         直接调用即可，不用提前配置
 * 参  数： 
 * 返回值： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
******************************************************************************/
void NVICSet(u8 NVIC_Channel, u8 Preemption)
{    
    static u8 setGrouped=0;
    if(setGrouped ==0){
        // 全局分级设置,统一为组4, 值0b11,即：NVIC->IPx中高4位:主级4位(16级), 子级0位(0级）  
        SCB->AIRCR = ((u32)0x05FA0000)|(0x03<<8);   // 优先级分组设置, 已查,是3， F103和F429寄存器通用	    
        setGrouped =1;
    }
    
    // 通道中断优先级设置
    NVIC->IP[NVIC_Channel] &= ~(0xF<<4);                     // 清空 		 
    NVIC->IP[NVIC_Channel]  =  (Preemption&0xF)<<4;          // 写入抢占级\优先级
    // 通道中断使能
    NVIC->ISER[NVIC_Channel/32] |= 1 << (NVIC_Channel % 32); // 使能中断通道	    
    //NVIC->ICER[];   		                                 // 中断失能, 很少用到	   
}

/***************************************************************************** 
 * 函  数： EXTISet
 * 功  能： 外部中断配置函数
 *         重要: 一次只能配置1个IO口,  2020-2-26
 *         只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
 *         该函数会自动开启对应中断,以及屏蔽线  
 *         
 * 参  数： 【GPIOx】:GPIOA~G, 代表GPIOA~G
 *          【BITx】:GPIO_Pin_0~15, 需要使能的位;
 *          【TRIM】:触发模式, EXTI_FTIR/1:下降沿;  EXTI_RTIR/2:上升沿; 3:任意电平触发
 * 返  回： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
*****************************************************************************/
void EXTISet(GPIO_TypeDef* GPIOx, u16 PINx, u8 TRIM)
{
    u8 gpioNum = 0;
    u8 pinNum  = 0;
    
    // 转换GPIOx为数字
    if(GPIOx==GPIOA )  gpioNum=0;
    if(GPIOx==GPIOB )  gpioNum=1;
    if(GPIOx==GPIOC )  gpioNum=2;
    if(GPIOx==GPIOD )  gpioNum=3; 
    if(GPIOx==GPIOE )  gpioNum=4; 
    if(GPIOx==GPIOF )  gpioNum=5; 
    if(GPIOx==GPIOG )  gpioNum=6; 
    
    // 转换PINx为数字
    for(u8 i=0; i<16; i++){
        if( PINx== ((u32)1<<i)){
            pinNum=i;
            break;
        }          
    }    
    
    u8 offSet   = (pinNum%4)*4;                    // 寄存器内偏移
	RCC->APB2ENR |=0x01;                           // 使能io复用时钟			 
	AFIO->EXTICR[pinNum/4] &=~(0x000F << offSet);  // 清0
	AFIO->EXTICR[pinNum/4] |=  gpioNum << offSet;  // EXTI.BITx映射到GPIOx.BITx 
	// 使能line BITx上的中断, 1:使能  0:屏蔽
	EXTI->IMR |= PINx ;                            
	//EXTI->EMR|=1<<BITx;                          // 不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
    // 触发沿
    if(TRIM & 0x01)  EXTI->FTSR |= PINx ;          // line BITx上事件下降沿触发
	if(TRIM & 0x02)  EXTI->RTSR |= PINx ;          // line BITx上事件上升降沿触发
}











// ############################################    4 -   辅助   #################################################################



/******************************************************************************
 * 函  数： System_SwdMode
 * 功  能： 设置芯片调试方式(SWD)
 *          关闭JTAG只保留SWD，可释放引脚PB3、PB4、PA15，只需PA13、PA14
 * 参  数：
 * 返回值： 
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        
*****************************************************************************/
void  System_SwdMode(void)                            
{
	RCC->APB2ENR|=1<<0;           // 开启辅助时钟	   
	AFIO->MAPR &= 0XF8FFFFFF;     // 清0MAPR的[26:24]
	AFIO->MAPR |= 0x2<<24;        // 设置模式  000:全开   010：只开SWD   100:全关 
}



/*****************************************************************************
 * 函  数： GetSystemClock
 * 功  能： 获取系统时钟频率，
 * 参  数：
 * 返回值： u32 当前系统时钟频率
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
*****************************************************************************/
u32 System_GetSystemClock(void) 
{    
    SystemCoreClockUpdate();               // 获取当前时钟频率， 更新全局变量 SystemCoreClock值 
    return SystemCoreClock ;
}



/*****************************************************************************
 * 函  数： System_Reset
 * 功  能： 系统软复位(F103和F429通用) 
 * 参  数：
 * 返回值：
 * 备  注： 魔女开发板团队  资料存放Q群：262901124        最后修改_2020年05月05日
*****************************************************************************/   
void System_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 	



// 采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}

// 关闭所有中断
void System_IntxDisable(void)
{		  
	__ASM volatile("cpsid i");
}

// 开启所有中断
void System_IntxEnable(void)
{
	__ASM volatile("cpsie i");		  
} 

// 进入待机模式	  
void System_Standby(void)
{
	SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)	   
  	RCC->APB1ENR|=1<<28;     //使能电源时钟	    
 	PWR->CSR|=1<<8;          //设置WKUP用于唤醒
	PWR->CR|=1<<2;           //清除Wake-up 标志
	PWR->CR|=1<<1;           //PDDS置位		  
	WFI_SET();				 //执行WFI指令		 
}	  



// 把时钟输出引脚, 方便使用示波器检查时钟是否正确
void System_MCO1Init(u8 source)
{
    RCC->CFGR &= ~(0x0f << 24);
    RCC->CFGR |= (source & 0x0f)<<24;
}
