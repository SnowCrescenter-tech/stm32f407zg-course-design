#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲区
volatile u8 USART_RX_FLAG = 0;       //接收完毕标志
volatile u16 USART_RX_LEN = 0;      //接收到的字节长度

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);  //使能DMA2时钟
 
    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
    //配置 DMA2 Stream 5 Channel 4 (USART1_RX)
    DMA_DeInit(DMA2_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR; //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART_RX_BUF; //内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设到内存
    DMA_InitStructure.DMA_BufferSize = USART_REC_LEN; //数据长度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //字节单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //单次模式，每次接收完在空闲中断中重置
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    
    //开启串口接收 DMA 请求
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    //启动 DMA
    DMA_Cmd(DMA2_Stream5, ENABLE);
    
    //开启串口1
    USART_Cmd(USART1, ENABLE);  
	
#if EN_USART1_RX	
    //使能空闲中断 IDLE
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//初始化NVIC

#endif
	
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
    volatile u32 temp;
    u32 rx_left;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
    //如果是串口空闲中断
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
	{
        //清除空闲中断标志 (先读SR再读DR)
        temp = USART1->SR;
        temp = USART1->DR;
        
        // 1. 关闭 DMA Stream 提取数据
        DMA_Cmd(DMA2_Stream5, DISABLE);
        while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}
        
        // 2. 计算已接收数据长度
        rx_left = DMA_GetCurrDataCounter(DMA2_Stream5);
        USART_RX_LEN = USART_REC_LEN - rx_left;
        
        if (USART_RX_LEN > 0)
        {
            if (USART_RX_LEN >= USART_REC_LEN) 
            {
                USART_RX_LEN = USART_REC_LEN - 1;
            }
            USART_RX_BUF[USART_RX_LEN] = '\0'; // 写入字符串结束符
            USART_RX_FLAG = 1; // 标记接收完毕
        }
        
        // 3. 重新设置缓冲区大小并使能 DMA
        DMA_SetCurrDataCounter(DMA2_Stream5, USART_REC_LEN);
        DMA_Cmd(DMA2_Stream5, ENABLE);
	} 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif
