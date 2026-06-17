#include "adc.h"
#include "delay.h"		 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//初始化ADC
//这里我们仅以规则通道为例
//我们默认仅开启通道1																	   
void  Adc_Init(void)
{    
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//使能ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//使能ADC3时钟

    // 初始化 PF7 作为模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化  
 
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	//复位ADC1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//结束复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	//复位ADC3
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);	//结束复位
 
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA禁用
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; //4分频，确保频率安全
    ADC_CommonInit(&ADC_CommonInitStructure);
	
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
    ADC_InitStructure.ADC_NbrOfConversion = 1;//1次转换
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC3, &ADC_InitStructure);
	
    ADC_TempSensorVrefintCmd(ENABLE); // 使能内部温度传感器

    ADC_Cmd(ADC1, ENABLE);//开启ADC1
    ADC_Cmd(ADC3, ENABLE);//开启ADC3
}				  

u16 Get_Adc_Value(ADC_TypeDef* ADCx, u8 ch)   
{
    ADC_RegularChannelConfig(ADCx, ch, 1, ADC_SampleTime_480Cycles); //480周期采样速度较慢，保证高精度			    
    ADC_SoftwareStartConv(ADCx);		//软件触发转换	
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));//等待转换完成
    return ADC_GetConversionValue(ADCx);	//返回转换结果
}

u16 Get_Adc_Average(ADC_TypeDef* ADCx, u8 ch, u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0;t<times;t++)
    {
        temp_val+=Get_Adc_Value(ADCx, ch);
        delay_ms(5);
    }
    return temp_val/times;
} 
	 









