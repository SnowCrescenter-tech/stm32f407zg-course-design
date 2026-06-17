#include "stm32f4xx_conf.h"
#include "includes.h"
#include "exti.h"
#include "rtc.h"
#include "led.h"
#include "delay.h"
#include "usart.h" 

NVIC_InitTypeDef   NVIC_InitStructure;

//RTC时间设置
ErrorStatus RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	RTC_TimeTypeInitStructure.RTC_Hours=hour;
	RTC_TimeTypeInitStructure.RTC_Minutes=min;
	RTC_TimeTypeInitStructure.RTC_Seconds=sec;
	RTC_TimeTypeInitStructure.RTC_H12=ampm;
	
	return RTC_SetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
}

//RTC日期设置
ErrorStatus RTC_Set_Date(u8 year,u8 month,u8 date,u8 week)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
	RTC_DateTypeInitStructure.RTC_Date=date;
	RTC_DateTypeInitStructure.RTC_Month=month;
	RTC_DateTypeInitStructure.RTC_WeekDay=week;
	RTC_DateTypeInitStructure.RTC_Year=year;
	return RTC_SetDate(RTC_Format_BIN,&RTC_DateTypeInitStructure);
}

//RTC初始化
u8 My_RTC_Init(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	u16 retry=0X1FFF; 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); //使能PWR时钟
	PWR_BackupAccessCmd(ENABLE);	//使能后备寄存器访问 
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0x5050)		//是否第一次初始化
	{
		RCC_LSEConfig(RCC_LSE_ON); //开启LSE    
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)	//等待LSE就绪
		{
			retry++;
			delay_ms(10);
		}
		if(retry==0)return 1;		//LSE开启失败
			
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//设置RTC时钟源
		RCC_RTCCLKCmd(ENABLE);	//使能RTC时钟 

        RTC_InitStructure.RTC_AsynchPrediv = 0x7F; //异步分频
        RTC_InitStructure.RTC_SynchPrediv  = 0xFF; //同步分频
        RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
        RTC_Init(&RTC_InitStructure);
 
		RTC_Set_Time(23,59,56,RTC_H12_AM);	
		RTC_Set_Date(14,5,5,1);		
	 
		RTC_WriteBackupRegister(RTC_BKP_DR0,0x5050);	
	} 
 
    // 等待影子寄存器同步
    if (RTC_WaitForSynchro() == ERROR)
    {
        return 2; 
    }
 
	return 0;
}

//设置闹钟时间(按星期闹铃,24小时制)
void RTC_Set_AlarmA(u8 week,u8 hour,u8 min,u8 sec)
{ 
	EXTI_InitTypeDef   EXTI_InitStructure;
	RTC_AlarmTypeDef RTC_AlarmTypeInitStructure;
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);
	
    RTC_TimeTypeInitStructure.RTC_Hours=hour;
	RTC_TimeTypeInitStructure.RTC_Minutes=min;
	RTC_TimeTypeInitStructure.RTC_Seconds=sec;
	RTC_TimeTypeInitStructure.RTC_H12=RTC_H12_AM;
  
	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDay=week;
	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDaySel=RTC_AlarmDateWeekDaySel_WeekDay;
	RTC_AlarmTypeInitStructure.RTC_AlarmMask=RTC_AlarmMask_None;
	RTC_AlarmTypeInitStructure.RTC_AlarmTime=RTC_TimeTypeInitStructure;
    RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmTypeInitStructure);
 
	RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
	
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//周期性唤醒定时器设置
void RTC_Set_WakeUp(u32 wksel,u16 cnt)
{ 
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RTC_WakeUpCmd(DISABLE);
	RTC_WakeUpClockConfig(wksel);
	RTC_SetWakeUpCounter(cnt);
	
	RTC_ClearITPendingBit(RTC_IT_WUT); 
    EXTI_ClearITPendingBit(EXTI_Line22);
	 
	RTC_ITConfig(RTC_IT_WUT,ENABLE);
	RTC_WakeUpCmd( ENABLE);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
 
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//RTC闹钟中断服务例程 (已添加中断上下文告知)
void RTC_Alarm_IRQHandler(void)
{    
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)
	{
		RTC_ClearFlag(RTC_FLAG_ALRAF);
	}   
	EXTI_ClearITPendingBit(EXTI_Line17);	
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

//RTC WAKE UP中断服务例程
void RTC_WKUP_IRQHandler(void)
{    
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
	if(RTC_GetFlagStatus(RTC_FLAG_WUTF)==SET)
	{ 
		RTC_ClearFlag(RTC_FLAG_WUTF);
		LED1=!LED1; 
	}   
	EXTI_ClearITPendingBit(EXTI_Line22);
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

//BKP 读写函数
void RTC_BKP_Write(u32 val)
{
    PWR_BackupAccessCmd(ENABLE); 
    RTC_WriteBackupRegister(RTC_BKP_DR1, val);
}

u32 RTC_BKP_Read(void)
{
    return RTC_ReadBackupRegister(RTC_BKP_DR1);
}

//格式化 RTC 时间字符串 "hh:mm:ss"
void RTC_Get_Time_Str(char *str)
{
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct); 
    
    sprintf(str, "%02d:%02d:%02d", RTC_TimeStruct.RTC_Hours, RTC_TimeStruct.RTC_Minutes, RTC_TimeStruct.RTC_Seconds);
}
