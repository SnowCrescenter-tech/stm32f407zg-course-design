#ifndef __RTC_H
#define __RTC_H	 
#include "sys.h" 

u8 My_RTC_Init(void);						//RTC初始化
ErrorStatus RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm);			//RTC时间设置
ErrorStatus RTC_Set_Date(u8 year,u8 month,u8 date,u8 week); 		//RTC日期设置
void RTC_Set_AlarmA(u8 week,u8 hour,u8 min,u8 sec);		//闹钟设置
void RTC_Set_WakeUp(u32 wksel,u16 cnt);					//唤醒中断设置
void RTC_BKP_Write(u32 val);
u32 RTC_BKP_Read(void);
void RTC_Get_Time_Str(char *str);

#endif
