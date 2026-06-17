#include <stdio.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "lcd.h"
#include "adc.h"
#include "key.h"
#include "exti.h"
#include "rtc.h"
#include "sram.h"
#include "string.h"
#include "math.h"
#include "dac.h"

/* 外部 SRAM 双缓冲显存定义 (基地址 0x68000000) */
#define SRAM_BANK3_ADDR ((u32)0x68000000)
u16 *UI_Buffer = (u16 *)SRAM_BANK3_ADDR;

#define UI_W 480
#define UI_H 800

/* 任务优先级与堆栈大小定义 */
#define START_TASK_PRIO 10
#define START_STK_SIZE 128
__align(8) OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);

#define CONTROL_TASK_PRIO 4
#define CONTROL_STK_SIZE 256
__align(8) OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
void control_task(void *pdata);

#define SENSOR_TASK_PRIO 5
#define SENSOR_STK_SIZE 512 /* float+FPU需要更大栈空间 */
__align(8) OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
void sensor_task(void *pdata);

#define COM_TASK_PRIO 6
#define COM_STK_SIZE 512
__align(8) OS_STK COM_TASK_STK[COM_STK_SIZE];
void com_task(void *pdata);

#define UI_TASK_PRIO 7
#define UI_STK_SIZE 768 /* 大屏(480x800)绘图需要更大栈 */
__align(8) OS_STK UI_TASK_STK[UI_STK_SIZE];
void ui_task(void *pdata);

/* 内核对象指针 */
OS_EVENT *UI_Queue;
OS_EVENT *COM_Queue;
OS_EVENT *Key_Sem;
OS_EVENT *Config_Mutex;

#define QUEUE_MAX_SIZE 5
void *UI_Msg_Buf[QUEUE_MAX_SIZE];
void *COM_Msg_Buf[QUEUE_MAX_SIZE];

/* 全局状态变量 */
u8 UI_Mode = 0;    /* 0: 折线图, 1: 仪表盘, 2: 原始日志 */
u8 g_Led1_Cmd = 1; /* LED1 控制字: 0=串口命令关闭, 1=串口命令打开, 默认开 */
volatile u8 Menu_State = 0;   /* 0: 待机, 1: 菜单导航, 2: 数值编辑 */
volatile u8 Menu_Cursor = 0;  /* 0~5 代表 6 个菜单项 */

/* 传感器数据传输结构体 */
typedef struct
{
    int temperature;  /* 放大 10 倍的温度值，如 385 代表 38.5C */
    uint16_t light;   /* 归一化亮度百分比 (0~100) */
    uint16_t dac_vol; /* DAC输出电压 (mV) */
    uint16_t adc_vol; /* ADC回读电压 (mV) */
    uint8_t time[10];
} SensorData_t;

/* 系统配置参数结构体 */
typedef struct
{
    int temp_limit; /* 放大 10 倍的温度限制，如 385 代表 38.5C */
    uint16_t light_limit;
    u8 emergency_stop;
    u8 beep_mute;
    u8 led_state;
} GlobalConfig_t;

GlobalConfig_t GlobalConfig;

/* 曲线历史缓冲区 */
#define PLOT_POINTS_MAX 40
int Temp_History[PLOT_POINTS_MAX];
uint16_t Light_History[PLOT_POINTS_MAX];
u8 History_Count = 0;
u8 History_Head = 0;

/* 通信日志缓冲区 */
#define LOG_LINES_MAX 20
char Comm_Logs[LOG_LINES_MAX][80];
u8 Log_Count = 0;
u8 Log_Head = 0;

/* 共享缓冲区与控制变量声明 */
static char str_buf[128];
static char temp_logs[20][80];
static int backup_temp = 380;
static uint16_t backup_light = 80;

/* ================== 所有子函数原型声明 ================== */
static void BKP_Light_Write(uint16_t val);
static uint16_t BKP_Light_Read(void);
static void UI_Clear_Area(u16 sx, u16 sy, u16 width, u16 height, u16 color);
static void LCD_Flush_Area(u16 sx, u16 sy, u16 width, u16 height);
static void check_watchdog_reset(void);
static void load_system_config(void);
static void iwdg_init(void);
static void sensor_alarm_control(u8 alarm_type);
static void control_handle_standby(void);
static void control_handle_menu_nav(void);
static void control_handle_menu_edit(void);
static void ui_draw_curve(u8 menu_s, int limit);
static void ui_draw_dashboard(SensorData_t *data, int limit);
static void ui_draw_terminal_logs(u8 force_clear);
static void ui_draw_bottom_bar(SensorData_t *data, int limit, u8 led_st, u8 mute);
static void ui_draw_menu(u8 menu_s, u8 menu_c, int limit, u8 is_emergency, u8 mute, u8 menu_bg_draw);

/* ================== 公共日志添加函数 ================== */
void Add_Comm_Log(const char *log_str)
{
    OS_CPU_SR cpu_sr;
    u8 tail;

    cpu_sr = 0;
    OS_ENTER_CRITICAL(); /* 进入临界区 */

    tail = (Log_Head + Log_Count) % LOG_LINES_MAX;
    if (Log_Count < LOG_LINES_MAX)
    {
        Log_Count++;
    }
    else
    {
        Log_Head = (Log_Head + 1) % LOG_LINES_MAX;
    }
    strncpy(Comm_Logs[tail], log_str, 79);
    Comm_Logs[tail][79] = '\0';

    OS_EXIT_CRITICAL(); /* 退出临界区 */
}

/* ================== 底层画图 API ================== */
void UI_Clear(u16 color)
{
    u32 total;
    u32 i;

    total = (u32)lcddev.width * lcddev.height;
    for (i = 0; i < total; i++)
    {
        UI_Buffer[i] = color;
    }
}

void UI_DrawPoint(u16 x, u16 y, u16 color)
{
    if (x < lcddev.width && y < lcddev.height)
    {
        UI_Buffer[y * lcddev.width + x] = color;
    }
}

void UI_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    u16 t;
    int xerr;
    int yerr;
    int delta_x;
    int delta_y;
    int distance;
    int incx;
    int incy;
    int uRow;
    int uCol;

    xerr = 0;
    yerr = 0;
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0)
        incx = 1;
    else if (delta_x == 0)
        incx = 0;
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0;
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
        distance = delta_x;
    else
        distance = delta_y;

    for (t = 0; t <= distance + 1; t++)
    {
        UI_DrawPoint(uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/* ================== 备份寄存器与 SRAM 辅助静态函数 ================== */
static void BKP_Light_Write(uint16_t val)
{
    PWR_BackupAccessCmd(ENABLE); 
    RTC_WriteBackupRegister(RTC_BKP_DR2, val);
}

static uint16_t BKP_Light_Read(void)
{
    return (uint16_t)RTC_ReadBackupRegister(RTC_BKP_DR2);
}

static void UI_Clear_Area(u16 sx, u16 sy, u16 width, u16 height, u16 color)
{
    u16 x;
    u16 y;
    for (y = sy; y < sy + height; y++)
    {
        for (x = sx; x < sx + width; x++)
        {
            UI_Buffer[y * (u32)lcddev.width + x] = color;
        }
    }
}

static void LCD_Flush_Area(u16 sx, u16 sy, u16 width, u16 height)
{
    u16 x;
    u16 y;
    LCD_Set_Window(sx, sy, width, height);
    LCD_WriteRAM_Prepare();
    for (y = sy; y < sy + height; y++)
    {
        for (x = sx; x < sx + width; x++)
        {
            LCD->LCD_RAM = UI_Buffer[y * (u32)lcddev.width + x];
        }
    }
    LCD_Set_Window(0, 0, lcddev.width, lcddev.height);
}

/* ================== 重构提取的系统/传感器控制子函数 ================== */

/**
 * @brief  检查系统是否由于独立看门狗 (IWDG) 复位引起
 * @note   若检测到 IWDG 复位，则全屏提示 "SYSTEM CRASH DETECTED"，蜂鸣器急促鸣叫4声，提示用户系统曾发生卡死并已自动重启。
 * @param  无
 * @retval 无
 */
static void check_watchdog_reset(void)
{
    int k;
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
    {
        RCC_ClearFlag(); /* 清除复位标志，防反复报警 */
        
        /* 全屏灰色背景 */
        LCD_Fill(0, 0, lcddev.width - 1, lcddev.height - 1, GRAY);
        
        /* 打印卡死和复位提示 */
        POINT_COLOR = RED;
        BACK_COLOR = GRAY;
        LCD_ShowString((lcddev.width - 240) / 2, lcddev.height / 2 - 40, 240, 24, 24, (u8 *)"SYSTEM CRASH DETECTED");
        POINT_COLOR = WHITE;
        LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2, 200, 16, 16, (u8 *)"Rebooting system...");
        
        /* BEEP 急促发出四声短促滴滴滴滴 */
        for (k = 0; k < 4; k++)
        {
            BEEP = 1;
            delay_ms(100);
            BEEP = 0;
            delay_ms(100);
        }
        delay_ms(500); /* 维持半秒让用户看清提示 */
    }
}

/**
 * @brief  系统配置参数载入函数
 * @note   读取 RTC 备份寄存器固化的温度和光照限制参数。若首次上电（无有效备份标志），则载入默认配置。
 * @param  无
 * @retval 无
 */
static void load_system_config(void)
{
    u32 bkp_val;
    if (My_RTC_Init() == 0)
    {
        bkp_val = RTC_BKP_Read();
        if ((bkp_val >> 16) == 0x55AA)
        {
            GlobalConfig.temp_limit = (int)(bkp_val & 0xFFFF);
        }
        else
        {
            GlobalConfig.temp_limit = 380; /* 38.0C 放大 10 倍 */
            RTC_BKP_Write((0x55AA << 16) | (uint16_t)380);
        }

        bkp_val = BKP_Light_Read();
        if ((bkp_val >> 8) == 0x55)
        {
            GlobalConfig.light_limit = bkp_val & 0xFF;
        }
        else
        {
            GlobalConfig.light_limit = 80; /* 默认光照报警值 80% */
            BKP_Light_Write((0x55 << 8) | 80);
        }
    }
    else
    {
        GlobalConfig.temp_limit = 380;
        GlobalConfig.light_limit = 80;
    }
    GlobalConfig.emergency_stop = 0;
    GlobalConfig.beep_mute = 0;
    GlobalConfig.led_state = 0;
}

/**
 * @brief  初始化并开启独立看门狗 (IWDG)
 * @note   分频系数设置为 64 (500Hz)，重装载值为 2000，溢出周期约为 4 秒 (2000 * 2ms = 4s)。
 * @param  无
 * @retval 无
 */
static void iwdg_init(void)
{
    IWDG->KR = 0x5555; /* 开启寄存器写权限 */
    IWDG->PR = 4;      /* 64分频 (32kHz / 64 = 500Hz, 每刻度 2ms) */
    IWDG->RLR = 2000;  /* 重装载值 2000 (2000 * 2ms = 4000ms = 4秒) */
    IWDG->KR = 0xAAAA; /* 重载计数器 */
    IWDG->KR = 0XCCCC; /* 启动看门狗 */
}

/**
 * @brief  50ms 高频声光报警状态机控制
 * @note   根据当前的报警类型 (0=正常, 1=普通超限, 2=紧急拉闸) 周期性控制红灯和有源蜂鸣器的状态。
 * @param  alarm_type: 报警类型
 * @retval 无
 */
static void sensor_alarm_control(u8 alarm_type)
{
    u8 err;
    u8 mute;
    static u8 beep_tick = 0;
    static u8 last_alarm_type = 0;

    /* 状态发生改变时重置蜂鸣器节拍 */
    if (alarm_type != last_alarm_type)
    {
        last_alarm_type = alarm_type;
        beep_tick = 0;
    }

    OSMutexPend(Config_Mutex, 0, &err);
    mute = GlobalConfig.beep_mute;
    OSMutexPost(Config_Mutex);

    if (alarm_type == 2) /* 紧急报警：红灯高频闪烁，有源蜂鸣器急促短鸣 */
    {
        if (beep_tick >= 12) beep_tick = 0; /* 0.6秒周期 */
        if (!mute)
        {
            BEEP = (beep_tick % 4 < 2) ? 1 : 0; /* 100ms 鸣叫, 100ms 停止 */
        }
        else
        {
            BEEP = 0;
        }
        LED0 = (beep_tick % 4 < 2) ? 0 : 1; /* 红灯同步 5Hz 闪烁 */
        beep_tick++;
    }
    else if (alarm_type == 1) /* 普通超限报警：红灯长亮，有源蜂鸣器“滴-滴-嘀——”循环 */
    {
        if (beep_tick >= 24) beep_tick = 0; /* 1.2秒周期 */
        if (!mute)
        {
            if (beep_tick < 2) BEEP = 1;        /* 0-100ms 鸣叫 (滴) */
            else if (beep_tick < 4) BEEP = 0;   /* 100-200ms 停止 */
            else if (beep_tick < 6) BEEP = 1;   /* 200-300ms 鸣叫 (滴) */
            else if (beep_tick < 8) BEEP = 0;   /* 300-400ms 停止 */
            else if (beep_tick < 13) BEEP = 1;  /* 400-650ms 鸣叫 (嘀——) */
            else BEEP = 0;                      /* 650-1200ms 停止 */
        }
        else
        {
            BEEP = 0;
        }
        LED0 = 0; /* 红灯长亮 */
        beep_tick++;
    }
    else /* 正常无警报 */
    {
        BEEP = 0;
        LED0 = 1; /* 红灯熄灭 */
        beep_tick = 0;
    }
}

/* ================== 重构提取的按键控制子函数 ================== */

/**
 * @brief  待机模式下的物理按键逻辑处理
 * @note   在待机状态下，根据触发的按键执行呼出菜单、紧急拉闸/恢复、消音以及UI页面切换。
 * @param  无
 * @retval 无
 */
static void control_handle_standby(void)
{
    if (Last_Key == 0) /* KEY0: 呼出系统菜单 */
    {
        Menu_State = 1;
        Menu_Cursor = 0;
    }
    else if (Last_Key == 2) /* KEY2: 紧急拉闸/恢复 */
    {
        GlobalConfig.emergency_stop = !GlobalConfig.emergency_stop;
        if (GlobalConfig.emergency_stop == 0)
        {
            BEEP = 0;
            LED0 = 1;
        }
    }
    else if (Last_Key == 3) /* WK_UP: 消音 */
    {
        GlobalConfig.beep_mute = 1;
    }
    else if (Last_Key == 1) /* KEY1 (DOWN): 待机下直接向下循环切换 UI 页面 */
    {
        UI_Mode = (UI_Mode + 1) % 3;
    }
    if (Last_Key == 3)
    {
        UI_Mode = (UI_Mode + 2) % 3; /* 向上循环切换 UI 页面 */
    }
}

/**
 * @brief  菜单导航模式下的物理按键逻辑处理
 * @note   在一级菜单导航状态下，处理光标移动、确认进入二级编辑菜单以及返回退出菜单。
 * @param  无
 * @retval 无
 */
static void control_handle_menu_nav(void)
{
    if (Last_Key == 3) /* WK_UP: 向上选择菜单项 */
    {
        if (Menu_Cursor > 0) Menu_Cursor--;
        else Menu_Cursor = 5;
    }
    else if (Last_Key == 1) /* KEY1: 向下选择菜单项 */
    {
        if (Menu_Cursor < 5) Menu_Cursor++;
        else Menu_Cursor = 0;
    }
    else if (Last_Key == 0) /* KEY0: 确认选中当前项 */
    {
        if (Menu_Cursor == 5) /* Exit */
        {
            Menu_State = 0;
        }
        else if (Menu_Cursor == 2) /* Emg Stop 直接切换 ON/OFF */
        {
            GlobalConfig.emergency_stop = !GlobalConfig.emergency_stop;
            if (GlobalConfig.emergency_stop == 0)
            {
                BEEP = 0;
                LED0 = 1;
            }
        }
        else if (Menu_Cursor == 3) /* Beep Mute 直接切换 ON/OFF */
        {
            GlobalConfig.beep_mute = !GlobalConfig.beep_mute;
        }
        else /* Temp Limit (0), Light Limit (1), Crash Sim (4) 进入状态 2 */
        {
            Menu_State = 2;
            backup_temp = GlobalConfig.temp_limit;
            backup_light = GlobalConfig.light_limit;
        }
    }
    else if (Last_Key == 2) /* KEY2: 返回并退出菜单 */
    {
        Menu_State = 0;
    }
}

/**
 * @brief  二级数值编辑与卡死仿真处理
 * @note   在参数编辑模式下高频周期性轮询物理引脚电平，处理参数调节（温度、光照）、卡死仿真启动、确认与放弃修改逻辑。
 * @param  无
 * @retval 无
 */
static void control_handle_menu_edit(void)
{
    u8 pin_key;
    u8 err;
    u16 menu_w;
    u16 menu_h;
    u16 menu_x;
    u16 menu_y;

    pin_key = 0; /* 0=未按, 1=WK_UP(加), 2=KEY1(减), 3=KEY0(确定), 4=KEY2(回退) */
    
    /* 直接读取按键的物理 GPIO 电平 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1)      pin_key = 1; /* WK_UP 物理高电平代表按下 */
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0) pin_key = 2; /* KEY1 物理低电平代表按下 */
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0) pin_key = 3; /* KEY0 物理低电平代表按下 */
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0) pin_key = 4; /* KEY2 物理低电平代表按下 */

    if (pin_key > 0)
    {
        OSMutexPend(Config_Mutex, 0, &err);
        if (Menu_Cursor == 0) /* 编辑 Temp Limit (15.0 ~ 50.0) */
        {
            if (pin_key == 1) /* WK_UP (+) */
            {
                GlobalConfig.temp_limit += 5; /* 增加 0.5C */
                if (GlobalConfig.temp_limit > 500) GlobalConfig.temp_limit = 500;
            }
            else if (pin_key == 2) /* KEY1 (-) */
            {
                GlobalConfig.temp_limit -= 5; /* 减少 0.5C */
                if (GlobalConfig.temp_limit < 150) GlobalConfig.temp_limit = 150;
            }
            else if (pin_key == 3) /* KEY0 (确认保存并返回) */
            {
                RTC_BKP_Write((0x55AA << 16) | (uint16_t)GlobalConfig.temp_limit);
                Menu_State = 1;
            }
            else if (pin_key == 4) /* KEY2 (放弃修改并返回) */
            {
                GlobalConfig.temp_limit = backup_temp; /* 恢复旧值 */
                Menu_State = 1;
            }
        }
        else if (Menu_Cursor == 1) /* 编辑 Light Limit (10 ~ 95) */
        {
            if (pin_key == 1) /* WK_UP (+) */
            {
                GlobalConfig.light_limit += 5;
                if (GlobalConfig.light_limit > 95) GlobalConfig.light_limit = 95;
            }
            else if (pin_key == 2) /* KEY1 (-) */
            {
                GlobalConfig.light_limit -= 5;
                if (GlobalConfig.light_limit < 10) GlobalConfig.light_limit = 10;
            }
            else if (pin_key == 3) /* KEY0 (确认保存并返回) */
            {
                BKP_Light_Write((0x55 << 8) | GlobalConfig.light_limit);
                Menu_State = 1;
            }
            else if (pin_key == 4) /* KEY2 (放弃修改并返回) */
            {
                GlobalConfig.light_limit = backup_light; /* 恢复旧值 */
                Menu_State = 1;
            }
        }
        else if (Menu_Cursor == 4) /* 编辑 Crash Sim */
        {
            if (pin_key == 1 || pin_key == 3) /* WK_UP (+) 或 KEY0 (确定) */
            {
                menu_w = 320;
                menu_h = 250;
                menu_x = (lcddev.width - menu_w) / 2;
                menu_y = (lcddev.height - menu_h) / 2;
                
                OSMutexPost(Config_Mutex); /* 释放信号量 */
                
                /* 覆盖画红底白字弹窗提示 */
                LCD_Fill(menu_x, menu_y, menu_x + menu_w - 1, menu_y + menu_h - 1, RED);
                POINT_COLOR = WHITE;
                BACK_COLOR = RED;
                LCD_ShowString(menu_x + 30, menu_y + 80, 260, 24, 24, (u8*)"CRASH SIMULATION");
                LCD_ShowString(menu_x + 20, menu_y + 130, 280, 16, 16, (u8*)"System will crash in 2s...");
                
                /* 延时 2 秒让用户看清提示 */
                OSTimeDly(200); 
                
                /* 关中断并死循环，彻底卡死系统 */
                __disable_irq();
                while (1)
                    ;
            }
            else if (pin_key == 2 || pin_key == 4) /* KEY1 (-) 或 KEY2 (返回) */
            {
                Menu_State = 1; /* 取消并返回 */
            }
        }
        OSMutexPost(Config_Mutex);

        /* 发送信号重绘，让数值和“竖着的条条”进度柱立刻变化！ */
        OSQPost(UI_Queue, (void *)0xFFFFFFFF);

        /* 根据按键类型做不同的延时 */
        if (pin_key == 3 || pin_key == 4)
        {
            /* 确认/返回键：必须等待所有物理按键完全释放以防穿透与连跳 */
            while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0)
            {
                OSTimeDly(2); /* 每次延时 20ms 等待松手 */
            }
            OSTimeDly(5); /* 松手后消抖 50ms */
            
            /* 彻底清除在这期间的所有挂起中断，并重新使能外部中断 */
            EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4);
            EXTI->IMR |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4);
            
            /* 彻底排空在轮询与释放期间，中断所记录的所有残留信号量，防穿透 */
            while (OSSemAccept(Key_Sem) > 0);
            Last_Key = 0xFF;
        }
        else
        {
            OSTimeDly(10); /* 阈值加减调节：长按快滚延时 (100ms) 带来流畅连调体验 */
        }
    }
    else
    {
        /* 无按键按下时，延迟 20ms，降低 CPU 负载并防止高频扫描 */
        OSTimeDly(2);
    }
}

/* ================== 重构提取的 UI 绘制子函数 ================== */

/**
 * @brief  绘制折线图看板
 * @note   在外部 SRAM 显存中离屏绘制网格线、坐标轴、双曲线（温度曲线和光照曲线）以及温度/光照限制线，最后一次性拷贝刷新到物理 LCD。
 * @param  menu_s: 菜单状态
 * @param  limit: 温度限制值
 * @retval 无
 */
static void ui_draw_curve(u8 menu_s, int limit)
{
    u16 grid_x;
    u16 grid_w;
    u16 grid_y;
    u16 grid_h;
    int i;
    int y_limit;
    int step_x;
    int idx1;
    int idx2;
    int t1;
    int t2;
    int x1;
    int x2;
    int y1;
    int y2;

    grid_x = 55;
    grid_w = lcddev.width - 110;
    grid_y = 40;
    grid_h = lcddev.height - 200;

    /* 局部擦除外部 SRAM 中的对应显示区域 */
    if (menu_s == 0) 
    {
        UI_Clear_Area(grid_x, grid_y, grid_w + 1, grid_h + 1, BLACK);
    }

    /* 在外部 SRAM 显存中离屏绘制网格背景线 */
    for (i = 0; i <= 6; i++)
    {
        UI_DrawLine(grid_x, grid_y + i * (grid_h / 6), grid_x + grid_w, grid_y + i * (grid_h / 6), GRAY);
    }
    for (i = 0; i <= 8; i++)
    {
        UI_DrawLine(grid_x + i * (grid_w / 8), grid_y, grid_x + i * (grid_w / 8), grid_y + grid_h, GRAY);
    }

    /* 在外部 SRAM 显存中离屏绘制坐标轴边框 */
    UI_DrawLine(grid_x, grid_y, grid_x, grid_y + grid_h, WHITE);
    UI_DrawLine(grid_x, grid_y + grid_h, grid_x + grid_w, grid_y + grid_h, WHITE);
    UI_DrawLine(grid_x + grid_w, grid_y, grid_x + grid_w, grid_y + grid_h, WHITE);

    /* 绘制左侧温度刻度轴 (10 ~ 60 C，直接绘制在 LCD 上，不受显存刷新清空影响) */
    POINT_COLOR = GREEN;
    BACK_COLOR = BLACK;
    for (i = 0; i <= 6; i++)
    {
        sprintf(str_buf, "%2d", 60 - i * 10);
        LCD_ShowString(grid_x - 30, grid_y + i * (grid_h / 6) - 8, 25, 16, 16, (u8*)str_buf);
    }
    LCD_ShowString(grid_x - 20, grid_y - 22, 40, 16, 16, (u8*)"T(C)");

    /* 绘制右侧光照刻度轴 (0 ~ 100 %，直接绘制在 LCD 上) */
    POINT_COLOR = YELLOW;
    BACK_COLOR = BLACK;
    for (i = 0; i <= 6; i++)
    {
        sprintf(str_buf, "%3d", 100 - i * 20);
        LCD_ShowString(grid_x + grid_w + 8, grid_y + i * (grid_h / 6) - 8, 30, 16, 16, (u8*)str_buf);
    }
    LCD_ShowString(grid_x + grid_w - 20, grid_y - 22, 40, 16, 16, (u8*)"L(%)");

    /* 绘制底部时间坐标偏移 (直接绘制在 LCD 上) */
    POINT_COLOR = WHITE;
    LCD_ShowString(grid_x, grid_y + grid_h + 5, 40, 16, 16, (u8*)"-40s");
    LCD_ShowString(grid_x + grid_w / 2 - 15, grid_y + grid_h + 5, 40, 16, 16, (u8*)"-20s");
    LCD_ShowString(grid_x + grid_w - 25, grid_y + grid_h + 5, 30, 16, 16, (u8*)"0s");

    /* 红色温度阈值线 (在外部 SRAM 中绘制) */
    if (limit >= 100 && limit <= 600)
    {
        y_limit = grid_y + grid_h - (limit - 100) * grid_h / 500;
        if (y_limit >= grid_y && y_limit <= grid_y + grid_h)
        {
            UI_DrawLine(grid_x, y_limit, grid_x + grid_w, y_limit, RED);
        }
    }

    /* 黄色光照阈值虚线 (在外部 SRAM 中绘制) */
    if (GlobalConfig.light_limit <= 100)
    {
        int y_lim_l = grid_y + grid_h - (u32)GlobalConfig.light_limit * grid_h / 100;
        if (y_lim_l >= grid_y && y_lim_l <= grid_y + grid_h)
        {
            for (i = 0; i < grid_w; i += 8)
            {
                UI_DrawLine(grid_x + i, y_lim_l, grid_x + i + 4, y_lim_l, YELLOW);
            }
        }
    }

    step_x = grid_w / PLOT_POINTS_MAX;
    if (step_x == 0) step_x = 10;

    /* 绘制绿色温度曲线 (在外部 SRAM 中绘制) */
    for (i = 0; i < (int)History_Count - 1; i++)
    {
        idx1 = (History_Head + i) % PLOT_POINTS_MAX;
        idx2 = (History_Head + i + 1) % PLOT_POINTS_MAX;
        t1 = Temp_History[idx1];
        t2 = Temp_History[idx2];
        x1 = grid_x + i * step_x;
        x2 = grid_x + (i + 1) * step_x;
        y1 = grid_y + grid_h - (t1 - 100) * grid_h / 500;
        y2 = grid_y + grid_h - (t2 - 100) * grid_h / 500;
        if (y1 < grid_y) y1 = grid_y;
        if (y1 > grid_y + grid_h) y1 = grid_y + grid_h;
        if (y2 < grid_y) y2 = grid_y;
        if (y2 > grid_y + grid_h) y2 = grid_y + grid_h;
        UI_DrawLine(x1, y1, x2, y2, GREEN);
    }

    /* 绘制黄色光照曲线 (在外部 SRAM 中绘制) */
    for (i = 0; i < (int)History_Count - 1; i++)
    {
        idx1 = (History_Head + i) % PLOT_POINTS_MAX;
        idx2 = (History_Head + i + 1) % PLOT_POINTS_MAX;
        t1 = Light_History[idx1];
        t2 = Light_History[idx2];
        x1 = grid_x + i * step_x;
        x2 = grid_x + (i + 1) * step_x;
        y1 = grid_y + grid_h - (u32)t1 * grid_h / 100;
        y2 = grid_y + grid_h - (u32)t2 * grid_h / 100;
        if (y1 < grid_y) y1 = grid_y;
        if (y1 > grid_y + grid_h) y1 = grid_y + grid_h;
        if (y2 < grid_y) y2 = grid_y;
        if (y2 > grid_y + grid_h) y2 = grid_y + grid_h;
        UI_DrawLine(x1, y1, x2, y2, YELLOW);
    }

    /* 画完后一瞬间将外部 SRAM 数据拷贝并刷新到物理 LCD */
    if (menu_s == 0)
    {
        LCD_Flush_Area(grid_x, grid_y, grid_w + 1, grid_h + 1);
    }
}

/**
 * @brief  绘制仪表盘看板
 * @note   在物理 LCD 上绘制四个通道（温度、光照强度、DAC 输出电压、ADC 回读电压）的进度条以及实时数值。
 * @param  data: 指向当前传感器数据结构体的指针
 * @param  limit: 温度限制值
 * @retval 无
 */
static void ui_draw_dashboard(SensorData_t *data, int limit)
{
    u16 bar_w;
    u16 fill_w;
    int temp_val;
    int temp_abs;

    bar_w = lcddev.width - 80;
    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    
    /* 1. TEMPERATURE GAUGE */
    LCD_ShowString(40, 55, 200, 16, 16, (u8*)"1. TEMPERATURE (C)");
    temp_val = data->temperature;
    temp_abs = temp_val < 0 ? -temp_val : temp_val;
    /* 尾部增加空格以消除数值位宽减小时的残留像素残影 */
    sprintf(str_buf, "%s%d.%d   ", temp_val < 0 ? "-" : "", temp_abs / 10, temp_abs % 10);
    LCD_ShowString(lcddev.width - 120, 55, 80, 16, 16, (u8*)str_buf);
    
    if (data->temperature < 100) fill_w = 0;
    else if (data->temperature > 600) fill_w = bar_w;
    else fill_w = (u16)((data->temperature - 100) * bar_w / 500);
    
    LCD_Fill(40, 75, 40 + fill_w - 1, 95 - 1, (data->temperature > limit) ? RED : GREEN);
    if (fill_w < bar_w) LCD_Fill(40 + fill_w, 75, 40 + bar_w - 1, 95 - 1, DARKBLUE);
    
    /* 2. LIGHT STRENGTH GAUGE */
    LCD_ShowString(40, 125, 200, 16, 16, (u8*)"2. LIGHT STRENGTH (%)");
    /* 尾部增加空格 */
    sprintf(str_buf, "%d%%   ", data->light);
    LCD_ShowString(lcddev.width - 120, 125, 80, 16, 16, (u8*)str_buf);
    
    fill_w = (u16)((u32)data->light * bar_w / 100);
    if (fill_w > bar_w) fill_w = bar_w;
    
    LCD_Fill(40, 145, 40 + fill_w - 1, 165 - 1, (data->light > GlobalConfig.light_limit) ? RED : YELLOW);
    if (fill_w < bar_w) LCD_Fill(40 + fill_w, 145, 40 + bar_w - 1, 165 - 1, DARKBLUE);

    /* 3. DAC OUTPUT GAUGE (PA4) */
    LCD_ShowString(40, 195, 200, 16, 16, (u8*)"3. DAC OUTPUT (PA4)");
    /* 尾部增加空格，并且将颜色由暗沉的蓝色更换为青色 */
    sprintf(str_buf, "%d.%02dV  ", data->dac_vol / 1000, (data->dac_vol % 1000) / 10);
    LCD_ShowString(lcddev.width - 120, 195, 80, 16, 16, (u8*)str_buf);
    
    fill_w = (u16)((u32)data->dac_vol * bar_w / 3300);
    if (fill_w > bar_w) fill_w = bar_w;
    
    LCD_Fill(40, 215, 40 + fill_w - 1, 235 - 1, CYAN);
    if (fill_w < bar_w) LCD_Fill(40 + fill_w, 215, 40 + bar_w - 1, 235 - 1, DARKBLUE);

    /* 4. ADC LOOPBACK GAUGE (PA5) */
    LCD_ShowString(40, 265, 200, 16, 16, (u8*)"4. ADC LOOPBACK (PA5)");
    /* 尾部增加空格 */
    sprintf(str_buf, "%d.%02dV  ", data->adc_vol / 1000, (data->adc_vol % 1000) / 10);
    LCD_ShowString(lcddev.width - 120, 265, 80, 16, 16, (u8*)str_buf);
    
    fill_w = (u16)((u32)data->adc_vol * bar_w / 3300);
    if (fill_w > bar_w) fill_w = bar_w;
    
    LCD_Fill(40, 285, 40 + fill_w - 1, 305 - 1, MAGENTA);
    if (fill_w < bar_w) LCD_Fill(40 + fill_w, 285, 40 + bar_w - 1, 305 - 1, DARKBLUE);
}

/**
 * @brief  绘制串口通信终端日志看板
 * @note   在屏幕中部绘制一个终端框，提取并显示最近 6 条串口通信日志，自动处理单条日志的换行，擦除空白部分。
 * @param  force_clear: 是否强制重绘框体背景和标题顶栏
 * @retval 无
 */
static void ui_draw_terminal_logs(u8 force_clear)
{
    u16 box_x;
    u16 box_w;
    u16 box_y;
    u16 box_h;
    int draw_y;
    u8 temp_log_count;
    u8 temp_log_head;
    int i;
    int log_idx;
    OS_CPU_SR cpu_sr;
    u8 show_cnt;
    u8 start_i;

    box_x = 30;
    box_w = lcddev.width - 60;
    box_y = 40;
    box_h = lcddev.height - 230;

    if (force_clear)
    {
        /* 绘制框体背景 */
        LCD_Fill(box_x, box_y, box_x + box_w - 1, box_y + box_h - 1, BLACK);
        
        /* 绘制标题顶栏 (深蓝色) */
        LCD_Fill(box_x + 1, box_y + 1, box_x + box_w - 2, box_y + 30, DARKBLUE);
        
        /* 绘制灰色细边框 */
        POINT_COLOR = GRAY;
        LCD_DrawLine(box_x, box_y, box_x + box_w, box_y);
        LCD_DrawLine(box_x, box_y + box_h, box_x + box_w, box_y + box_h);
        LCD_DrawLine(box_x, box_y, box_x, box_y + box_h);
        LCD_DrawLine(box_x + box_w, box_y, box_x + box_w, box_y + box_h);
        
        /* 绘制标题文本 */
        POINT_COLOR = WHITE;
        BACK_COLOR = DARKBLUE;
        LCD_ShowString(box_x + 15, box_y + 8, box_w - 30, 16, 16, (u8 *)"COMMUNICATION TERMINAL MONITOR");
    }

    cpu_sr = 0;
    OS_ENTER_CRITICAL();
    temp_log_count = Log_Count;
    temp_log_head = Log_Head;
    
    /* 提取最近的 6 条记录 (最老为 rec[0]，最新为 rec[show_cnt-1]) */
    show_cnt = temp_log_count > 6 ? 6 : temp_log_count;
    start_i = temp_log_count > 6 ? (temp_log_count - 6) : 0;
    for (i = 0; i < (int)show_cnt; i++)
    {
        log_idx = (temp_log_head + start_i + i) % 20;
        strcpy(temp_logs[i], Comm_Logs[log_idx]);
    }
    temp_log_count = show_cnt; /* 更新为实际显示的条数 */
    OS_EXIT_CRITICAL();

    /* 依次在屏幕上绘制这些日志 (从上往下，最老在最上面，最新在最下面) */
    draw_y = box_y + 45;
    POINT_COLOR = GREEN;
    BACK_COLOR = BLACK;
    for (i = 0; i < (int)temp_log_count; i++)
    {
        char *p_log = temp_logs[i];
        int len = (int)strlen(p_log);
        char line1[55];
        char line2[55];
        
        if (len <= 48)
        {
            strcpy(line1, p_log);
            line2[0] = '\0';
        }
        else
        {
            strncpy(line1, p_log, 48);
            line1[48] = '\0';
            strncpy(line2, p_log + 48, 48);
            line2[48] = '\0';
        }
        
        /* 绘制第一行 */
        sprintf(str_buf, "%-50s", line1);
        LCD_ShowString(box_x + 15, draw_y, box_w - 30, 16, 16, (u8 *)str_buf);
        draw_y += 20;
        
        /* 绘制第二行 (或者擦除第二行区域) */
        if (line2[0] != '\0')
        {
            sprintf(str_buf, "%-50s", line2);
            LCD_ShowString(box_x + 15, draw_y, box_w - 30, 16, 16, (u8 *)str_buf);
            draw_y += 20;
        }
        else
        {
            /* 擦除原本可能属于第二行的区域 */
            LCD_Fill(box_x + 1, draw_y, box_x + box_w - 2, draw_y + 19, BLACK);
            draw_y += 20;
        }
        
        /* 擦除两条记录之间的间隔带，并增加 Y 轴偏量 */
        LCD_Fill(box_x + 1, draw_y, box_x + box_w - 2, draw_y + 29, BLACK);
        draw_y += 30;
    }
    
    /* 擦除下方剩余空白部分 */
    if (draw_y < box_y + box_h - 1)
    {
        LCD_Fill(box_x + 1, draw_y, box_x + box_w - 2, box_y + box_h - 2, BLACK);
    }
}

/**
 * @brief  绘制屏幕底部状态栏
 * @note   在屏幕底部区域显示系统时间、温度与光照实测值与限值，以及 LED1、蜂鸣器消音状态和当前 UI 模式名。
 * @param  data: 指向当前传感器数据结构体的指针
 * @param  limit: 温度限制值
 * @param  led_st: LED1 状态 (0/1)
 * @param  mute: 蜂鸣器消音状态 (0/1)
 * @retval 无
 */
static void ui_draw_bottom_bar(SensorData_t *data, int limit, u8 led_st, u8 mute)
{
    int text_y;
    int temp_val;
    int temp_abs;
    int limit_abs;

    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    text_y = lcddev.height - 90;
    if (text_y < 200) text_y = 200;

    sprintf(str_buf, "Time: %-10s", data->time);
    LCD_ShowString(20, text_y, 200, 16, 16, (u8 *)str_buf);

    temp_val = data->temperature;
    temp_abs = temp_val < 0 ? -temp_val : temp_val;
    limit_abs = limit < 0 ? -limit : limit;
    sprintf(str_buf, "Temp: %s%d.%dC Limit: %s%d.%dC   ",
            temp_val < 0 ? "-" : "", temp_abs / 10, temp_abs % 10,
            limit < 0 ? "-" : "", limit_abs / 10, limit_abs % 10);
    LCD_ShowString(20, text_y + 20, 320, 16, 16, (u8 *)str_buf);

    sprintf(str_buf, "Light: %-3d Limit: %-3d   ", data->light, GlobalConfig.light_limit);
    LCD_ShowString(20, text_y + 40, 320, 16, 16, (u8 *)str_buf);

    sprintf(str_buf, "LED1: %-3s Mute: %-3s   ", led_st ? "ON" : "OFF", mute ? "YES" : "NO");
    LCD_ShowString(20, text_y + 60, 240, 16, 16, (u8 *)str_buf);

    if (UI_Mode == 0)      LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: CURVE VIEW ");
    else if (UI_Mode == 1) LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: DASHBOARD  ");
    else if (UI_Mode == 2) LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: COMM LOGS  ");
}

/**
 * @brief  绘制系统设置主菜单与调节竖条
 * @note   在屏幕居中位置绘制系统设置主菜单背景、选项列表、选中的光标指示器，并在二级数值编辑模式下，于右侧绘制带颜色填充（温度为红、光照为黄）的比例调节竖条。
 * @param  menu_s: 菜单状态 (1=导航模式, 2=编辑模式)
 * @param  menu_c: 当前菜单光标索引 (0~5)
 * @param  limit: 当前温度限制值
 * @param  is_emergency: 是否处于紧急拉闸状态
 * @param  mute: 蜂鸣器是否消音
 * @param  menu_bg_draw: 是否重绘菜单框体灰色背景及顶栏标题
 * @retval 无
 */
static void ui_draw_menu(u8 menu_s, u8 menu_c, int limit, u8 is_emergency, u8 mute, u8 menu_bg_draw)
{
    u16 menu_w;
    u16 menu_h;
    u16 menu_x;
    u16 menu_y;
    u16 menu_i;
    u16 item_y;
    int limit_abs;

    menu_w = 320;
    menu_h = 250;
    menu_x = (lcddev.width - menu_w) / 2;
    menu_y = (lcddev.height - menu_h) / 2;
    
    if (menu_bg_draw)
    {
        LCD_Fill(menu_x, menu_y, menu_x + menu_w - 1, menu_y + menu_h - 1, GRAY);
        POINT_COLOR = WHITE;
        LCD_DrawLine(menu_x, menu_y, menu_x + menu_w, menu_y);
        LCD_DrawLine(menu_x, menu_y + menu_h, menu_x + menu_w, menu_y + menu_h);
        LCD_DrawLine(menu_x, menu_y, menu_x, menu_y + menu_h);
        LCD_DrawLine(menu_x + menu_w, menu_y, menu_x + menu_w, menu_y + menu_h);
        
        BACK_COLOR = GRAY;
        LCD_ShowString(menu_x + 90, menu_y + 15, 140, 24, 24, (u8*)"SYSTEM MENU");
        LCD_DrawLine(menu_x + 10, menu_y + 45, menu_x + menu_w - 10, menu_y + 45);
    }
    
    /* 清空之前的进度条痕迹，防止返回导航时花屏 */
    if (menu_s == 1)
    {
        LCD_Fill(menu_x + 250, menu_y + 60, menu_x + 300, menu_y + 220, GRAY);
    }
    
    BACK_COLOR = GRAY;
    for (menu_i = 0; menu_i < 6; menu_i++)
    {
        item_y = menu_y + 55 + menu_i * 28;
        if (menu_i == menu_c) 
        {
            POINT_COLOR = YELLOW;
            /* 层级不同使用不同前缀：二级修改为 '>' 提示，一级菜单为 '*' 提示 */
            if (menu_s == 2) LCD_ShowString(menu_x + 15, item_y, 20, 16, 16, (u8*)">");
            else             LCD_ShowString(menu_x + 15, item_y, 20, 16, 16, (u8*)"*");
        }
        else
        {
            POINT_COLOR = WHITE;
            LCD_ShowString(menu_x + 15, item_y, 20, 16, 16, (u8*)" ");
        }
        
        if (menu_i == 0) 
        {
            limit_abs = limit < 0 ? -limit : limit;
            if (menu_s == 2 && menu_c == 0)
                sprintf(str_buf, "1. Temp Limit  : [%s%d.%dC]", limit < 0 ? "-" : "", limit_abs / 10, limit_abs % 10);
            else
                sprintf(str_buf, "1. Temp Limit  :  %s%d.%dC", limit < 0 ? "-" : "", limit_abs / 10, limit_abs % 10);
        }
        else if (menu_i == 1) 
        {
            if (menu_s == 2 && menu_c == 1)
                sprintf(str_buf, "2. Light Limit : [%-3d]", GlobalConfig.light_limit);
            else
                sprintf(str_buf, "2. Light Limit :  %-3d", GlobalConfig.light_limit);
        }
        else if (menu_i == 2) sprintf(str_buf, "3. Emg Stop    : %-3s", is_emergency?"ON ":"OFF");
        else if (menu_i == 3) sprintf(str_buf, "4. Beep Mute   : %-3s", mute?"ON ":"OFF");
        else if (menu_i == 4) 
        {
            if (menu_s == 2 && menu_c == 4)
                sprintf(str_buf, "5. Crash Sim   : [YES]");
            else
                sprintf(str_buf, "5. Crash Sim   :  NO ");
        }
        else if (menu_i == 5) sprintf(str_buf, "6. Exit");
        
        LCD_ShowString(menu_x + 30, item_y, 220, 16, 16, (u8*)str_buf);
    }

    /* --- 绘制右侧温度/光照调节的“竖条条” --- */
    if (menu_s == 2 && (menu_c == 0 || menu_c == 1))
    {
        u16 bar_x = menu_x + 270;
        u16 bar_y = menu_y + 70;
        u16 bar_w = 16;
        u16 bar_h = 130;
        u16 fill_h = 0;
        
        /* 绘制外边框 */
        POINT_COLOR = WHITE;
        LCD_DrawLine(bar_x, bar_y, bar_x + bar_w, bar_y);
        LCD_DrawLine(bar_x, bar_y + bar_h, bar_x + bar_w, bar_y + bar_h);
        LCD_DrawLine(bar_x, bar_y, bar_x, bar_y + bar_h);
        LCD_DrawLine(bar_x + bar_w, bar_y, bar_x + bar_w, bar_y + bar_h);
        
        if (menu_c == 0) /* 温度范围为 15.0 ~ 50.0 C (150 ~ 500) */
        {
            if (limit < 150) fill_h = 0;
            else if (limit > 500) fill_h = bar_h - 2;
            else fill_h = (u16)((limit - 150) * (bar_h - 2) / 350);
            
            /* 红色填充代表温度 */
            if (fill_h > 0) LCD_Fill(bar_x + 1, bar_y + bar_h - fill_h - 1, bar_x + bar_w - 1, bar_y + bar_h - fill_h - 2 + fill_h + 1, RED);
            if (fill_h < (bar_h - 2)) LCD_Fill(bar_x + 1, bar_y + 1, bar_x + bar_w - 1, bar_y + bar_h - fill_h - 2, DARKBLUE);
        }
        else /* 光照范围为 10 ~ 95 */
        {
            if (GlobalConfig.light_limit < 10) fill_h = 0;
            else if (GlobalConfig.light_limit > 95) fill_h = bar_h - 2;
            else fill_h = (u16)((GlobalConfig.light_limit - 10) * (bar_h - 2) / 85);
            
            /* 黄色填充代表光照 */
            if (fill_h > 0) LCD_Fill(bar_x + 1, bar_y + bar_h - fill_h - 1, bar_x + bar_w - 1, bar_y + bar_h - fill_h - 2 + fill_h + 1, YELLOW);
            if (fill_h < (bar_h - 2)) LCD_Fill(bar_x + 1, bar_y + 1, bar_x + bar_w - 1, bar_y + bar_h - fill_h - 2, DARKBLUE);
        }
    }
}

/* ================== 系统核心任务函数 ================== */

/**
 * @brief  系统主函数入口
 */
int main(void)
{
    u8 err;

    /* ====== 显式开启 FPU (防止编译器未定义 __FPU_USED 导致浮点运算 HardFault) ====== */
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); /* 设中断优先级分组2 */
    delay_init(168);                                /* 初始化延时函数 */
    uart_init(115200);                              /* 初始化串口1 */
    LED_Init();                                     /* 初始化 LED 及蜂鸣器 */
    LED1 = 0;                                       /* 上电默认点亮LED1(g_Led1_Cmd=1) */
    LCD_Init();                                     /* 初始化 LCD */

    /* ====== 独立看门狗复位检测与声光报警提示 ====== */
    check_watchdog_reset();

    Adc_Init();                                     /* 初始化 ADC */
    Dac1_Init();                                    /* 初始化 DAC 通道1 (PA4) */
    EXTIX_Init();                                   /* 初始化按键 EXTI 中断 */
    FSMC_SRAM_Init();                               /* 初始化外部 SRAM */

    /* 固化参数参数载入 */
    load_system_config();

    OSInit();

    UI_Queue = OSQCreate(UI_Msg_Buf, QUEUE_MAX_SIZE);
    COM_Queue = OSQCreate(COM_Msg_Buf, QUEUE_MAX_SIZE);
    Key_Sem = OSSemCreate(0);
    Config_Mutex = OSMutexCreate(0, &err); /* 无天花板优先级,依赖优先级继承自动处理 */

    OSTaskCreate(start_task, (void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE - 2], START_TASK_PRIO);
    OSStart();
    return 0;
}

/**
 * @brief  系统启动任务，用于创建其他核心应用任务
 */
void start_task(void *pdata)
{
    pdata = pdata;

    OSTaskCreate(control_task, (void *)0, (OS_STK *)&CONTROL_TASK_STK[CONTROL_STK_SIZE - 2], CONTROL_TASK_PRIO);
    OSTaskCreate(sensor_task, (void *)0, (OS_STK *)&SENSOR_TASK_STK[SENSOR_STK_SIZE - 2], SENSOR_TASK_PRIO);
    OSTaskCreate(com_task, (void *)0, (OS_STK *)&COM_TASK_STK[COM_STK_SIZE - 2], COM_TASK_PRIO);
    OSTaskCreate(ui_task, (void *)0, (OS_STK *)&UI_TASK_STK[UI_STK_SIZE - 2], UI_TASK_PRIO);

    /* 防止 UCOS-II 早期 PendSV 抢占导致栈溢出,先延时让调度稳定 */
    OSTimeDly(2);

    /* 挂起自身(start_task不再需要) */
    OSTaskSuspend(OS_PRIO_SELF);
    
    /* 兜底:挂起失败时死循环防止PC跑飞 */
    while (1)
        ;
}

/**
 * @brief  传感器采集与声光警报任务
 */
void sensor_task(void *pdata)
{
    int temp_window[10];
    uint16_t light_window[10];
    u8 window_idx;
    u8 window_full;
    u8 cycle_cnt;
    int temp_sum;
    u32 light_sum;
    u8 sample_num;
    int avg_temp_adc;
    uint16_t avg_light;
    int cur_temp_10;
    u8 err;
    u8 is_emergency;
    int limit;
    uint16_t light_limit;
    SensorData_t *p_data;
    int i;
    int cur_light;
    u16 adc_val;
    uint16_t dac_vol_mv;
    uint16_t adc_vol_mv;
    u8 led_cmd;

    /* 静态变量 */
    static u8 dac_idx = 0;    /* 静态变量，记录 DAC 查表索引 */
    static u8 alarm_type = 0; /* 报警状态: 0=正常, 1=普通超限, 2=紧急拉闸 */
    
    /* 正弦查表数据，表示 0.9V ~ 2.9V，避开极限电压，波形平滑 */
    const uint16_t Dac_Sin_Table[16] = {1900, 2280, 2600, 2800, 2900, 2800, 2600, 2280, 1900, 1520, 1200, 1000, 900, 1000, 1200, 1520};

    static SensorData_t data_pool[QUEUE_MAX_SIZE + 1];
    static u8 pool_idx = 0;

    memset(temp_window, 0, sizeof(temp_window));
    memset(light_window, 0, sizeof(light_window));
    window_idx = 0;
    window_full = 0;
    cycle_cnt = 0;

    pdata = pdata;

    printf("[SENSOR] Task started\r\n");

    /* ====== 开启独立看门狗 (IWDG) 监控系统 ====== */
    iwdg_init();

    while (1)
    {
        IWDG->KR = 0xAAAA; /* 喂狗 */

        /* 50ms 高频周期性采集 (读取原始 ADC 整数值) */
        temp_window[window_idx] = Get_Adc_Value(ADC1, ADC_Channel_TempSensor);
        light_window[window_idx] = Get_Adc_Value(ADC3, ADC_Channel_5);
        window_idx++;
        if (window_idx >= 10)
        {
            window_idx = 0;
            window_full = 1;
        }

        cycle_cnt++;
        if (cycle_cnt >= 5) /* 250ms (4Hz) 处理与上报 */
        {
            cycle_cnt = 0;

            temp_sum = 0;
            light_sum = 0;
            sample_num = window_full ? 10 : window_idx;
            if (sample_num == 0)
                sample_num = 1;

            for (i = 0; i < sample_num; i++)
            {
                temp_sum += temp_window[i];
                light_sum += light_window[i];
            }
            avg_temp_adc = temp_sum / sample_num;
            avg_light = light_sum / sample_num;

            /* 内部传感器纯整数乘移位的高精度公式，放大 10 倍，不需要任何 FPU 和浮点资源 */
            cur_temp_10 = ((avg_temp_adc * 825) >> 8) - 2790;
            if (avg_light > 4000) avg_light = 4000;
            cur_light = 100 - (avg_light / 40);

            OSMutexPend(Config_Mutex, 0, &err);
            is_emergency = GlobalConfig.emergency_stop;
            limit = GlobalConfig.temp_limit;
            light_limit = GlobalConfig.light_limit;
            
            led_cmd = g_Led1_Cmd;
            if (led_cmd == 1)
                LED1 = 0; /* 低电平亮 */
            else
                LED1 = 1; /* 高电平灭 */
                
            if (!is_emergency && cur_temp_10 <= limit && cur_light <= light_limit)
            {
                GlobalConfig.beep_mute = 0;
            }
            OSMutexPost(Config_Mutex);

            /* 更新报警状态，由 50ms 状态机在小周期内执行声光控制 */
            if (is_emergency)
            {
                alarm_type = 2;
            }
            else if (cur_temp_10 > limit || cur_light > light_limit)
            {
                alarm_type = 1;
            }
            else
            {
                alarm_type = 0;
            }

            /* 查正弦表输出 1Hz 动态电压信号 */
            dac_vol_mv = Dac_Sin_Table[dac_idx];
            Dac1_Set_Vol(dac_vol_mv);
            dac_idx = (dac_idx + 1) % 16;

            /* 回读 PA5 电压 */
            adc_val = Get_Adc_Value(ADC1, ADC_Channel_5);
            adc_vol_mv = (uint16_t)((u32)adc_val * 3300 / 4096);

            p_data = &data_pool[pool_idx];
            p_data->temperature = cur_temp_10;
            p_data->light = cur_light;
            p_data->dac_vol = dac_vol_mv;
            p_data->adc_vol = adc_vol_mv;
            RTC_Get_Time_Str((char *)p_data->time);

            OSQPost(UI_Queue, (void *)p_data);
            OSQPost(COM_Queue, (void *)p_data);

            pool_idx = (pool_idx + 1) % (QUEUE_MAX_SIZE + 1);
        }

        /* ====== 50ms 级别声光报警状态机 ====== */
        sensor_alarm_control(alarm_type);

        delay_ms(50);
    }
}

/**
 * @brief  系统控制逻辑与按键交互任务
 */
void control_task(void *pdata)
{
    u8 err;
    pdata = pdata;
    while (1)
    {
        if (Menu_State != 2) /* 在 待机(0) 和 导航(1) 状态下，使用信号量阻塞等待外部中断（按一下响应一次，省电防连跳） */
        {
            OSSemPend(Key_Sem, 0, &err); /* 等待按键中断同步信号量 */
            
            /* 立即硬件级屏蔽这四个按键 of EXTI 中断触发，以供软件防抖 */
            EXTI->IMR &= ~((1 << 0) | (1 << 2) | (1 << 3) | (1 << 4));

            OSMutexPend(Config_Mutex, 0, &err);

            if (Menu_State == 0) /* 待机模式 */
            {
                control_handle_standby();
            }
            else if (Menu_State == 1) /* 菜单导航模式 (共 6 项) */
            {
                control_handle_menu_nav();
            }

            Last_Key = 0xFF;
            OSMutexPost(Config_Mutex);

            /* 主动发送瞬时刷新指令，让屏幕在微秒内完成状态重绘 */
            OSQPost(UI_Queue, (void *)0xFFFFFFFF);

            /* 挂起任务 100ms 去抖 */
            OSTimeDly(10); 

            /* 等待物理按键全部松开，从根本上杜绝连按或连跳问题 */
            while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0 ||
                   GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0)
            {
                OSTimeDly(2); /* 每次延时 20ms */
            }

            /* 松手后再防抖 50ms */
            OSTimeDly(5);

            /* 清除此期间产生的 EXTI 挂起寄存器标志 */
            EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4);
            
            /* 只有当非编辑模式时，才重新使能外部中断 */
            if (Menu_State != 2)
            {
                EXTI->IMR |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4);
            }
            
            /* 清理队列多余信号量 */
            while (OSSemAccept(Key_Sem) > 0);
        }
        else /* 在 编辑(2) 状态下，为了实现长按快速调节，我们直接高频周期性轮询物理引脚电平！ */
        {
            control_handle_menu_edit();
        }
    }
}

/**
 * @brief  串口双向数据通信与命令解析任务
 */
void com_task(void *pdata)
{
    u8 err;
    SensorData_t *data;
    char log_buf[80];
    char json_str[160];
    int limit;
    char *p;
    char *p_led;
    char *p_val;
    int val_led;
    char *p_alert;
    int val_alert;
    int temp_i, temp_d;
    int temp_val, temp_abs;
    int limit_abs;
    int light_limit;
    u8 is_emergency;
    int light_val;
    const char *warn_str;

    pdata = pdata;

    while (1)
    {
        /* 限时 100ms 等待发送队列，以保证有时间处理接收 */
        data = (SensorData_t *)OSQPend(COM_Queue, 10, &err);
        if (err == OS_ERR_NONE && data != NULL)
        {
            OSMutexPend(Config_Mutex, 0, &err);
            limit = GlobalConfig.temp_limit;
            light_limit = GlobalConfig.light_limit;
            is_emergency = GlobalConfig.emergency_stop;
            OSMutexPost(Config_Mutex);

            temp_val = data->temperature;
            temp_abs = temp_val < 0 ? -temp_val : temp_val;
            limit_abs = limit < 0 ? -limit : limit;
            light_val = data->light;

            warn_str = "NONE";
            if (is_emergency)
            {
                warn_str = "EMERGENCY";
            }
            else
            {
                if (temp_val > limit && light_val > light_limit)
                {
                    warn_str = "TEMP_AND_LIGHT_OVER_LIMIT";
                }
                else if (temp_val > limit)
                {
                    warn_str = "TEMP_OVER_LIMIT";
                }
                else if (light_val > light_limit)
                {
                    warn_str = "LIGHT_OVER_LIMIT";
                }
            }

            sprintf(json_str, "{\"time\":\"%s\",\"temp\":%s%d.%d,\"light\":%d,\"alert_t\":%s%d.%d,\"alert_l\":%d,\"dac_v\":%d.%02d,\"adc_v\":%d.%02d,\"warn\":\"%s\"}\r\n",
                    data->time,
                    temp_val < 0 ? "-" : "", temp_abs / 10, temp_abs % 10,
                    light_val,
                    limit < 0 ? "-" : "", limit_abs / 10, limit_abs % 10,
                    light_limit,
                    data->dac_vol / 1000, (data->dac_vol % 1000) / 10,
                    data->adc_vol / 1000, (data->adc_vol % 1000) / 10,
                    warn_str);

            p = json_str;
            while (*p)
            {
                /* 等待发送完毕 */
                while ((USART1->SR & 0X40) == 0)
                    ;
                USART1->DR = (u8)*p++;
            }
            /* 构造 TX 日志信息，由于 json_str 包含回车换行，截断并去除尾部回车 */
            strncpy(log_buf, "TX: ", 4);
            strncpy(log_buf + 4, json_str, 75);
            log_buf[79] = '\0';
            {
                char *p_nl = strpbrk(log_buf, "\r\n");
                if (p_nl != NULL) *p_nl = '\0';
            }
            Add_Comm_Log(log_buf);
        }

        /* 轮询串口 DMA 接收完成标志 */
        if (USART_RX_FLAG == 1)
        {
            /* 构造 RX 日志信息，把真实接收的指令字符串打印出来 */
            strncpy(log_buf, "RX: ", 4);
            strncpy(log_buf + 4, (char *)USART_RX_BUF, 75);
            log_buf[79] = '\0';
            {
                char *p_nl = strpbrk(log_buf, "\r\n");
                if (p_nl != NULL) *p_nl = '\0';
            }
            Add_Comm_Log(log_buf);

            p_led = strstr((char *)USART_RX_BUF, "\"LED\"");
            if (p_led != NULL)
            {
                p_val = strchr(p_led, ':');
                if (p_val != NULL)
                {
                    val_led = 0;
                    sscanf(p_val + 1, "%d", &val_led);
                    OSMutexPend(Config_Mutex, 0, &err);
                    GlobalConfig.led_state = val_led;
                    g_Led1_Cmd = val_led;
                    /* 立即生效 GPIO,无需等 sensor_task 的 1Hz 周期 */
                    if (val_led == 1)
                        LED1 = 0;
                    else
                        LED1 = 1;
                    OSMutexPost(Config_Mutex);
                }
            }

            p_alert = strstr((char *)USART_RX_BUF, "\"SET_ALERT\"");
            if (p_alert != NULL)
            {
                p_val = strchr(p_alert, ':');
                if (p_val != NULL)
                {
                    val_alert = 0;
                    temp_i = 0;
                    temp_d = 0;
                    if (sscanf(p_val + 1, "%d.%d", &temp_i, &temp_d) == 2)
                    {
                        val_alert = temp_i * 10 + temp_d;
                    }
                    else if (sscanf(p_val + 1, "%d", &temp_i) == 1)
                    {
                        val_alert = temp_i * 10;
                    }

                    if (val_alert >= 150 && val_alert <= 500)
                    {
                        OSMutexPend(Config_Mutex, 0, &err);
                        GlobalConfig.temp_limit = val_alert;
                        RTC_BKP_Write((0x55AA << 16) | (uint16_t)val_alert);
                        OSMutexPost(Config_Mutex);
                    }
                }
            }

            USART_RX_FLAG = 0;
            USART_RX_LEN = 0;
        }
    }
}

/**
 * @brief  LCD 液晶屏界面刷新与显示绘制任务
 */
void ui_task(void *pdata)
{
    u8 err;
    SensorData_t *data;

    /* 声明所有局部变量于函数顶部，完美适配 C89 编译器 */
    int limit;
    u8 is_emergency;
    u8 led_st;
    u8 menu_s;
    u8 menu_c;
    u8 mute;
    u8 tail;
    u8 last_mode;
    u8 last_emg;
    u8 last_menu_s;
    u8 last_menu_c;
    u8 force_clear;
    u8 menu_bg_draw;

    static SensorData_t last_data;
    static u8 has_last_data = 0;

    last_mode = 0xFF;
    last_emg = 0xFF;
    last_menu_s = 0xFF;
    last_menu_c = 0xFF;
    force_clear = 0;
    menu_bg_draw = 0;

    pdata = pdata;

    // ========== 开机欢迎画面 ==========
    /* 在最开始在 LCD 上画欢迎画面，差分刷新无 SRAM 显存占用 */
    LCD_Fill(0, 0, lcddev.width - 1, lcddev.height - 1, BLACK);
    POINT_COLOR = BLUE;
    LCD_DrawLine(20, 20, lcddev.width - 20, 20);
    LCD_DrawLine(20, lcddev.height - 20, lcddev.width - 20, lcddev.height - 20);
    LCD_DrawLine(20, 20, 20, lcddev.height - 20);
    LCD_DrawLine(lcddev.width - 20, 20, lcddev.width - 20, lcddev.height - 20);

    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2 - 40, 200, 24, 24, (u8 *)"Gateway System");
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2, 200, 16, 16, (u8 *)"STM32F407 + UCOS-II");
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2 + 30, 200, 16, 16, (u8 *)"Initializing...");
    delay_ms(500);
    // ========== 欢迎画面结束 ==========

    while (1)
    {
        data = (SensorData_t *)OSQPend(UI_Queue, 0, &err);
        if (err == OS_ERR_NONE && data != NULL)
        {
            if (data == (SensorData_t *)0xFFFFFFFF)
            {
                if (!has_last_data) continue;
                data = &last_data;
            }
            else
            {
                last_data = *data;
                has_last_data = 1;

                /* 只有收到真正的传感器 data 才更新历史曲线 */
                tail = (History_Head + History_Count) % PLOT_POINTS_MAX;
                if (History_Count < PLOT_POINTS_MAX)
                {
                    History_Count++;
                }
                else
                {
                    History_Head = (History_Head + 1) % PLOT_POINTS_MAX;
                }
                Temp_History[tail] = data->temperature;
                Light_History[tail] = data->light;
            }

            OSMutexPend(Config_Mutex, 0, &err);
            limit = GlobalConfig.temp_limit;
            is_emergency = GlobalConfig.emergency_stop;
            led_st = GlobalConfig.led_state;
            mute = GlobalConfig.beep_mute;
            menu_s = Menu_State;
            menu_c = Menu_Cursor;
            OSMutexPost(Config_Mutex);

            force_clear = 0;
            menu_bg_draw = 0;
            if (is_emergency != last_emg || UI_Mode != last_mode || menu_s != last_menu_s)
            {
                force_clear = 1;
                last_emg = is_emergency;
                last_mode = UI_Mode;
                last_menu_s = menu_s;
                
                LCD_Fill(0, 0, lcddev.width - 1, lcddev.height - 1, is_emergency ? RED : BLACK);
                if (menu_s > 0) menu_bg_draw = 1;
            }

            if (menu_s > 0 && menu_c != last_menu_c)
            {
                last_menu_c = menu_c;
            }

            if (is_emergency)
            {
                if (force_clear) {
                    POINT_COLOR = WHITE;
                    BACK_COLOR = RED;
                    LCD_ShowString((lcddev.width - 360) / 2, 80, 360, 24, 24, (u8 *)"SYSTEM EMERGENCY STOP!");
                    LCD_ShowString((lcddev.width - 320) / 2, 140, 320, 16, 16, (u8 *)"Press KEY2 to recover system");
                }
            }
            else
            {
                if (menu_s == 0 || force_clear)
                {
                    if (UI_Mode == 0) /* 折线图看板 */
                    {
                        ui_draw_curve(menu_s, limit);
                    }
                    else if (UI_Mode == 1) /* 仪表盘看板 */
                    {
                        ui_draw_dashboard(data, limit);
                    }
                    else if (UI_Mode == 2) /* 日志看板 */
                    {
                        ui_draw_terminal_logs(force_clear);
                    }
                }

                /* BOTTOM TEXT (总是绘制，背景覆盖实现差分刷新) */
                ui_draw_bottom_bar(data, limit, led_st, mute);

                /* --- 绘制居中菜单 --- */
                if (menu_s > 0)
                {
                    ui_draw_menu(menu_s, menu_c, limit, is_emergency, mute, menu_bg_draw);
                }
            }
        }
    }
}
