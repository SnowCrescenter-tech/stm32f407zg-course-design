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

// 外部 SRAM 双缓冲显存定义 (基地址 0x68000000)
#define SRAM_BANK3_ADDR ((u32)0x68000000)
u16 *UI_Buffer = (u16 *)SRAM_BANK3_ADDR;

#define UI_W 480
// UI_H 改为 800，适配 NT35510 竖屏（480x800），同时也自动适配其他屏幕
// 原值 272 导致只刷新顶部 1/3，底部文字画在白底上不可见
#define UI_H 800

// 画图底层 API
void UI_Clear(u16 color)
{
    u32 total = (u32)lcddev.width * lcddev.height;
    u32 i;
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
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
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

// 任务堆栈大小和任务定义
#define START_TASK_PRIO 10
#define START_STK_SIZE 128
OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);

#define CONTROL_TASK_PRIO 4
#define CONTROL_STK_SIZE 256
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
void control_task(void *pdata);

#define SENSOR_TASK_PRIO 5
#define SENSOR_STK_SIZE 512 // 原256不足,float+FPU需要更大栈空间
OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
void sensor_task(void *pdata);

#define COM_TASK_PRIO 6
#define COM_STK_SIZE 512
OS_STK COM_TASK_STK[COM_STK_SIZE];
void com_task(void *pdata);

#define UI_TASK_PRIO 7
#define UI_STK_SIZE 768 // 原512,大屏(480x800)绘图需要更大栈
OS_STK UI_TASK_STK[UI_STK_SIZE];
void ui_task(void *pdata);

// 内核对象指针
OS_EVENT *UI_Queue;
OS_EVENT *COM_Queue;
OS_EVENT *Key_Sem;
OS_EVENT *Config_Mutex;

#define QUEUE_MAX_SIZE 5
void *UI_Msg_Buf[QUEUE_MAX_SIZE];
void *COM_Msg_Buf[QUEUE_MAX_SIZE];

// 全局状态变量
u8 UI_Mode = 0;    // 0: 折线图, 1: 仪表盘, 2: 原始日志
u8 g_Led1_Cmd = 1; // LED1 控制字: 0=串口命令关闭, 1=串口命令打开, 默认开

// 传感器数据传输结构体
typedef struct
{
    float temperature;
    uint16_t light;
    uint8_t time[10];
} SensorData_t;

// 系统配置参数结构体
typedef struct
{
    float temp_limit;
    uint16_t light_limit;
    u8 emergency_stop;
    u8 beep_mute;
    u8 led_state;
} GlobalConfig_t;

GlobalConfig_t GlobalConfig;

// 曲线历史缓冲区
#define PLOT_POINTS_MAX 40
float Temp_History[PLOT_POINTS_MAX];
uint16_t Light_History[PLOT_POINTS_MAX];
u8 History_Count = 0;
u8 History_Head = 0;

// 通信日志缓冲区
#define LOG_LINES_MAX 5
char Comm_Logs[LOG_LINES_MAX][80];
u8 Log_Count = 0;
u8 Log_Head = 0;

void Add_Comm_Log(const char *log_str)
{
    OS_CPU_SR cpu_sr = 0;
    u8 tail;

    OS_ENTER_CRITICAL(); // 进入临界区

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

    OS_EXIT_CRITICAL(); // 退出临界区
}

int main(void)
{
    u32 bkp_val;
    u8 err;

    // ====== 显式开启 FPU (防止编译器未定义 __FPU_USED 导致浮点运算 HardFault) ======
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设中断优先级分组2
    delay_init(168);                                // 初始化延时函数
    uart_init(115200);                              // 初始化串口1
    LED_Init();                                     // 初始化 LED 及蜂鸣器
    LED1 = 0;                                       // 上电默认点亮LED1(g_Led1_Cmd=1)
    LCD_Init();                                     // 初始化 LCD
    Adc_Init();                                     // 初始化 ADC
    EXTIX_Init();                                   // 初始化按键 EXTI 中断
    FSMC_SRAM_Init();                               // 初始化外部 SRAM

    // 固化参数参数载入
    if (My_RTC_Init() == 0)
    {
        bkp_val = RTC_BKP_Read();
        if ((bkp_val >> 16) == 0x55AA)
        {
            GlobalConfig.temp_limit = (float)(bkp_val & 0xFFFF) / 100.0f;
        }
        else
        {
            GlobalConfig.temp_limit = 38.0f;
            RTC_BKP_Write((0x55AA << 16) | (uint16_t)(38.0f * 100));
        }
    }
    else
    {
        GlobalConfig.temp_limit = 38.0f;
    }
    GlobalConfig.light_limit = 4000; // 默认光照报警值
    GlobalConfig.emergency_stop = 0;
    GlobalConfig.beep_mute = 0;
    GlobalConfig.led_state = 0;

    OSInit();

    UI_Queue = OSQCreate(UI_Msg_Buf, QUEUE_MAX_SIZE);
    COM_Queue = OSQCreate(COM_Msg_Buf, QUEUE_MAX_SIZE);
    Key_Sem = OSSemCreate(0);
    Config_Mutex = OSMutexCreate(0, &err); // 无天花板优先级,依赖优先级继承自动处理

    OSTaskCreate(start_task, (void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO);
    OSStart();
    return 0;
}

void start_task(void *pdata)
{
    pdata = pdata;

    OSTaskCreate(control_task, (void *)0, (OS_STK *)&CONTROL_TASK_STK[CONTROL_STK_SIZE - 1], CONTROL_TASK_PRIO);
    OSTaskCreate(sensor_task, (void *)0, (OS_STK *)&SENSOR_TASK_STK[SENSOR_STK_SIZE - 1], SENSOR_TASK_PRIO);
    OSTaskCreate(com_task, (void *)0, (OS_STK *)&COM_TASK_STK[COM_STK_SIZE - 1], COM_TASK_PRIO);
    OSTaskCreate(ui_task, (void *)0, (OS_STK *)&UI_TASK_STK[UI_STK_SIZE - 1], UI_TASK_PRIO);

    // 防止 UCOS-II 早期 PendSV 抢占导致栈溢出,先延时让调度稳定
    OSTimeDly(2);

    // 挂起自身(start_task不再需要)
    OSTaskSuspend(OS_PRIO_SELF);
    // 兜底:挂起失败时死循环防止PC跑飞
    while (1)
        ;
}

void sensor_task(void *pdata)
{
    float temp_window[10] = {0};
    uint16_t light_window[10] = {0};
    u8 window_idx = 0;
    u8 window_full = 0;
    u8 cycle_cnt = 0;

#define DATA_POOL_SIZE (QUEUE_MAX_SIZE + 1)
    static SensorData_t data_pool[DATA_POOL_SIZE];
    static u8 pool_idx = 0;

    // 变量声明移至最顶部以支持 C89 标准
    float temp_sum;
    u32 light_sum;
    u8 sample_num;
    float avg_temp_adc;
    uint16_t avg_light;
    float v_sense;
    float cur_temp;
    u8 err;
    u8 is_emergency;
    u8 mute;
    float limit;
    uint16_t light_limit;
    SensorData_t *p_data;
    int i;

    pdata = pdata;

    printf("[SENSOR] Task started\r\n");

    while (1)
    {
        // 50ms 高频周期性采集
        temp_window[window_idx] = Get_Adc_Value(ADC1, ADC_Channel_TempSensor);
        light_window[window_idx] = Get_Adc_Value(ADC3, ADC_Channel_5);
        window_idx++;
        if (window_idx >= 10)
        {
            window_idx = 0;
            window_full = 1;
        }

        cycle_cnt++;
        if (cycle_cnt >= 20) // 1Hz 处理与上报
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
            avg_temp_adc = temp_sum / (float)sample_num;
            avg_light = light_sum / sample_num;

            // 内部传感器转换公式
            v_sense = avg_temp_adc * (3.3f / 4096.0f);
            cur_temp = (v_sense - 0.76f) / 0.0025f + 25.0f;

            OSMutexPend(Config_Mutex, 0, &err);
            is_emergency = GlobalConfig.emergency_stop;
            mute = GlobalConfig.beep_mute;
            limit = GlobalConfig.temp_limit;
            light_limit = GlobalConfig.light_limit;
            {
                u8 led_cmd = g_Led1_Cmd;
                if (led_cmd == 1)
                    LED1 = 0; // 低电平亮
                else
                    LED1 = 1; // 高电平灭
            }
            OSMutexPost(Config_Mutex);

            if (is_emergency)
            {
                BEEP = 1;
                LED0 = !LED0; // 红灯爆闪
            }
            else
            {
                if (cur_temp > limit || avg_light > light_limit)
                {
                    if (!mute)
                    {
                        BEEP = !BEEP;
                    }
                    else
                    {
                        BEEP = 0;
                    }
                    LED0 = 0; // 亮红灯
                }
                else
                {
                    BEEP = 0;
                    LED0 = 1; // 灭红灯
                    OSMutexPend(Config_Mutex, 0, &err);
                    GlobalConfig.beep_mute = 0;
                    OSMutexPost(Config_Mutex);
                }
            }

            p_data = &data_pool[pool_idx];
            p_data->temperature = cur_temp;
            p_data->light = avg_light;
            RTC_Get_Time_Str((char *)p_data->time);

            OSQPost(UI_Queue, (void *)p_data);
            OSQPost(COM_Queue, (void *)p_data);

            pool_idx = (pool_idx + 1) % DATA_POOL_SIZE;
        }

        delay_ms(50);
    }
}

void control_task(void *pdata)
{
    u8 err;
    pdata = pdata;
    while (1)
    {
        OSSemPend(Key_Sem, 0, &err); // 等待按键 EXTI 中断同步信号量
        LED0 = 0;                    // 诊断:收到信号量→LED0亮,1秒后sensor_task恢复
        delay_ms(20);                // 软件消抖延时，过滤多次边沿触发

        OSMutexPend(Config_Mutex, 0, &err);

        if (Last_Key == 0) // KEY0: 界面切换
        {
            UI_Mode = (UI_Mode + 1) % 3;
        }
        else if (Last_Key == 1) // KEY1: 温度阈值微调
        {
            GlobalConfig.temp_limit += 2.0f;
            if (GlobalConfig.temp_limit > 80.0f)
            {
                GlobalConfig.temp_limit = 30.0f;
            }
            RTC_BKP_Write((0x55AA << 16) | (uint16_t)(GlobalConfig.temp_limit * 100));
        }
        else if (Last_Key == 2) // KEY2: 紧急拉闸
        {
            GlobalConfig.emergency_stop = !GlobalConfig.emergency_stop;
            if (GlobalConfig.emergency_stop == 0)
            {
                BEEP = 0;
                LED0 = 1;
            }
        }
        else if (Last_Key == 3) // WK_UP: 消音
        {
            GlobalConfig.beep_mute = 1;
        }

        Last_Key = 0xFF;

        OSMutexPost(Config_Mutex);
    }
}

void com_task(void *pdata)
{
    u8 err;
    SensorData_t *data;
    char log_buf[80];
    char json_str[128];
    float limit;
    char *p;
    char *p_led;
    char *p_val;
    int val_led;
    char *p_alert;
    float val_alert;

    pdata = pdata;

    while (1)
    {
        // 限时 100ms 等待发送队列，以保证有时间处理接收
        data = (SensorData_t *)OSQPend(COM_Queue, 10, &err);
        if (err == OS_ERR_NONE && data != NULL)
        {
            OSMutexPend(Config_Mutex, 0, &err);
            limit = GlobalConfig.temp_limit;
            OSMutexPost(Config_Mutex);

            sprintf(json_str, "{\"time\":\"%s\",\"temp\":%.1f,\"light\":%d,\"alert_t\":%.1f}\r\n",
                    data->time, data->temperature, data->light, limit);

            p = json_str;
            while (*p)
            {
                while ((USART1->SR & 0X40) == 0)
                    ;
                USART1->DR = (u8)*p++;
            }

            sprintf(log_buf, "TX: %s", data->time);
            Add_Comm_Log(log_buf);
        }

        // 轮询串口 DMA 接收完成标志
        if (USART_RX_FLAG == 1)
        {
            sprintf(log_buf, "RX: %d Bytes", USART_RX_LEN);
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
                    // 立即生效 GPIO,无需等 sensor_task 的 1Hz 周期
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
                    val_alert = 0.0f;
                    sscanf(p_val + 1, "%f", &val_alert);
                    if (val_alert >= 10.0f && val_alert <= 80.0f)
                    {
                        OSMutexPend(Config_Mutex, 0, &err);
                        GlobalConfig.temp_limit = val_alert;
                        RTC_BKP_Write((0x55AA << 16) | (uint16_t)(val_alert * 100));
                        OSMutexPost(Config_Mutex);
                    }
                }
            }

            USART_RX_FLAG = 0;
            USART_RX_LEN = 0;
        }
    }
}

void ui_task(void *pdata)
{
    u8 err;
    SensorData_t *data;
    char str_buf[64];

    // 声明所有局部变量于函数顶部，完美适配 C89 编译器
    float limit;
    u8 is_emergency;
    u8 led_st;
    u8 mute;
    u8 tail;
    int grid_w;
    int grid_h;
    int y_limit;
    int step_x;
    int i;
    int idx1, idx2;
    float t1, t2;
    int x1, x2, y1, y2;
    int cx, cy, r;
    int angle;
    double rad;
    float t_ratio;
    double ptr_angle, ptr_rad;
    int px, py;
    int text_y;
    int log_idx;
    OS_CPU_SR cpu_sr;
    u16 disp_w;
    u16 disp_h;

    pdata = pdata;

    // ========== 开机欢迎画面 ==========
    // 在等待传感器数据之前,先显示启动画面
    UI_Clear(BLACK);
    UI_DrawLine(20, 20, lcddev.width - 20, 20, BLUE);
    UI_DrawLine(20, lcddev.height - 20, lcddev.width - 20, lcddev.height - 20, BLUE);
    UI_DrawLine(20, 20, 20, lcddev.height - 20, BLUE);
    UI_DrawLine(lcddev.width - 20, 20, lcddev.width - 20, lcddev.height - 20, BLUE);
    // 一次性输出到 LCD
    LCD_Color_Fill(0, 0, lcddev.width - 1, lcddev.height - 1, UI_Buffer);

    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2 - 40, 200, 24, 24, (u8 *)"Gateway System");
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2, 200, 16, 16, (u8 *)"STM32F407 + UCOS-II");
    LCD_ShowString((lcddev.width - 200) / 2, lcddev.height / 2 + 30, 200, 16, 16, (u8 *)"Initializing...");
    // ========== 欢迎画面结束 ==========

    while (1)
    {
        data = (SensorData_t *)OSQPend(UI_Queue, 0, &err);
        if (err == OS_ERR_NONE && data != NULL)
        {
            OSMutexPend(Config_Mutex, 0, &err);
            limit = GlobalConfig.temp_limit;
            is_emergency = GlobalConfig.emergency_stop;
            led_st = GlobalConfig.led_state;
            mute = GlobalConfig.beep_mute;
            OSMutexPost(Config_Mutex);

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

            UI_Clear(BLACK); // 清空外部 SRAM 显存

            if (is_emergency)
            {
                // 全屏红色警告遮罩 (采用自适应宽度绘制)
                for (y1 = 30; y1 < lcddev.height - 30; y1++)
                {
                    for (x1 = 40; x1 < lcddev.width - 40; x1++)
                    {
                        UI_Buffer[y1 * lcddev.width + x1] = RED;
                    }
                }
            }
            else
            {
                if (UI_Mode == 0) // 折线图看板
                {
                    // 网格背景 (采用 lcddev.width 和 lcddev.height 映射)
                    grid_w = lcddev.width - 80;
                    grid_h = lcddev.height - 80;
                    for (i = 0; i <= 6; i++)
                    {
                        UI_DrawLine(40, 40 + i * (grid_h / 6), lcddev.width - 40, 40 + i * (grid_h / 6), GRAY);
                    }
                    for (i = 0; i <= 8; i++)
                    {
                        UI_DrawLine(40 + i * (grid_w / 8), 40, 40 + i * (grid_w / 8), lcddev.height - 40, GRAY);
                    }
                    UI_DrawLine(40, 40, 40, lcddev.height - 40, WHITE);
                    UI_DrawLine(40, lcddev.height - 40, lcddev.width - 40, lcddev.height - 40, WHITE);

                    // 红色温度阈值线
                    if (limit >= 10.0f && limit <= 60.0f)
                    {
                        y_limit = (int)((float)(lcddev.height - 40) - (limit - 10.0f) * (float)grid_h / 50.0f);
                        if (y_limit >= 40 && y_limit <= lcddev.height - 40)
                        {
                            UI_DrawLine(40, y_limit, lcddev.width - 40, y_limit, RED);
                        }
                    }

                    // 折线连接 (计算绘图点 x 跨度)
                    step_x = grid_w / PLOT_POINTS_MAX;
                    if (step_x == 0)
                        step_x = 10;
                    for (i = 0; i < (int)History_Count - 1; i++)
                    {
                        idx1 = (History_Head + i) % PLOT_POINTS_MAX;
                        idx2 = (History_Head + i + 1) % PLOT_POINTS_MAX;

                        t1 = Temp_History[idx1];
                        t2 = Temp_History[idx2];

                        x1 = 40 + i * step_x;
                        x2 = 40 + (i + 1) * step_x;

                        y1 = (int)((float)(lcddev.height - 40) - (t1 - 10.0f) * (float)grid_h / 50.0f);
                        y2 = (int)((float)(lcddev.height - 40) - (t2 - 10.0f) * (float)grid_h / 50.0f);

                        if (y1 < 40)
                            y1 = 40;
                        if (y1 > lcddev.height - 40)
                            y1 = lcddev.height - 40;
                        if (y2 < 40)
                            y2 = 40;
                        if (y2 > lcddev.height - 40)
                            y2 = lcddev.height - 40;

                        UI_DrawLine(x1, y1, x2, y2, GREEN);
                    }
                }
                else if (UI_Mode == 1) // 仪表盘看板
                {
                    // 圆心为屏幕中心
                    cx = lcddev.width / 2;
                    cy = lcddev.height * 2 / 3;
                    r = lcddev.height / 2;

                    // 绘制刻度
                    for (angle = 180; angle >= 0; angle -= 30)
                    {
                        rad = (double)angle * 3.14159 / 180.0;
                        x1 = cx + (int)((r - 20) * cos(rad));
                        y1 = cy - (int)((r - 20) * sin(rad));
                        x2 = cx + (int)(r * cos(rad));
                        y2 = cy - (int)(r * sin(rad));
                        UI_DrawLine(x1, y1, x2, y2, WHITE);
                    }

                    // 绘制指针
                    t_ratio = (data->temperature - 10.0f) / 50.0f;
                    if (t_ratio < 0.0f)
                        t_ratio = 0.0f;
                    if (t_ratio > 1.0f)
                        t_ratio = 1.0f;
                    ptr_angle = 180.0 - (double)t_ratio * 180.0;
                    ptr_rad = ptr_angle * 3.14159 / 180.0;
                    px = cx + (int)((r - 40) * cos(ptr_rad));
                    py = cy - (int)((r - 40) * sin(ptr_rad));
                    UI_DrawLine(cx, cy, px, py, YELLOW);
                }
                else if (UI_Mode == 2) // 日志看板
                {
                    UI_DrawLine(20, 20, lcddev.width - 20, 20, BLUE);
                    UI_DrawLine(20, lcddev.height - 20, lcddev.width - 20, lcddev.height - 20, BLUE);
                    UI_DrawLine(20, 20, 20, lcddev.height - 20, BLUE);
                    UI_DrawLine(lcddev.width - 20, 20, lcddev.width - 20, lcddev.height - 20, BLUE);
                }
            }

            // 一次性无撕裂输出双缓冲显存至真实 LCD
            disp_w = (lcddev.width < UI_W) ? lcddev.width : UI_W;
            disp_h = (lcddev.height < UI_H) ? lcddev.height : UI_H;
            LCD_Color_Fill(0, 0, disp_w - 1, disp_h - 1, UI_Buffer);

            POINT_COLOR = WHITE;
            BACK_COLOR = BLACK;

            if (is_emergency)
            {
                POINT_COLOR = WHITE;
                BACK_COLOR = RED;
                LCD_ShowString((lcddev.width - 360) / 2, 80, 360, 24, 24, (u8 *)"SYSTEM EMERGENCY STOP!");
                LCD_ShowString((lcddev.width - 320) / 2, 140, 320, 16, 16, (u8 *)"Press KEY2 to recover system");
            }
            else
            {
                text_y = lcddev.height - 90;
                if (text_y < 200)
                    text_y = 200; // 边界保护

                sprintf(str_buf, "Time: %s", data->time);
                LCD_ShowString(20, text_y, 200, 16, 16, (u8 *)str_buf);

                sprintf(str_buf, "Temp: %.1f C  Limit: %.1f C", data->temperature, limit);
                LCD_ShowString(20, text_y + 20, 320, 16, 16, (u8 *)str_buf);

                sprintf(str_buf, "Light: %d  Limit: %d", data->light, GlobalConfig.light_limit);
                LCD_ShowString(20, text_y + 40, 320, 16, 16, (u8 *)str_buf);

                sprintf(str_buf, "LED1: %s  Mute: %s", led_st ? "ON" : "OFF", mute ? "YES" : "NO");
                LCD_ShowString(20, text_y + 60, 240, 16, 16, (u8 *)str_buf);

                if (UI_Mode == 0)
                {
                    LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: CURVE VIEW");
                }
                else if (UI_Mode == 1)
                {
                    LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: DASHBOARD");
                }
                else if (UI_Mode == 2)
                {
                    LCD_ShowString(lcddev.width - 180, text_y, 160, 16, 16, (u8 *)"MODE: COMM LOGS");
                    POINT_COLOR = GREEN;

                    cpu_sr = 0;
                    OS_ENTER_CRITICAL();
                    for (i = 0; i < (int)Log_Count; i++)
                    {
                        log_idx = (Log_Head + i) % LOG_LINES_MAX;
                        LCD_ShowString(40, 30 + i * 20, 400, 16, 16, (u8 *)Comm_Logs[log_idx]);
                    }
                    OS_EXIT_CRITICAL();
                }
            }
        }
    }
}
