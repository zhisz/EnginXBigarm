//
// Created by Konodoki on 2024-03-10.
//
#include "FreertosDebug.h"

#include "FreeRTOS.h"
#include "interface.h"
#include "task.h"
#include "tinybus.h"

static char PRINT_RunTimeInfo[RunTimeInfoBuffSize];
uint32_t FreeRTOSRunTimeTicks;
/// @brief 请在任意频率大于Freertos的计时器中断函数中调用,否则无法统计CPU利用率
void Time_IR_Callback()  // 定时器回调，用于计时
{
    FreeRTOSRunTimeTicks++;
}
int split_line(const char *b_str, char ***str_lines, int *len) {
    char *s = "\n";
    char *b_str_tmp = (char *)b_str;
    int cnt = 0;
    char *buf = strstr(b_str, s);
    while (buf != NULL) {
        cnt++;
        b_str = buf + strlen(s);
        buf = strstr(b_str, s);
    }
    *str_lines = (char **)JUST_MALLOC(sizeof(char *) * cnt);
    b_str = b_str_tmp;
    int i = 0;
    buf = strstr(b_str, s);
    while (buf != NULL) {
        buf[0] = '\0';
        (*str_lines)[i] = (char *)b_str;
        b_str = buf + strlen(s);
        buf = strstr(b_str, s);
        i++;
    }
    *len = cnt;
    return 0;
}
void configureTimerForRunTimeStats(void) {
    FreeRTOSRunTimeTicks = 0;
}
unsigned long getRunTimeCounterValue(void) {
    return FreeRTOSRunTimeTicks;
}
void PrintFreeRTOSelog_info() {
    // B：阻塞  R：就绪  D：删除  S：暂停  X：运行
#ifdef English
    elog_info("  B: Block  R: Ready  D: Delete  S: Stop  X: Running \r\n");
#else
    elog_info("FreeRtos", "  B：阻塞    R：就绪     D：删除     S：暂停   X：运行 \r\n");
#endif

    elog_info("FreeRtos", "---------------------------------------------\r\n");
    // 任务名 任务状态  优先级   剩余栈  任务序号
#ifdef English
    elog_info("Task      Task_Status Priority  Remaining_Stack Task_No.\r\n");
#else
    elog_info("FreeRtos", "任务名        任务状态  优先级  剩余栈 任务序号 \r\n");
#endif

    memset(PRINT_RunTimeInfo, 0, RunTimeInfoBuffSize);  // 信息缓冲区清零
    osThreadList(PRINT_RunTimeInfo);                    // 获取任务运行时间信息
    // 将信息分割成一行一行
    int info_len;
    char **info_lines;
    split_line(PRINT_RunTimeInfo, &info_lines, &info_len);
    for (int i = 0; i < info_len; ++i) {
        elog_info("FreeRtos", info_lines[i]);
    }
    JUST_FREE(info_lines);
    elog_info("FreeRtos", "---------------------------------------------\r\n");
    memset(PRINT_RunTimeInfo, 0, RunTimeInfoBuffSize);  // 信息缓冲区清零
    vTaskGetRunTimeStats((char *)&PRINT_RunTimeInfo);

    split_line(PRINT_RunTimeInfo, &info_lines, &info_len);
    // 任务 运行次数 CPU利用率
#ifdef English
    elog_info("Task       Running_Count        Cpu_Utilization\r\n");
#else
    elog_info("FreeRtos", "任务       运行次数               CPU利用率 \r\n");
#endif
    for (int i = 0; i < info_len; ++i) {
        elog_info("FreeRtos", info_lines[i]);
    }

    JUST_FREE(info_lines);
    elog_info("FreeRtos", "---------------------------------------------\r\n\n\n\n");
}
void vTaskRunTimeStats()  // 这是打印的信息
{
    for (;;) {
        PrintFreeRTOSelog_info();
        osDelay(1000);
    }
}
void Freeretos_Debug_Start() {
    osThreadDef(Freeretos_Debug_Task, vTaskRunTimeStats, osPriorityLow, 0, 256);
    // osThreadCreate(osThread(Freeretos_Debug_Task), NULL);
}
