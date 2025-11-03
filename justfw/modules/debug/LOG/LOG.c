#include "LOG.h"

static osThreadId LOG_MainLoopTaskHandle;

void LOG_Init() {
    BSP_W25Qx_Init();
    easyflash_init();

    extern void EF_init();
    EF_init();

    /* init Easylogger */
    elog_init();

    /* set EasyLogger log format */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);

    /*Eenbale color*/
    elog_set_text_color_enabled(true);
    elog_flash_init();
    /* start EasyLogger */
    elog_start();

    extern void elog_entry();
    osThreadDef(LOG_MainLoop_Task, elog_entry, osPriorityLow, 0, 512);
    LOG_MainLoopTaskHandle = osThreadCreate(osThread(LOG_MainLoop_Task), NULL);
}