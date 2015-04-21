#include <rtthread.h>
#if 1
#include "finsh.h"
#include "rt_stm32f0xx_spi.h"
#include "stm32l0538_discovery_epd.h"

void
test_epd_init(void)
{
    BSP_EPD_Init();
    BSP_EPD_RefreshDisplay();
    BSP_EPD_Clear(EPD_COLOR_WHITE);
    BSP_EPD_RefreshDisplay();
    BSP_EPD_Clear(EPD_COLOR_LIGHTGRAY);
    BSP_EPD_RefreshDisplay();
    BSP_EPD_Clear(EPD_COLOR_DARKGRAY);
    BSP_EPD_RefreshDisplay();
    BSP_EPD_Clear(EPD_COLOR_BLACK);
    BSP_EPD_RefreshDisplay();
    BSP_EPD_DisplayStringAtLine(0,"Hello EPD");
    BSP_EPD_RefreshDisplay();
    BSP_EPD_DrawRect(20, 20, 20, 20);
    BSP_EPD_RefreshDisplay();
}
FINSH_FUNCTION_EXPORT(test_epd_init, test tpd);
#endif
