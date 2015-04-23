#include <rtthread.h>
#if 1
#include "finsh.h"
#include "stm32l0538_discovery_epd.h"

void
test_epd_init(void)
{
    BSP_EPD_DisplayStringAtLine(0,"Hello");
    BSP_EPD_DisplayStringAtLine(1,"EPD");
    BSP_EPD_RefreshDisplay();
}
FINSH_FUNCTION_EXPORT(test_epd_init, test tpd);
#endif
