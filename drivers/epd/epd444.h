#ifndef __EPD_H__
#define __EPD_H__

#include <rtgui/rtgui.h>
#include <rtgui/driver.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include "rtthread.h"

#define EPD_WIDTH       72U                 /* Screen Width (in pixels)           */
#define EPD_HEIGHT      172U                /* Screen Hight (in pixels)           */
#define SPI_BUS_NAME    "spi1"

void rt_hw_epd_init(void);
#endif
