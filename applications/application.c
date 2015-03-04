/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-11-15     bright       add init thread and components initial
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include <board.h>
#include <rtthread.h>
#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#include "led.h"
#include "spi_tdc_gp21.h"

/* led thread entry */
static void led_thread_entry(void* parameter)
{
    while(1) {
        rt_hw_led_on();
        rt_thread_delay(RT_TICK_PER_SECOND);

        rt_hw_led_off();
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

/* tdc thread entry */
static void tdc_thread_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    struct spi_tdc_gp21_temp_scales temp;
    struct spi_tdc_gp21_tof_data tof;

    /* open spi bus */
    dev = rt_device_find("spi1");
    if(dev != RT_NULL) {
        rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
    }
    else {
        rt_kprintf("Can not find spi1\n");
    }
    dev = RT_NULL;

    /* register tdc device */
    gp21_register("tdc1", "spi1");
    dev = rt_device_find("tdc1");
    if(dev != RT_NULL) {
        rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
    }
    else {
        rt_kprintf("Can not find tdc1\n");
    }

    while(1) {
        rt_device_control(dev, SPI_TDC_GP21_CTRL_MEASURE_TEMP, &temp);
        rt_thread_delay(20);
        rt_device_control(dev, SPI_TDC_GP21_CTRL_MEASURE_TOF2, &tof);
        rt_thread_delay(20);
    }
}

static void rt_init_thread_entry(void* parameter)
{
    rt_thread_t tmp_thread;

    /* Initialization RT-Thread Components */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_init();
#endif

    /* Set finsh device */
#ifdef  RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */

    /* Create led thread */
    tmp_thread = rt_thread_create("led",
                                  led_thread_entry, RT_NULL,
                                  256, 20, 20);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);

    /* Create TDC thread 
    tmp_thread = rt_thread_create("tdc",
                                  tdc_thread_entry, RT_NULL,
                                  512, 10, 20);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);*/
}

int rt_application_init()
{
    rt_thread_t init_thread;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   256, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   256, 80, 20);
#endif
    if(init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}


/*@}*/
