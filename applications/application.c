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

#include <stdio.h>
#include <board.h>
#include <rtthread.h>
#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

extern void hm_store_thread_entry(void* parameter);
extern void hm_print_thread_entry(void* parameter);
extern void hm_tdc_thread_entry(void* parameter);
extern void hm_heatcal_thread_entry(void* parameter);


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

    /* Create store thread */
    tmp_thread = rt_thread_create("store",
                                  hm_store_thread_entry, RT_NULL,
                                  1024, 10, 100);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);

    /* Create print thread */
    tmp_thread = rt_thread_create("print",
                                  hm_print_thread_entry, RT_NULL,
                                  1024, 30, 100);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);

    /* Create tof thread */
    tmp_thread = rt_thread_create("tdc",
                                  hm_tdc_thread_entry, RT_NULL,
                                  1024, 3, 20);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);

    /* Create heatcal thread */
    tmp_thread = rt_thread_create("heatcal",
                                  hm_heatcal_thread_entry, RT_NULL,
                                  1024, 7, 20);
    if(tmp_thread != RT_NULL)
        rt_thread_startup(tmp_thread);
}

int rt_application_init()
{
    rt_thread_t init_thread;

    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   1024, 1, 20);
    if(init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}


/*@}*/
