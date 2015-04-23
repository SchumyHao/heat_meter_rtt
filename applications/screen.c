/*
 * File      : screen.c
 *
 * This file impliment the data print thread.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-4-22      Schumy       First version
 */

#include <rtthread.h>

#include "stm32l0538_discovery_epd.h"

#define PRINT_MAILBOX_MAX_NUM          (16)
#define PRINT_SEND_DELAY               (1000)
static rt_mailbox_t print_mb = RT_NULL;

struct hm_print_data {
    rt_uint16_t line;
    const char* str;
};

rt_err_t hm_print(rt_uint16_t line, const char* str)
{
    static struct hm_print_data send_data;
    send_data.line = line;
    send_data.str = str;

    if(RT_NULL == print_mb) {
        return -RT_EEMPTY;
    }
    return rt_mb_send_wait(print_mb, (rt_uint32_t)&send_data, PRINT_SEND_DELAY);
}

static void hm_screen_first_image(void)
{
    hm_print(0,"Heat Meter!");
}

void hm_print_thread_entry(void* parameter)
{
    struct hm_print_data* recv_data;
    print_mb = rt_mb_create("printmb", PRINT_MAILBOX_MAX_NUM, RT_IPC_FLAG_FIFO);
    RT_ASSERT(print_mb != RT_NULL);
    hm_screen_first_image();

    while(1) {
        if(rt_mb_recv(print_mb, (rt_uint32_t*)&recv_data, RT_WAITING_FOREVER)==RT_EOK) {
            RT_ASSERT(recv_data);
            RT_ASSERT(recv_data->str);
            BSP_EPD_DisplayStringAtLine((uint16_t)recv_data->line, (uint8_t*)recv_data->str);
            BSP_EPD_RefreshDisplay();
        }
    }
}
