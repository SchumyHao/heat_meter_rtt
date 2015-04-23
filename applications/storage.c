/*
 * File      : storage.c
 *
 * This file impliment the data store thread.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-4-22      Schumy       First version
 */

#include <rtthread.h>

#include "board.h"
#include "spi_flash_w25qxx.h"

#define WRITE_MAILBOX_MAX_NUM          (16)
#define WRITE_SEND_DELAY               (200)

static char* storagebuf;
static rt_uint16_t block_count;
static rt_mailbox_t store_mb = RT_NULL;

struct hm_store_data {
    const char* str;
    rt_size_t len;
};

rt_err_t hm_write(const char* str, rt_size_t size)
{
    struct hm_store_data write_data;
    write_data.str = str;
    write_data.len = size;

    if(RT_NULL == store_mb) {
        return -RT_EEMPTY;
    }
    return rt_mb_send_wait(store_mb, (rt_uint32_t)&write_data, WRITE_SEND_DELAY);
}

void hm_store_thread_entry(void* parameter)
{
    struct hm_store_data* write_data = RT_NULL;
    struct rt_device_blk_geometry flash_geo;
    rt_device_t flash = RT_NULL;
    char* ptr = RT_NULL;
    rt_size_t in_block_count = 0;

    store_mb = rt_mb_create("writemb", WRITE_MAILBOX_MAX_NUM, RT_IPC_FLAG_FIFO);
    RT_ASSERT(store_mb != RT_NULL);
    flash = rt_device_find(HM_BOARD_FLASH_NAME);
    RT_ASSERT(flash);
    rt_device_open(flash, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(flash, RT_DEVICE_CTRL_BLK_GETGEOME, &flash_geo);
    storagebuf = (char*)rt_malloc(flash_geo.block_size);
    RT_ASSERT(storagebuf);
    ptr = storagebuf;

    while(1) {
        if(rt_mb_recv(store_mb, (rt_uint32_t*)write_data, RT_WAITING_FOREVER)==RT_EOK) {
            const char* write_ptr = write_data->str;
            rt_size_t writen = write_data->len;
            RT_ASSERT(write_data);
            RT_ASSERT(write_data->str);

            while(writen) {
                if(writen + in_block_count > flash_geo.block_size) {
                    rt_size_t temp_len = flash_geo.block_size - in_block_count;
                    rt_memcpy(ptr, write_ptr, temp_len);
                    ptr += temp_len;
                    in_block_count += temp_len;
                    write_ptr += temp_len;
                    writen -= temp_len;
                }
                else {
                    rt_memcpy(ptr, write_ptr, writen);
                    ptr += writen;
                    in_block_count += writen;
                    writen = 0;
                }
                if(in_block_count == flash_geo.block_size) {
                    rt_device_write(flash, block_count, storagebuf, 1);
                    block_count = (block_count+1)%flash_geo.sector_count;
                    ptr = storagebuf;
                    in_block_count = 0;
                }
            }
        }
    }
}
