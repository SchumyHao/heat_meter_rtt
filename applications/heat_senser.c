/*
 * File      : heat_meter.c
 *
 * This file impliment the thread process senser data about heat meter.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-4-22      Schumy       First version
 */

#include <rtthread.h>

#include "spi_tdc_gp21.h"
#include "board.h"

#define TOF_DATA_BUF_LEN            (30)
static struct spi_tdc_gp21_tof_data tof_data[TOF_DATA_BUF_LEN];
#define TEMP_DATA_BUF_LEN           (5)
static struct spi_tdc_gp21_temp_scales temp_data[TEMP_DATA_BUF_LEN];

#define TOF_SLEEP_TIME_MS           (500)
#define TEMP_SLEEP_TIME_MS          (3000)

static rt_event_t cal_event;
#define TOF_DATA_FULL_EVENT         (1U<<0)
#define TEMP_DATA_FULL_EVENT        (1U<<1)

static rt_mutex_t tof_lock;
static rt_mutex_t temp_lock;
#define LOCK_TACK_WAIT_TIME_MS      (100)

#define MS_TO_TICKS(ms)             ((ms)*RT_TICK_PER_SECOND/1000)

extern rt_err_t hm_print(rt_uint16_t line, const char* str);
rt_inline void heat_print(rt_uint32_t heat)
{
    static char buf[30];
    rt_sprintf(buf, "Heat: %d", heat);
    hm_print(0, buf);
}
rt_inline void tof_print(float up, float down)
{
    static char buf[30];
    rt_sprintf(buf, "TOF: up=%.3f down=%0.3f", up, down);
    hm_print(1, buf);
}

rt_inline void temp_print(float hot, float cold)
{
    static char buf[30];
    rt_sprintf(buf, "Temp: hot=%.3f cold=%0.3f", hot, cold);
    hm_print(2, buf);
}

void hm_tof_thread_entry(void* parameter)
{
    rt_device_t tdc = RT_NULL;
    rt_uint8_t tof_data_count = 0;

    tdc = rt_device_find(HM_BOARD_TDC_NAME);
    RT_ASSERT(tdc);
    rt_device_open(tdc, RT_DEVICE_OFLAG_RDWR);
    tof_lock = rt_mutex_create("L_tof", RT_IPC_FLAG_FIFO);
    RT_ASSERT(tof_lock);

    while(1) {
        if(rt_mutex_take(tof_lock, MS_TO_TICKS(LOCK_TACK_WAIT_TIME_MS)) != RT_EOK) {
            rt_kprintf("TOF take lock error\n");
        }
        rt_device_control(tdc, SPI_TDC_GP21_CTRL_MEASURE_TOF2, tof_data+tof_data_count);
        rt_mutex_release(tof_lock);
        tof_data_count++;
        if(tof_data_count==TOF_DATA_BUF_LEN) {
            if(rt_event_send(cal_event, TOF_DATA_FULL_EVENT) != RT_EOK) {
                rt_kprintf("TOF send event error\n");
            }
        }
        rt_thread_delay(MS_TO_TICKS(TOF_SLEEP_TIME_MS));
    }
}

void hm_temp_thread_entry(void* parameter)
{
    rt_device_t tdc = RT_NULL;
    rt_uint8_t temp_data_count = 0;

    tdc = rt_device_find(HM_BOARD_TDC_NAME);
    RT_ASSERT(tdc);
    rt_device_open(tdc, RT_DEVICE_OFLAG_RDWR);
    temp_lock = rt_mutex_create("L_temp", RT_IPC_FLAG_FIFO);
    RT_ASSERT(temp_lock);

    while(1) {
        if(rt_mutex_take(temp_lock, MS_TO_TICKS(LOCK_TACK_WAIT_TIME_MS)) != RT_EOK) {
            rt_kprintf("temprature take lock error\n");
        }
        rt_device_control(tdc, SPI_TDC_GP21_CTRL_MEASURE_TEMP, temp_data+temp_data_count);
        rt_mutex_release(temp_lock);
        temp_data_count++;
        if(temp_data_count==TEMP_DATA_BUF_LEN) {
            if(rt_event_send(cal_event, TEMP_DATA_FULL_EVENT) != RT_EOK) {
                rt_kprintf("temprature send event error\n");
            }
        }
        rt_thread_delay(MS_TO_TICKS(TEMP_SLEEP_TIME_MS));
    }
}

static rt_uint32_t do_heat_cal(void)
{
    return 0;
}

void hm_heatcal_thread_entry(void* parameter)
{
    rt_uint32_t event_set = 0;
    rt_uint32_t heat_used = 0;
    cal_event = rt_event_create("H_cal", RT_IPC_FLAG_FIFO);
    RT_ASSERT(cal_event);

    while(1) {
        if(rt_event_recv(cal_event, TOF_DATA_FULL_EVENT | TEMP_DATA_FULL_EVENT,
                         RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
                         &event_set)==RT_EOK) {
            if((rt_mutex_take(temp_lock, MS_TO_TICKS(LOCK_TACK_WAIT_TIME_MS)) ||
                (rt_mutex_take(tof_lock, MS_TO_TICKS(LOCK_TACK_WAIT_TIME_MS)))) != RT_EOK) {
                rt_kprintf("TOF and temprature take lock error\n");
            }
            heat_used += do_heat_cal();
            rt_mutex_release(tof_lock);
            rt_mutex_release(temp_lock);
            heat_print(heat_used);
						rt_thread_delay(1);
        }
    }
}
