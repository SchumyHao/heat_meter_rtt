/*
 * File      : spi_tdc_gp21.h
 * COPYRIGHT (C) 2014-2015, Schumy Hao
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-12-24     Schumy       the first version
 */
#ifndef __SPI_TDC_GP21_H__
#define __SPI_TDC_GP21_H__

/*
    include files
*/
#include <rtthread.h>
#include <drivers/spi.h>

#include "stm32f0xx.h"

#define SPI_BUS_NAME      "spi2"

/*
    struct defination
*/
struct spi_tdc_gp21_int_pin {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};
struct spi_tdc_gp21 {
    struct rt_device parent;
    struct rt_spi_device* spi_dev;
    struct spi_tdc_gp21_int_pin intpin;
    float corr_factor;
    rt_bool_t busy;
};

#define SPI_TDC_GP21_CTRL_MEASURE_TEMP   (0x01)
#define SPI_TDC_GP21_CTRL_MEASURE_TOF2   (0x02)
struct spi_tdc_gp21_temp_scales {
    float cold;
    float hot;
};
struct spi_tdc_gp21_tof_data {
    float up;
    float down;
};

/*
    global functions
*/
rt_err_t
tdc_gp21_register(const char* tdc_device_name, const char* spi_device_name);

#endif
