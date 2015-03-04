/*
 * File      : spi.h
 * COPYRIGHT (C) 2014-2015, Schumy Hao
 * Description: SPI bus driver for stm32f0xx
 * Change Logs:
 * Date           Author       Notes
 * 2014-12-19     Schumy       the first version
 */

#ifndef __RT_STM32F0XX_SPI_H__
#define __RT_STM32F0XX_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <drivers/spi.h>
#include "stm32f0xx.h"

struct stm32_spi_bus_cs {
    void (*init)(struct stm32_spi_bus_cs* cs);
    void (*take)(struct stm32_spi_bus_cs* cs);
    void (*release)(struct stm32_spi_bus_cs* cs);
};

/* public function list */
int rt_hw_spi_bus_register(void);
#ifdef __cplusplus
}
#endif

#endif // STM32F0xx_SPI_H_INCLUDED
