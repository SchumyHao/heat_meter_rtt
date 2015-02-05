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

//#define SPI_USE_DMA

struct stm32f0xx_spi_bus {
    struct rt_spi_bus parent;
    SPI_TypeDef* SPI;
#ifdef SPI_USE_DMA
    DMA_Channel_TypeDef* DMA_Channel_TX;
    DMA_Channel_TypeDef* DMA_Channel_RX;
    uint32_t DMA_Channel_TX_FLAG_TC;
    uint32_t DMA_Channel_TX_FLAG_TE;
    uint32_t DMA_Channel_RX_FLAG_TC;
    uint32_t DMA_Channel_RX_FLAG_TE;
#endif /* SPI_USE_DMA */
};

struct stm32f0xx_spi_cs {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};

/* public function list */
rt_err_t stm32f0xx_spi_register(SPI_TypeDef* SPI,
                                struct stm32f0xx_spi_bus* stm32f0xx_spi,
                                const char* spi_bus_name);

#ifdef __cplusplus
}
#endif

#endif // STM32F0xx_SPI_H_INCLUDED
