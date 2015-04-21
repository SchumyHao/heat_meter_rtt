/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 * 2013-11-15     bright       fix SRAM size for heap management
 * 2015-04-21     Schumy       add board.h to heat meter bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f0xx.h>

/* board configuration */
// <e> UART configuration
// <i> Enable on board UART
#define HM_BOARD_UART            1
//   <e> UART1
//   <i> Enable on board UART1
#define HM_BOARD_UART_1          1
//     <s0.8> name
//     <i> uart1 rt_device name
#define HM_BOARD_UART_1_NAME     "uart1"
//     <q> rx interrupt
#define HM_BOARD_UART_1_RX_INT   1
//     <q> rx DMA
#define HM_BOARD_UART_1_RX_DMA   0
//     <q> tx interrupt
#define HM_BOARD_UART_1_TX_INT   0
//     <q> tx DMA
#define HM_BOARD_UART_1_TX_DMA   0
//   </e>
//   <e> UART6
//   <i> Enable on board UART6
#define HM_BOARD_UART_6          1
//     <s0.8> name
//     <i> uart6 rt_device name
#define HM_BOARD_UART_6_NAME     "uartBLE"
//     <q> rx interrupt
#define HM_BOARD_UART_6_RX_INT   1
//     <q> rx DMA
#define HM_BOARD_UART_6_RX_DMA   0
//     <q> tx interrupt
#define HM_BOARD_UART_6_TX_INT   0
//     <q> tx DMA
#define HM_BOARD_UART_6_TX_DMA   0
//   </e>
// </e>

// <e> SPI configuration
// <i> Enable on board SPI
#define HM_BOARD_SPI_BUS          1
//   <e> SPI1
//   <i> Enable on board SPI1
#define HM_BOARD_SPI_BUS_1        1
//     <s0.8> name
//     <i> spi1 rt_device name
#define HM_BOARD_SPI_BUS_1_NAME   "spi1"
//     <q> rx interrupt
#define HM_BOARD_SPI_BUS_1_RX_INT 1
//     <q> rx DMA
#define HM_BOARD_SPI_BUS_1_RX_DMA 0
//     <q> tx interrupt
#define HM_BOARD_SPI_BUS_1_TX_INT 1
//     <q> tx DMA
#define HM_BOARD_SPI_BUS_1_TX_DMA 0
//   </e>
//   <e> SPI2
//   <i> Enable on board SPI2
#define HM_BOARD_SPI_BUS_2        1
//     <s0.8> name
//     <i> spi2 rt_device name
#define HM_BOARD_SPI_BUS_2_NAME   "spi2"
//     <q> rx interrupt
#define HM_BOARD_SPI_BUS_2_RX_INT 1
//     <q> rx DMA
#define HM_BOARD_SPI_BUS_2_RX_DMA 0
//     <q> tx interrupt
#define HM_BOARD_SPI_BUS_2_TX_INT 1
//     <q> tx DMA
#define HM_BOARD_SPI_BUS_2_TX_DMA 0
//   </e>
// </e>

// <q> LED
// <i> Enable on board LED
#define HM_BOARD_LED              1

// <e> Button configuration
// <i> Enable on board Button
#define HM_BOARD_BUT              1
//   <q> Button 1
#define HM_BOARD_BUT1             1
//   <q> Button 2
#define HM_BOARD_BUT2             1
//   <q> Button 3
#define HM_BOARD_BUT3             1
// </e>

// <e> SPI Flash configuration
// <i> Enable on board spi flash
#define HM_BOARD_FLASH            1
//   <s0.8> name
//   <i> flash rt_device name
#define HM_BOARD_FLASH_NAME       "flash"
//   <s0.8> spi device name
//   <i> flash rt_spi_dev name
#define HM_BOARD_FLASH_SPI_NAME   "spimem"
// </e>

// <e> EPD configuration
// <i> Enable on board EPD screen
#define HM_BOARD_EPD              1
//   <s0.8> name
//   <i> EPD rt_device name
#define HM_BOARD_EPD_NAME         "epd"
//   <s0.8> spi device name
//   <i> EPD rt_spi_dev name
#define HM_BOARD_EPD_SPI_NAME     "spiepd"
// </e>

// <e> TDC configuration
// <i> Enable on board TDC chip
#define HM_BOARD_TDC             1
//   <s0.8> name
//   <i> TDC rt_device name
#define HM_BOARD_TDC_NAME         "tdc"
//   <s0.8> spi device name
//   <i> TDC rt_spi_dev name
#define HM_BOARD_TDC_SPI_NAME     "spitdc"
// </e>

// <o> Internal SRAM memory size[Kbytes] <8-32>
//  <i>Default: 32
#define STM32_SRAM_SIZE         32
#define STM32_SRAM_END          (0x20000000 + STM32_SRAM_SIZE * 1024)

void rt_hw_board_init(void);

#ifdef __cplusplus
}
#endif

#endif
// <<< end of configuration section >>>
