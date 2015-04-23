/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/*******************************************************************************
 * RT Thread system config section
*******************************************************************************/
/* RT_NAME_MAX*/
#define RT_NAME_MAX	   8

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE	4

/* PRIORITY_MAX */
#define RT_THREAD_PRIORITY_MAX	32

/* Tick per Second */
#define RT_TICK_PER_SECOND	100

/* SECTION: RT_DEBUG */
/* Thread Debug */
#define RT_DEBUG
#define RT_DEBUG_INIT 1
#define RT_USING_OVERFLOW_CHECK

/* Using Hook */
/* #define RT_USING_HOOK */

/* Using Software Timer */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO		4
#define RT_TIMER_THREAD_STACK_SIZE	512
#define RT_TIMER_TICK_PER_SECOND	10

/* SECTION: IPC */
/* Using Semaphore*/
#define RT_USING_SEMAPHORE

/* Using Mutex */
#define RT_USING_MUTEX

/* Using Event */
#define RT_USING_EVENT

/* Using MailBox */
#define RT_USING_MAILBOX

/* Using Message Queue */
/* #define RT_USING_MESSAGEQUEUE */

/* SECTION: Memory Management */
/* Using Memory Pool Management*/
/* #define RT_USING_MEMPOOL */

/* Using Dynamic Heap Management */
#define RT_USING_HEAP

/* Using Small MM */
#define RT_USING_SMALL_MEM
#define RT_USING_TINY_SIZE

#define RT_USING_COMPONENTS_INIT


/*******************************************************************************
 * RT Thread decive config section
*******************************************************************************/
/* Using Device System */
#define RT_USING_DEVICE
#define RT_USING_DEVICE_IPC

#include "board.h"
#if HM_BOARD_UART
#define RT_USING_SERIAL
#if HM_BOARD_UART_1
#define RT_USING_UART1
#define RT_UART1_DEVICE_NAME      HM_BOARD_UART_1_NAME
#if HM_BOARD_UART_1_RX_INT
#define RT_UART1_RX_INT
#endif /* HM_BOARD_UART_1_RX_INT */
#if HM_BOARD_UART_1_RX_DMA
#define RT_UART1_RX_DMA
#endif /* HM_BOARD_UART_1_RX_DMA */
#if HM_BOARD_UART_1_TX_INT
#define RT_UART1_TX_INT
#endif /* HM_BOARD_UART_1_TX_INT */
#if HM_BOARD_UART_1_TX_DMA
#define RT_UART1_TX_DMA
#endif /* HM_BOARD_UART_1_TX_DMA */
#endif /* HM_BOARD_UART_1 */
#if HM_BOARD_UART_6
#define RT_USING_UART6
#define RT_UART6_DEVICE_NAME      HM_BOARD_UART_6_NAME
#if HM_BOARD_UART_6_RX_INT
#define RT_UART6_RX_INT
#endif /* HM_BOARD_UART_6_RX_INT */
#if HM_BOARD_UART_6_RX_DMA
#define RT_UART6_RX_DMA
#endif /* HM_BOARD_UART_6_RX_DMA */
#if HM_BOARD_UART_6_TX_INT
#define RT_UART6_TX_INT
#endif /* HM_BOARD_UART_6_TX_INT */
#if HM_BOARD_UART_6_TX_DMA
#define RT_UART6_TX_DMA
#endif /* HM_BOARD_UART_6_TX_DMA */
#endif /* HM_BOARD_UART_6 */
#endif /* HM_BOARD_UART */

#if HM_BOARD_SPI_BUS
#define RT_USING_SPI
#if HM_BOARD_SPI_BUS_1
#define RT_USING_SPI1
#define RT_SPI1_DEVICE_NAME      HM_BOARD_SPI_BUS_1_NAME
#if HM_BOARD_SPI_BUS_1_RX_INT
#endif /* HM_BOARD_SPI_BUS_1_RX_INT */
#if HM_BOARD_SPI_BUS_1_RX_DMA
#define RT_USING_SPI1_RX_DMA
#endif /* HM_BOARD_SPI_BUS_1_RX_DMA */
#if HM_BOARD_SPI_BUS_1_TX_INT
#endif /* HM_BOARD_SPI_BUS_1_TX_INT */
#if HM_BOARD_SPI_BUS_1_TX_DMA
#define RT_USING_SPI1_TX_DMA
#endif /* HM_BOARD_SPI_BUS_1_TX_DMA */
#endif /* HM_BOARD_SPI_BUS_1 */
#if HM_BOARD_SPI_BUS_2
#define RT_USING_SPI2
#define RT_SPI2_DEVICE_NAME      HM_BOARD_SPI_BUS_2_NAME
#if HM_BOARD_SPI_BUS_2_RX_INT
#endif /* HM_BOARD_SPI_BUS_2_RX_INT */
#if HM_BOARD_SPI_BUS_2_RX_DMA
#define RT_USING_SPI2_RX_DMA
#endif /* HM_BOARD_SPI_BUS_2_RX_DMA */
#if HM_BOARD_SPI_BUS_2_TX_INT
#endif /* HM_BOARD_SPI_BUS_2_TX_INT */
#if HM_BOARD_SPI_BUS_2_TX_DMA
#define RT_USING_SPI2_TX_DMA
#endif /* HM_BOARD_SPI_BUS_2_TX_DMA */
#endif /* HM_BOARD_SPI_BUS_2 */
#endif /* HM_BOARD_SPI_BUS */

#if HM_BOARD_LED
#define RT_USING_LED
#endif /* HM_BOARD_LED */

#if HM_BOARD_BUT
#define RT_USING_BUT
#endif /* HM_BOARD_BUT */

#if HM_BOARD_FLASH
#define RT_USING_FLASH
#define RT_FLASH_DEVICE_NAME     HM_BOARD_FLASH_NAME
#define RT_FLASH_SPI_DEVICE_NAME HM_BOARD_FLASH_SPI_NAME
#endif /* HM_BOARD_FLASH */

#if HM_BOARD_EPD
#define RT_USING_EPD
#define RT_EPD_DEVICE_NAME       HM_BOARD_EPD_NAME
#define RT_EPD_SPI_DEVICE_NAME   HM_BOARD_EPD_SPI_NAME
#endif /* HM_BOARD_EPD */


#if HM_BOARD_TDC
#define RT_USING_TDC_GP21
#define RT_TDC_DEVICE_NAME       HM_BOARD_TDC_NAME
#define RT_TDC_SPI_DEVICE_NAME   HM_BOARD_TDC_SPI_NAME
#endif /* HM_BOARD_TDC */

#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	     128
#define RT_CONSOLE_DEVICE_NAME	 RT_UART1_DEVICE_NAME

#define RT_USING_FINSH
/* configure finsh parameters */
#define FINSH_THREAD_PRIORITY    25
#define FINSH_THREAD_STACK_SIZE	 1024
#define FINSH_HISTORY_LINES	     5

/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION

/* SECTION: TC, a unit test component */
//#define RT_USING_TC

/* SECTION: libc management */
#define RT_USING_LIBC

#endif
