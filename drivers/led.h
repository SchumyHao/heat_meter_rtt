/*
 * File      : led.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-13-05     bright       the first version
 */

#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rthw.h>
#include <rtthread.h>
#include <stm32f0xx.h>

#define LED_GPIO_PIN                    GPIO_Pin_8
#define LED_GPIO_PIN_GROUP              GPIOA
#define LED_GPIO_PIN_RCC                RCC_AHBPeriph_GPIOA

#define rt_hw_led_on()   GPIO_SetBits(LED_GPIO_PIN_GROUP, LED_GPIO_PIN)
#define rt_hw_led_off()  GPIO_ResetBits(LED_GPIO_PIN_GROUP, LED_GPIO_PIN)

int rt_hw_led_init(void);

#ifdef __cplusplus
}
#endif

#endif
