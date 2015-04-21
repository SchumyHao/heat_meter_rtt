#ifndef __GDE021A1_DEVICE_H__
#define __GDE021A1_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <stdio.h>
#include "board.h"
#include "gde021a1.h"
#include "stm32f0xx.h"

/**
  * @brief  GDE021A1 Pins
  */
#define EPD_GPIO_DC_PIN                     GPIO_Pin_3
#define EPD_GPIO_DC_PIN_GROUP               GPIOA
#define EPD_GPIO_DC_PIN_RCC                 RCC_AHBPeriph_GPIOA
#define EPD_GPIO_RESET_PIN                  GPIO_Pin_2
#define EPD_GPIO_RESET_PIN_GROUP            GPIOA
#define EPD_GPIO_RESET_PIN_RCC              RCC_AHBPeriph_GPIOA
#define EPD_GPIO_BUSY_PIN                   GPIO_Pin_1
#define EPD_GPIO_BUSY_PIN_GROUP             GPIOA
#define EPD_GPIO_BUSY_PIN_RCC               RCC_AHBPeriph_GPIOA
#define EPD_GPIO_PWR_PIN                    GPIO_Pin_0
#define EPD_GPIO_PWR_PIN_GROUP              GPIOA
#define EPD_GPIO_PWR_PIN_RCC                RCC_AHBPeriph_GPIOA
#define EPD_PWR_LOW()     GPIO_ResetBits(EPD_GPIO_PWR_PIN_GROUP,EPD_GPIO_PWR_PIN)
#define EPD_PWR_HIGH()    GPIO_SetBits(EPD_GPIO_PWR_PIN_GROUP,EPD_GPIO_PWR_PIN)
#define EPD_RESET_LOW()   GPIO_ResetBits(EPD_GPIO_RESET_PIN_GROUP,EPD_GPIO_RESET_PIN)
#define EPD_RESET_HIGH()  GPIO_SetBits(EPD_GPIO_RESET_PIN_GROUP,EPD_GPIO_RESET_PIN)
#define EPD_DC_LOW()      GPIO_ResetBits(EPD_GPIO_DC_PIN_GROUP,EPD_GPIO_DC_PIN)
#define EPD_DC_HIGH()     GPIO_SetBits(EPD_GPIO_DC_PIN_GROUP,EPD_GPIO_DC_PIN)
#define EPD_BUSY_GET()    GPIO_ReadInputDataBit(EPD_GPIO_BUSY_PIN_GROUP,EPD_GPIO_BUSY_PIN)

rt_err_t
epd_gde_init(const char* epd_device_name, const char* spi_device_name);

#ifdef __cplusplus
}
#endif

#endif
