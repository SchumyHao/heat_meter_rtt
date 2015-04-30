/*
 * File      : spi_tdc_gp21.c
 * COPYRIGHT (C) 2014-2015, Schumy Hao
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-12-24     Schumy       the first version
 */

/*
    include files
*/
#include "spi_tdc_gp21.h"
#include "rt_stm32f0xx_spi.h"

/*
    local define
*/
//#define TDC_DEBUG

#ifdef TDC_DEBUG
#define TDC_TRACE       rt_kprintf
#else
#define TDC_TRACE(...)
#endif

#define TDC_GPIO_IT_PIN                     GPIO_Pin_1
#define TDC_GPIO_IT_PIN_RCC                 RCC_AHBPeriph_GPIOB
#define TDC_GPIO_IT_PIN_GROUP               GPIOB
#define TDC_GPIO_IT_PIN_PIN_SOURCE          EXTI_PinSource1
#define TDC_GPIO_IT_PIN_PORT_SOURCE         EXTI_PortSourceGPIOB
#define TDC_GPIO_IT_EXTI_LINE               EXTI_Line1
#define TDC_GPIO_IT_EXTI_IRQN               EXTI0_1_IRQn
#define TDC_GPIO_NRST_PIN                   GPIO_Pin_2
#define TDC_GPIO_NRST_PIN_RCC               RCC_AHBPeriph_GPIOB
#define TDC_GPIO_NRST_PIN_GROUP             GPIOB
#define TDC_GPIO_EN_STOP1_PIN               GPIO_Pin_12
#define TDC_GPIO_EN_STOP1_PIN_RCC           RCC_AHBPeriph_GPIOB
#define TDC_GPIO_EN_STOP1_PIN_GROUP         GPIOB
#define TDC_GPIO_EN_STOP2_PIN               GPIO_Pin_11
#define TDC_GPIO_EN_STOP2_PIN_RCC           RCC_AHBPeriph_GPIOB
#define TDC_GPIO_EN_STOP2_PIN_GROUP         GPIOB

/* write config registers opcode */
#define GP21_WRITE_REG0_REGISTER            (0x80)
#define GP21_WRITE_REG1_REGISTER            (0x81)
#define GP21_WRITE_REG2_REGISTER            (0x82)
#define GP21_WRITE_REG3_REGISTER            (0x83)
#define GP21_WRITE_REG4_REGISTER            (0x84)
#define GP21_WRITE_REG5_REGISTER            (0x85)
#define GP21_WRITE_REG6_REGISTER            (0x86)
/* read read registers opcode */
#define GP21_READ_RES0_REGISTER             (0xB0)
#define GP21_READ_RES1_REGISTER             (0xB1)
#define GP21_READ_RES2_REGISTER             (0xB2)
#define GP21_READ_RES3_REGISTER             (0xB3)
#define GP21_READ_STAT_REGISTER             (0xB4)
#define GP21_READ_REG1_REGISTER             (0xB5)
/* others opcode */
#define GP21_READ_ID                        (0xB7)
#define GP21_WRITE_CFG_TO_EEPROM            (0xC0)
#define GP21_WRITE_EEPROM_TO_CFG            (0xF0)
#define GP21_COMPARE_EEPROM_CFG             (0xC6)
#define GP21_INITIATE_TDC                   (0x70)
#define GP21_POWER_ON_RESET                 (0x50)
#define GP21_START_TOF                      (0x01)
#define GP21_START_TEMP                     (0x02)
#define GP21_START_CAL_RESONATOR            (0x03)
#define GP21_START_CAL_TDC                  (0x04)
#define GP21_START_TOF_RESTART              (0x05)
#define GP21_START_TEMP_RESTART             (0x06)
/* interrupt flag */
#define GP21_INT_TIMEOUT_BIT                (31)
#define GP21_INT_TIMEOUT_ENABLE             (1<<GP21_INT_TIMEOUT_BIT)
#define GP21_INT_TIMEOUT_DISABLE            (0<<GP21_INT_TIMEOUT_BIT)
#define GP21_INT_ENDHITS_BIT                (30)
#define GP21_INT_ENDHITS_ENABLE             (1<<GP21_INT_ENDHITS_BIT)
#define GP21_INT_ENDHITS_DISABLE            (0<<GP21_INT_ENDHITS_BIT)
#define GP21_INT_ALUOVER_BIT                (29)
#define GP21_INT_ALUOVER_ENABLE             (1<<GP21_INT_ALUOVER_BIT)
#define GP21_INT_ALUOVER_DISABLE            (0<<GP21_INT_ALUOVER_BIT)
#define GP21_INT_EEPROM_BIT                 (21)
#define GP21_INT_EEPROM_ENABLE              (1<<GP21_INT_EEPROM_BIT)
#define GP21_INT_EEPROM_DISABLE             (0<<GP21_INT_EEPROM_BIT)
/* error masks */
#define GP21_ERR_MASK_TDC_TIMEOUT           (0x0200)
#define GP21_ERR_MASK_PRE_CNT_TIMEOUT       (0x0400)
#define GP21_ERR_MASK_TEMP_SENSER_OPEN      (0x0800)
#define GP21_ERR_MASK_TEMP_SENSER_SHORT     (0x1000)
#define GP21_ERR_MASK_EEPROM_ERR            (0x2000)
#define GP21_ERR_MASK_EEPROM_ERRS           (0x4000)
#define GP21_ERR_ERRORS                     (GP21_ERR_MASK_EEPROM_ERRS       |\
                                             GP21_ERR_MASK_EEPROM_ERR        |\
                                             GP21_ERR_MASK_TEMP_SENSER_SHORT |\
                                             GP21_ERR_MASK_TEMP_SENSER_OPEN  |\
                                             GP21_ERR_MASK_PRE_CNT_TIMEOUT   |\
                                             GP21_ERR_MASK_TDC_TIMEOUT)

/* config value */
#define GP21_CONFIG_VALUE_ID_H              (0x00445679U)
#define GP21_CONFIG_VALUE_ID_L              (0x12345678U)
#define GP21_CONFIG_VALUE_STOPMASK_DELAY_US (200U)
#define GP21_CONFIG_VALUE_ANZ_FIRE          (10U)
#define GP21_CONFIG_VALUE_FIRE_DIV          (3U)
/*
    config register0 :
    ANZ_FIRE = 31        DIV_FIRE = 3      ANZ_PER_CALRES = 3
    DIV_CLKHS = 0        START_CLKHS = 2   ANZ_PORT = 1
    TCYCLE = 1           ANZ_FAKE = 0      SEL_ECLK_TMP = 1
    CALIBRATE = 1        NO_CAL_AUTO = 0   MESSB2 = 1
    NEG_STOP/NEGSTART=0  ID0 = 0x10
*/
#define GP21_CONFIG_VALUE_ANZ_FIRE_L(v)     (((v)&0x0FU)<<28)
#define GP21_CONFIG_VALUE_DIV_FIRE(v)       (((v)&0x0FU)<<24)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_MASK (3U<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_2  (0U<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_4  (1U<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_8  (2U<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_16 (3U<<22)
#define GP21_CONFIG_VALUE_DIV_CLKHS_MASK    (3U<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_1       (0U<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_2       (1U<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_4       (2U<<20)
#define GP21_CONFIG_VALUE_START_CLKHS_L(v)  (((v)&0x03U)<<18)
#define GP21_CONFIG_VALUE_START_CLKHS_OFF   (0U)
#define GP21_CONFIG_VALUE_START_CLKHS_ON    (1U)
#define GP21_CONFIG_VALUE_START_CLKHS_480US (2U)
#define GP21_CONFIG_VALUE_START_CLKHS_1MS   (3U)
#define GP21_CONFIG_VALUE_START_CLKHS_2MS   (4U)
#define GP21_CONFIG_VALUE_START_CLKHS_5MS   (5U)
#define GP21_CONFIG_VALUE_ANZ_PORT_MASK     (1U<<17)
#define GP21_CONFIG_VALUE_ANZ_PORT_2        (0U<<17)
#define GP21_CONFIG_VALUE_ANZ_PORT_4        (1U<<17)
#define GP21_CONFIG_VALUE_TCYCLE_MASK       (1U<<16)
#define GP21_CONFIG_VALUE_TCYCLE_128US      (0U<<16)
#define GP21_CONFIG_VALUE_TCYCLE_512US      (1U<<16)
#define GP21_CONFIG_VALUE_ANZ_FAKE_MASK     (1U<<15)
#define GP21_CONFIG_VALUE_ANZ_FAKE_2        (0U<<15)
#define GP21_CONFIG_VALUE_ANZ_FAKE_7        (1U<<15)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_MASK (1U<<14)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_L    (0U<<14)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_H    (1U<<14)
#define GP21_CONFIG_VALUE_CALIBRATE_MASK    (1U<<13)
#define GP21_CONFIG_VALUE_CALIBRATE_OFF     (0U<<13)
#define GP21_CONFIG_VALUE_CALIBRATE_ON      (1U<<13)
#define GP21_CONFIG_VALUE_NO_CAL_AUTO_MASK  (1U<<12)
#define GP21_CONFIG_VALUE_CAL_AUTO_ON       (0U<<12)
#define GP21_CONFIG_VALUE_CAL_AUTO_OFF      (1U<<12)
#define GP21_CONFIG_VALUE_MRANGE_MASK       (1U<<11)
#define GP21_CONFIG_VALUE_MRANGE_1          (0U<<11)
#define GP21_CONFIG_VALUE_MRANGE_2          (1U<<11)
#define GP21_CONFIG_VALUE_STOP2_EDGE_MASK   (1U<<10)
#define GP21_CONFIG_VALUE_STOP2_EDGE_RISE   (0U<<10)
#define GP21_CONFIG_VALUE_STOP2_EDGE_FALL   (1U<<10)
#define GP21_CONFIG_VALUE_STOP1_EDGE_MASK   (1U<<9)
#define GP21_CONFIG_VALUE_STOP1_EDGE_RISE   (0U<<9)
#define GP21_CONFIG_VALUE_STOP1_EDGE_FALL   (1U<<9)
#define GP21_CONFIG_VALUE_START_EDGE_MASK   (1U<<8)
#define GP21_CONFIG_VALUE_START_EDGE_RISE   (0U<<8)
#define GP21_CONFIG_VALUE_START_EDGE_FALL   (1U<<8)
#define GP21_CONFIG_VALUE_DEF_REG0          (0x00000000U)
#define GP21_CONFIG_VALUE_ID0(v)            (((v)>>0)&0xFFU)

/*
    config register1 :
    HIT2 = 2             HIT1 = 1          EN_FAST_INIT = 0
    HITIN2 = 0           HITIN1 = 4        CURR32K = 0
    SEL_START_FIRE = 1   SEL_TSTO2 = 2     SEL_TSTO1 = 7
    ID1 = 0x12
*/
#define GP21_CONFIG_VALUE_HIT2_MASK         (15U<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_1 (2U<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_2 (3U<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_3 (4U<<28)
#define GP21_CONFIG_VALUE_HIT1_MASK         (15U<<24)
#define GP21_CONFIG_VALUE_HIT1_MR2_CH_START (1U<<24)
#define GP21_CONFIG_VALUE_FAST_INIT_MASK    (1U<<23)
#define GP21_CONFIG_VALUE_FAST_INIT_OFF     (0U<<23)
#define GP21_CONFIG_VALUE_FAST_INIT_ON      (1U<<23)
#define GP21_CONFIG_VALUE_HITIN_SP2_MASK    (7U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_OFF     (0U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_1       (1U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_2       (2U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_3       (3U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_4       (4U<<19)
#define GP21_CONFIG_VALUE_HITIN_SP1_MASK    (7U<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_OFF     (0U<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_1       (1U<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_2       (2U<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_3       (3U<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_4       (4U<<16)
#define GP21_CONFIG_VALUE_CURR32_MASK       (1U<<15)
#define GP21_CONFIG_VALUE_CURR32_L          (0U<<15)
#define GP21_CONFIG_VALUE_CURR32_H          (1U<<15)
#define GP21_CONFIG_VALUE_SEL_FIRE_MASK     (1U<<14)
#define GP21_CONFIG_VALUE_SEL_FIRE_OUT      (0U<<14)
#define GP21_CONFIG_VALUE_SEL_FIRE_INTER    (1U<<14)
#define GP21_CONFIG_VALUE_ENSTART_FN_MASK   (7U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_IN     (0U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_START  (1U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_SP1    (2U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_SP2    (3U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_TEMP_SP (4U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_TOF_ST (5U<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_4KHZ   (7U<<11)
#define GP21_CONFIG_VALUE_FIREIN_FN_MASK    (7U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_IN      (0U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_START   (1U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_SP1     (2U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_SP2     (3U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_TEMP_SP (4U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_ENSP    (5U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_COMP    (6U<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_32KHZ   (7U<<8)
#define GP21_CONFIG_VALUE_DEF_REG1          (0x00400000U)
#define GP21_CONFIG_VALUE_ID1(v)            (((v)>>8)&0xFFU)

/*
    config register2 :
    EN_INT = B1111       RFEDGE1 = 0       RFEDGE2 = 0
    DELVAL1 = 400        ID2 = 0x12
*/
#define GP21_CONFIG_VALUE_TIMEOUT_INT_MASK  (1U<<31)
#define GP21_CONFIG_VALUE_TIMEOUT_INT_ON    (1U<<31)
#define GP21_CONFIG_VALUE_TIMEOUT_INT_OFF   (0U<<31)
#define GP21_CONFIG_VALUE_HITEND_INT_MASK   (1U<<30)
#define GP21_CONFIG_VALUE_HITEND_INT_ON     (1U<<30)
#define GP21_CONFIG_VALUE_HITEND_INT_OFF    (0U<<30)
#define GP21_CONFIG_VALUE_ALU_INT_MASK      (1U<<29)
#define GP21_CONFIG_VALUE_ALU_INT_ON        (1U<<29)
#define GP21_CONFIG_VALUE_ALU_INT_OFF       (0U<<29)
#define GP21_CONFIG_VALUE_CH2_EDGE_MASK     (1U<<28)
#define GP21_CONFIG_VALUE_CH2_EDGE_BOTH     (1U<<28)
#define GP21_CONFIG_VALUE_CH2_EDGE_ONE      (0U<<28)
#define GP21_CONFIG_VALUE_CH1_EDGE_MASK     (1U<<27)
#define GP21_CONFIG_VALUE_CH1_EDGE_BOTH     (1U<<27)
#define GP21_CONFIG_VALUE_CH1_EDGE_ONE      (0U<<27)
#define GP21_CONFIG_VALUE_DELVAL1_MASK      (0x07FFFFU<<8)
#define GP21_CONFIG_VALUE_DELVAL1(v)        ((v)<<(7-(GP21_CONFIG_VALUE_DIV_CLKHS_1>>20)+8))
#define GP21_CONFIG_VALUE_DEF_REG2          (0x00000000U)
#define GP21_CONFIG_VALUE_ID2(v)            (((v)>>16)&0xFFU)

/*
    config register3 :
    EN_ERR_VAL = 0       SEL_TIMO_MB2 = 1  DELVAL2 = 408
    ID3 = 0x34
*/
#define GP21_CONFIG_VALUE_EN_ERR_VAL_MASK   (1U<<29)
#define GP21_CONFIG_VALUE_EN_ERR_VAL_OFF    (0U<<29)
#define GP21_CONFIG_VALUE_EN_ERR_VAL_ON     (1U<<29)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_MASK (3U<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_64   (0U<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_256  (1U<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_1024 (2U<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_4096 (3U<<27)
#define GP21_CONFIG_VALUE_DELVAL2_MASK      (0x07FFFFU<<8)
#define GP21_CONFIG_VALUE_DELVAL2(v)        ((GP21_CONFIG_VALUE_DELVAL1(v))+ \
        (4<<(5+(GP21_CONFIG_VALUE_DIV_CLKHS_1>>20)+8)))
#define GP21_CONFIG_VALUE_DEF_REG3          (0x00000000U)
#define GP21_CONFIG_VALUE_ID3(v)            (((v)>>24)&0xFFU)

/*
    config register4 :
    DELVAL3 = 416        ID4 = 0x56
*/
#define GP21_CONFIG_VALUE_DELVAL3_MASK      (0x07FFFFU<<8)
#define GP21_CONFIG_VALUE_DELVAL3(v)        ((GP21_CONFIG_VALUE_DELVAL2(v))+ \
        (4<<(5+(GP21_CONFIG_VALUE_DIV_CLKHS_1>>20)+8)))
#define GP21_CONFIG_VALUE_DEF_REG4          (0x20000000U)
#define GP21_CONFIG_VALUE_ID4(v)            (((v)>>0)&0xFFU)

/*
    config register5 :
    CON_FIRE = 0         EN_STARTNOISE = 0 DIS_PHASESHIFT = 0
    REPEAT_FIRE = 0      PHASE_FIRE = 0    ID5 = 0x78
*/
#define GP21_CONFIG_VALUE_CONF_FIRE_MASK    (7U<<29)
#define GP21_CONFIG_VALUE_CONF_FIRE_BOTH    (1U<<31)
#define GP21_CONFIG_VALUE_CONF_FIRE_EN_UP   (1U<<30)
#define GP21_CONFIG_VALUE_CONF_FIRE_EN_DOWN (1U<<29)
#define GP21_CONFIG_VALUE_STARTNOISE_MASK   (1U<<28)
#define GP21_CONFIG_VALUE_STARTNOISE_ON     (1U<<28)
#define GP21_CONFIG_VALUE_STARTNOISE_OFF    (0U<<28)
#define GP21_CONFIG_VALUE_DIS_PH_NOISE_MASK (1U<<27)
#define GP21_CONFIG_VALUE_PH_NOISE_ON       (0U<<27)
#define GP21_CONFIG_VALUE_PH_NOISE_OFF      (1U<<27)
#define GP21_CONFIG_VALUE_REPEAT_FIRE_MASK  (7U<<24)
#define GP21_CONFIG_VALUE_REPEAT_FIRE_TIMES(v) (((v)&0x07U)<<24)
#define GP21_CONFIG_VALUE_PH_FIRE_MASK      (0xFFFFU<<8)
#define GP21_CONFIG_VALUE_PH_FIRE_INV(ch)   (1U<<(8+(ch)))
#define GP21_CONFIG_VALUE_DEF_REG5          (0x00000000U)
#define GP21_CONFIG_VALUE_ID5(v)            (((v)>>8)&0xFFU)

/*
    config register6 :
    EN_ANALOG = 1        NEG_STOP_TEMP = 1 TW2 = 1
    EN_INT = b1111       START_CLKHS = 2   CYCLE_TEMP = 1
    CYCLE_TOF = 0        HZ60 = 0          FIREO_DEF = 1
    QUAD_RES = 0         DOUBLE_RES = 1    TEMP_PORTDIR = 0
    ANZ_FIRE = 31        ID6 = 0x90
*/
#define GP21_CONFIG_VALUE_EN_ANALOG_MASK    (1U<<31)
#define GP21_CONFIG_VALUE_ANALOG_ON         (1U<<31)
#define GP21_CONFIG_VALUE_ANALOG_OFF        (0U<<31)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_MASK (1U<<30)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_EX  (0U<<30)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_INTER (1U<<30)
#define GP21_CONFIG_VALUE_COMP_OFFSET_MASK  (15U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_0MV   (0U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_1MV   (1U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_2MV   (2U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_3MV   (3U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_4MV   (4U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_5MV   (5U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_6MV   (6U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_7MV   (7U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M1MV  (15U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M2MV  (14U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M3MV  (13U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M4MV  (12U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M5MV  (11U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M6MV  (10U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M7MV  (9U<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M8MV  (8U<<25)
#define GP21_CONFIG_VALUE_RC_TIME_MASK      (3U<<22)
#define GP21_CONFIG_VALUE_RC_TIME_90US      (0U<<22)
#define GP21_CONFIG_VALUE_RC_TIME_120US     (1U<<22)
#define GP21_CONFIG_VALUE_RC_TIME_150US     (2U<<22)
#define GP21_CONFIG_VALUE_RC_TIME_300US     (3U<<22)
#define GP21_CONFIG_VALUE_EEPROM_INT_MASK   (1U<<21)
#define GP21_CONFIG_VALUE_EEPROM_INT_ON     (1U<<21)
#define GP21_CONFIG_VALUE_EEPROM_INT_OFF    (0U<<21)
#define GP21_CONFIG_VALUE_START_CLKHS_H(v)  ((((v)>>2)&0x01U)<<20)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_MASK   (3U<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_1      (0U<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_1_5    (1U<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_2      (2U<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_2_5    (3U<<18)
#define GP21_CONFIG_VALUE_CYCLE_TOF_MASK    (3U<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_1       (0U<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_1_5     (1U<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_2       (2U<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_2_5     (3U<<16)
#define GP21_CONFIG_VALUE_HZ60_MASK         (1U<<15)
#define GP21_CONFIG_VALUE_HZ60              (1U<<15)
#define GP21_CONFIG_VALUE_HZ50              (0U<<15)
#define GP21_CONFIG_VALUE_FIREO_DEF_MASK    (1U<<14)
#define GP21_CONFIG_VALUE_FIREO_HZ          (0U<<14)
#define GP21_CONFIG_VALUE_FIREO_L           (1U<<14)
#define GP21_CONFIG_VALUE_QUAD_RES_MASK     (1U<<13)
#define GP21_CONFIG_VALUE_QUAD_RES_ON       (1U<<13)
#define GP21_CONFIG_VALUE_QUAD_RES_OFF      (0U<<13)
#define GP21_CONFIG_VALUE_DOUBLE_RES_MASK   (1U<<12)
#define GP21_CONFIG_VALUE_DOUBLE_RES_ON     (1U<<12)
#define GP21_CONFIG_VALUE_DOUBLE_RES_OFF    (0U<<12)
#define GP21_CONFIG_VALUE_TEMP_PORTDIR_MASK (1U<<11)
#define GP21_CONFIG_VALUE_TEMP_PORT_1_4     (0U<<11)
#define GP21_CONFIG_VALUE_TEMP_PORT_4_1     (1U<<11)
#define GP21_CONFIG_VALUE_ANZ_FIRE_H(v)     ((((v)>>4)&0x07U)<<8)
#define GP21_CONFIG_VALUE_DEF_REG6          (0x00000000U)
#define GP21_CONFIG_VALUE_ID6(v)            (((v)>>16)&0xFFU)

#define GP21_CONFIG_VALUE_REG0              (GP21_CONFIG_VALUE_DEF_REG0|\
        GP21_CONFIG_VALUE_ANZ_FIRE_L(GP21_CONFIG_VALUE_ANZ_FIRE)|\
        GP21_CONFIG_VALUE_DIV_FIRE(GP21_CONFIG_VALUE_FIRE_DIV)|\
        GP21_CONFIG_VALUE_ANZ_PER_CALRES_2  |\
        GP21_CONFIG_VALUE_DIV_CLKHS_1       |\
        GP21_CONFIG_VALUE_START_CLKHS_L(GP21_CONFIG_VALUE_START_CLKHS_1MS) |\
        GP21_CONFIG_VALUE_ANZ_PORT_4        |\
        GP21_CONFIG_VALUE_TCYCLE_512US      |\
        GP21_CONFIG_VALUE_ANZ_FAKE_7        |\
        GP21_CONFIG_VALUE_SEL_ECLK_TMP_H    |\
        GP21_CONFIG_VALUE_CALIBRATE_ON      |\
        GP21_CONFIG_VALUE_CAL_AUTO_ON       |\
        GP21_CONFIG_VALUE_MRANGE_2          |\
        GP21_CONFIG_VALUE_STOP2_EDGE_RISE   |\
        GP21_CONFIG_VALUE_STOP1_EDGE_RISE   |\
        GP21_CONFIG_VALUE_START_EDGE_RISE   |\
        GP21_CONFIG_VALUE_ID0(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG1              (GP21_CONFIG_VALUE_DEF_REG1|\
        GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_1|\
        GP21_CONFIG_VALUE_HIT1_MR2_CH_START |\
        GP21_CONFIG_VALUE_FAST_INIT_OFF     |\
        GP21_CONFIG_VALUE_HITIN_SP2_OFF     |\
        GP21_CONFIG_VALUE_HITIN_SP1_4       |\
        GP21_CONFIG_VALUE_CURR32_L          |\
        GP21_CONFIG_VALUE_SEL_FIRE_INTER    |\
        GP21_CONFIG_VALUE_ENSTART_FN_SP1   |\
        GP21_CONFIG_VALUE_FIREIN_FN_SP2   |\
        GP21_CONFIG_VALUE_ID1(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG2              (GP21_CONFIG_VALUE_DEF_REG2|\
        GP21_CONFIG_VALUE_TIMEOUT_INT_ON    |\
        GP21_CONFIG_VALUE_HITEND_INT_ON     |\
        GP21_CONFIG_VALUE_ALU_INT_ON        |\
        GP21_CONFIG_VALUE_CH2_EDGE_ONE      |\
        GP21_CONFIG_VALUE_CH1_EDGE_ONE      |\
        GP21_CONFIG_VALUE_ID2(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG3              (GP21_CONFIG_VALUE_DEF_REG3|\
        GP21_CONFIG_VALUE_EN_ERR_VAL_ON     |\
        GP21_CONFIG_VALUE_SEL_TIMO_MB2_4096 |\
        GP21_CONFIG_VALUE_ID3(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG4              (GP21_CONFIG_VALUE_DEF_REG4|\
        GP21_CONFIG_VALUE_ID4(GP21_CONFIG_VALUE_ID_H))

#define GP21_CONFIG_VALUE_REG5              (GP21_CONFIG_VALUE_DEF_REG5|\
        GP21_CONFIG_VALUE_CONF_FIRE_EN_UP   |\
        GP21_CONFIG_VALUE_STARTNOISE_OFF    |\
        GP21_CONFIG_VALUE_PH_NOISE_ON       |\
        GP21_CONFIG_VALUE_REPEAT_FIRE_TIMES(0)|\
        GP21_CONFIG_VALUE_ID5(GP21_CONFIG_VALUE_ID_H))

#define GP21_CONFIG_VALUE_REG6              (GP21_CONFIG_VALUE_DEF_REG6|\
        GP21_CONFIG_VALUE_ANALOG_ON         |\
        GP21_CONFIG_VALUE_NEG_STOP_TEMP_INTER |\
        GP21_CONFIG_VALUE_COMP_OFFSET_0MV   |\
        GP21_CONFIG_VALUE_RC_TIME_300US     |\
        GP21_CONFIG_VALUE_EEPROM_INT_ON     |\
        GP21_CONFIG_VALUE_START_CLKHS_H(GP21_CONFIG_VALUE_START_CLKHS_OFF) |\
        GP21_CONFIG_VALUE_CYCLE_TEMP_2      |\
        GP21_CONFIG_VALUE_CYCLE_TOF_2       |\
        GP21_CONFIG_VALUE_HZ50              |\
        GP21_CONFIG_VALUE_FIREO_L           |\
        GP21_CONFIG_VALUE_QUAD_RES_OFF      |\
        GP21_CONFIG_VALUE_DOUBLE_RES_ON    |\
        GP21_CONFIG_VALUE_TEMP_PORT_1_4     |\
        GP21_CONFIG_VALUE_ANZ_FIRE_H(GP21_CONFIG_VALUE_ANZ_FIRE)|\
        GP21_CONFIG_VALUE_ID6(GP21_CONFIG_VALUE_ID_H))


/*
    local functions
*/
#define TICKS_PER_US   (SystemCoreClock/1000000)
static void tdc_gp21_us_delay(rt_uint32_t us)
{
    rt_uint32_t delta;
    us = us*TICKS_PER_US;
    delta = SysTick->VAL;
    while((delta - SysTick->VAL) < us);
}

rt_inline float tdc_gp21_reg_to_float(rt_uint32_t reg)
{
    return (float)reg/65536.0;
}

rt_inline double tdc_gp21_reg_to_double(rt_uint32_t reg)
{
    return (double)reg/65536.0;
}

#if 0
rt_inline rt_uint8_t
tdc_gp21_read_register8(struct spi_tdc_gp21* tdc_gp21,
                        const rt_uint8_t opcode)
{
    rt_uint8_t recv_buf = 0;
    rt_spi_send_then_recv(tdc_gp21->spi_dev,
                          &opcode, 1,
                          &recv_buf, 1);
    return recv_buf;
}
#endif
rt_inline rt_uint16_t
tdc_gp21_read_register16(struct spi_tdc_gp21* tdc_gp21,
                         const rt_uint8_t opcode)
{
    rt_uint8_t recv_buf[2];
    rt_uint16_t recv_data = 0;
    rt_uint8_t* ptr = (rt_uint8_t*)&recv_data;
    rt_int8_t i = 1;

    rt_spi_send_then_recv(tdc_gp21->spi_dev,
                          &opcode, 1,
                          &recv_buf, 2);
    for(; i>=0; i--) {
        *ptr = recv_buf[i];
        ptr++;
    }

    return recv_data;
}

rt_inline rt_uint32_t
tdc_gp21_read_register32(struct spi_tdc_gp21* tdc_gp21,
                         const rt_uint8_t opcode)
{
    rt_uint8_t recv_buf[4];
    rt_uint32_t recv_data = 0;
    rt_uint8_t* ptr = (rt_uint8_t*)&recv_data;
    rt_int8_t i = 3;

    rt_spi_send_then_recv(tdc_gp21->spi_dev,
                          &opcode, 1,
                          &recv_buf, 4);
    for(; i>=0; i--) {
        *ptr = recv_buf[i];
        ptr++;
    }

    return recv_data;
}

rt_inline void
tdc_gp21_write_register24(struct spi_tdc_gp21* tdc_gp21,
                          const rt_uint8_t opcode,
                          rt_uint32_t send_data)
{
    rt_uint8_t send_buf[3];
    rt_uint8_t* ptr = ((rt_uint8_t*)&send_data)+1;
    rt_int8_t i = 2;
    for(; i>=0; i--) {
        send_buf[i] = *ptr;
        ptr++;
    }
    rt_spi_send_then_send(tdc_gp21->spi_dev,
                          &opcode, 1,
                          &send_buf, 3);
}

rt_inline void
tdc_gp21_write_register32(struct spi_tdc_gp21* tdc_gp21,
                          const rt_uint8_t opcode,
                          rt_uint32_t send_data)
{
    rt_uint8_t send_buf[4];
    rt_uint8_t* ptr = (rt_uint8_t*)&send_data;
    rt_int8_t i = 3;
    for(; i>=0; i--) {
        send_buf[i] = *ptr;
        ptr++;
    }
    rt_spi_send_then_send(tdc_gp21->spi_dev,
                          &opcode, 1,
                          &send_buf, 4);
}

static void tdc_gp21_error_print(struct spi_tdc_gp21* tdc_gp21,
                                 const rt_uint16_t stat)
{
    if(stat & GP21_ERR_MASK_TDC_TIMEOUT) {
			  rt_kprintf("0x%4x:%s TDC overflow!\n", stat, tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_PRE_CNT_TIMEOUT) {
        rt_kprintf("0x%4x:%s 14 bit precounter in MR2 overflow!\n", stat, tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_TEMP_SENSER_OPEN) {
        rt_kprintf("0x%4x:%s temperature sensor open!\n", stat, tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_TEMP_SENSER_SHORT) {
        rt_kprintf("0x%4x:%s temperature sensor short!\n", stat, tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_EEPROM_ERR) {
        rt_kprintf("0x%4x:%s has single error in EEPROM and been corrected!\n", stat, tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_EEPROM_ERRS) {
        rt_kprintf("0x%4x:%s has multiple errors in EEPROM!\n", stat, tdc_gp21->parent.parent.name);
    }
}

static rt_bool_t
tdc_gp21_check_id(struct spi_tdc_gp21* tdc_gp21, const rt_uint32_t id_l, const rt_uint32_t id_h)
{
    rt_uint8_t tmp[8] = {0};
    const rt_uint8_t opcode = GP21_READ_ID;

    rt_spi_send_then_recv(tdc_gp21->spi_dev,
                          &opcode, 1,
                          tmp, 7);
    if((id_l == *(rt_uint32_t*)tmp) &&
       (id_h == *(rt_uint32_t*)(tmp+4))) {
        return RT_TRUE;
    }
    else {
        return RT_FALSE;
    }

}

rt_inline void
tdc_gp21_busy_wait(struct spi_tdc_gp21* tdc_gp21)
{
    while(Bit_RESET!=GPIO_ReadInputDataBit(TDC_GPIO_IT_PIN_GROUP, TDC_GPIO_IT_PIN)) {
        /* set system into sleep mode until interrupt occered on gp21 initpin */
        //PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
        rt_thread_delay(1);
    }
}

static void
tdc_gp21_write_cmd(struct spi_tdc_gp21* tdc_gp21, rt_uint8_t opcode)
{

    /* some cmd need wait interrupt pin */
    if((opcode == GP21_WRITE_CFG_TO_EEPROM)  ||
       (opcode == GP21_WRITE_EEPROM_TO_CFG)  ||
       (opcode == GP21_COMPARE_EEPROM_CFG)   ||
       (opcode == GP21_START_TOF)            ||
       (opcode == GP21_START_TEMP)           ||
       (opcode == GP21_START_CAL_RESONATOR)  ||
       (opcode == GP21_START_TOF_RESTART)    ||
       (opcode == GP21_START_TEMP_RESTART)) {
        //tdc_gp21->busy = RT_TRUE;
        rt_spi_send(tdc_gp21->spi_dev, &opcode, 1);
        tdc_gp21_busy_wait(tdc_gp21);
    }
    else {
        rt_spi_send(tdc_gp21->spi_dev, &opcode, 1);
    }
}

rt_inline void tdc_gp21_reset_wait(void)
{
    rt_thread_delay(5);
}

rt_inline void tdc_gp21_reset(struct spi_tdc_gp21* tdc_gp21)
{
    GPIO_WriteBit(TDC_GPIO_NRST_PIN_GROUP, TDC_GPIO_NRST_PIN, Bit_RESET);
    tdc_gp21_reset_wait();
    GPIO_WriteBit(TDC_GPIO_NRST_PIN_GROUP, TDC_GPIO_NRST_PIN, Bit_SET);
    tdc_gp21_reset_wait();
    tdc_gp21_write_cmd(tdc_gp21, GP21_POWER_ON_RESET);
    tdc_gp21_reset_wait();
}



static void
tdc_gp21_measure_temp(struct spi_tdc_gp21* tdc_gp21,
                      struct spi_tdc_gp21_temp_data* args)
{
    rt_uint16_t stat = 0;
    rt_uint32_t res[2][4] = {0,0,0,0};

    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & GP21_ERR_ERRORS) {
        tdc_gp21_error_print(tdc_gp21, stat);
        tdc_gp21_write_cmd(tdc_gp21, GP21_INITIATE_TDC);
    }
    tdc_gp21_write_cmd(tdc_gp21, GP21_START_TEMP_RESTART);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & GP21_ERR_ERRORS) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    res[0][0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[0][1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[0][2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[0][3] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    /* wait for next measure finish */
    tdc_gp21_busy_wait(tdc_gp21);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    res[1][0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1][1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[1][2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[1][3] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    /* ust PT1000, referance resistance is 1k, gain error correction is 0.9931(PT1000,3.0v)
       k = 1000.0/(0.9940*2) = 503.0181
       but, reference to real heat meter, set k = 
    */
    args->hot  = 503.0181 *
                 (((float)res[0][0]/(float)res[0][2]) + ((float)res[1][0]/(float)res[1][2]));
    args->cold = 503.0181 *
                 (((float)res[0][1]/(float)res[0][3]) + ((float)res[1][1]/(float)res[1][3]));
}

rt_inline void
tdc_gp21_wait_for_alu(void)
{
    /* ALU calculate need no more than 4.6us */
    tdc_gp21_us_delay(5);

}

static void
tdc_gp21_measure_tof2(struct spi_tdc_gp21* tdc_gp21,
                      struct spi_tdc_gp21_tof_data* args)
{
    rt_uint16_t stat = 0;
    rt_uint32_t res[2][3] = {0,0,0};

    tdc_gp21_write_cmd(tdc_gp21, GP21_INITIATE_TDC);
    tdc_gp21_write_cmd(tdc_gp21, GP21_START_TOF_RESTART);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & GP21_ERR_ERRORS) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    /* ALU will automatically cal STOP1-START and store answer to res0 */
    /* ALU cal STOP2-START and store answer to res1 by send command below */
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER,
                              ((GP21_CONFIG_VALUE_REG1&(~GP21_CONFIG_VALUE_HIT2_MASK))|
                               GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_2));
    tdc_gp21_wait_for_alu();
    /* ALU cal STOP3-START and store answer to res2 by send command below */
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER,
                              ((GP21_CONFIG_VALUE_REG1&(~GP21_CONFIG_VALUE_HIT2_MASK))|
                               GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_3));
    tdc_gp21_wait_for_alu();
    res[0][0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[0][1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[0][2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);

    /* wait for next measure finish */
    //todo: whether should set reg1 to cal stop1_1?
    tdc_gp21_busy_wait(tdc_gp21);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & GP21_ERR_ERRORS) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER,
                              ((GP21_CONFIG_VALUE_REG1&(~GP21_CONFIG_VALUE_HIT2_MASK))|
                               GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_2));
    tdc_gp21_wait_for_alu();
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER,
                              ((GP21_CONFIG_VALUE_REG1&(~GP21_CONFIG_VALUE_HIT2_MASK))|
                               GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_3));
    tdc_gp21_wait_for_alu();
    res[1][0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1][1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[1][2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);

    /* cal up & down tof time
       as we set DIV_CLKHS=0 (REG0 20 21 bits) DIV_FIRE=3 CLKHS = tdc_gp21->Hzref_4M MHz
       so ffireclk1 = CLKHS / 2^DIV_CLKHS = 4M
       ffireclk2 = ffireclk1 * 2 / (DIV_FIRE+1) = 2M
       ffire = ffireclk2 / 2 = 1M
       TODO: should I minus 1/1M ?

       time = CLKHS/2^DIV_CLKHS * res
    */
    args->up = tdc_gp21_reg_to_double((res[0][0]+res[0][1]+res[0][2])/3)/tdc_gp21->Hzref_4M;//us
    args->down = tdc_gp21_reg_to_double((res[1][0]+res[1][1]+res[1][2])/3)/tdc_gp21->Hzref_4M;//us
}

static rt_err_t
tdc_gp21_init(rt_device_t dev)
{
    struct spi_tdc_gp21* tdc_gp21 = (struct spi_tdc_gp21*)dev;
    GPIO_InitTypeDef TDC_GPIO;
    //EXTI_InitTypeDef TDC_EXTI;
    //NVIC_InitTypeDef TDC_NVIC;
    rt_uint32_t reg = 0;
    RT_ASSERT(dev != RT_NULL);

    /* parameter init */
    //tdc_gp21->busy = RT_FALSE;

    /* set tdc interrupt pin */
    tdc_gp21->intpin.GPIOx = TDC_GPIO_IT_PIN_GROUP;
    tdc_gp21->intpin.GPIO_Pin = TDC_GPIO_IT_PIN;

    RCC_AHBPeriphClockCmd(TDC_GPIO_IT_PIN_RCC |
                          TDC_GPIO_NRST_PIN_RCC |
                          TDC_GPIO_EN_STOP1_PIN_RCC |
                          TDC_GPIO_EN_STOP2_PIN_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_StructInit(&TDC_GPIO);
    TDC_GPIO.GPIO_Pin = TDC_GPIO_IT_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_IN;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(tdc_gp21->intpin.GPIOx, &TDC_GPIO);
    TDC_GPIO.GPIO_Pin = TDC_GPIO_NRST_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(TDC_GPIO_NRST_PIN_GROUP, &TDC_GPIO);
		TDC_GPIO.GPIO_Pin = TDC_GPIO_EN_STOP1_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(TDC_GPIO_EN_STOP1_PIN_GROUP, &TDC_GPIO);
		GPIO_SetBits(TDC_GPIO_EN_STOP1_PIN_GROUP, TDC_GPIO_EN_STOP1_PIN);
		TDC_GPIO.GPIO_Pin = TDC_GPIO_EN_STOP2_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(TDC_GPIO_EN_STOP2_PIN_GROUP, &TDC_GPIO);
		GPIO_SetBits(TDC_GPIO_EN_STOP2_PIN_GROUP, TDC_GPIO_EN_STOP2_PIN);
    //SYSCFG_EXTILineConfig(TDC_GPIO_IT_PIN_PORT_SOURCE, TDC_GPIO_IT_PIN_PIN_SOURCE);
    //TDC_EXTI.EXTI_Line = TDC_GPIO_IT_EXTI_LINE;
    //TDC_EXTI.EXTI_LineCmd = ENABLE;
    //TDC_EXTI.EXTI_Mode = EXTI_Mode_Interrupt;
    //TDC_EXTI.EXTI_Trigger = EXTI_Trigger_Falling;
    //EXTI_Init(&TDC_EXTI);
    //TDC_NVIC.NVIC_IRQChannel = TDC_GPIO_IT_EXTI_IRQN;
    //TDC_NVIC.NVIC_IRQChannelCmd = ENABLE;
    //TDC_NVIC.NVIC_IRQChannelPriority = 3;
    //NVIC_Init(&TDC_NVIC);

    /* config GP21 */
    tdc_gp21_reset(tdc_gp21);
    tdc_gp21_write_cmd(tdc_gp21, GP21_WRITE_EEPROM_TO_CFG);
    /* check version */
    if(!tdc_gp21_check_id(tdc_gp21, GP21_CONFIG_VALUE_ID_L, GP21_CONFIG_VALUE_ID_H)) 
    {
        rt_uint16_t stat = 0;
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG0_REGISTER, GP21_CONFIG_VALUE_REG0);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG1_REGISTER, GP21_CONFIG_VALUE_REG1);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG2_REGISTER, GP21_CONFIG_VALUE_REG2);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG3_REGISTER, GP21_CONFIG_VALUE_REG3);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG4_REGISTER, GP21_CONFIG_VALUE_REG4);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG5_REGISTER, GP21_CONFIG_VALUE_REG5);
        tdc_gp21_write_register32(tdc_gp21, GP21_WRITE_REG6_REGISTER, GP21_CONFIG_VALUE_REG6);
        /*
            write config registers to eeprom
        */
        tdc_gp21_write_cmd(tdc_gp21, GP21_WRITE_CFG_TO_EEPROM);
        tdc_gp21_write_cmd(tdc_gp21, GP21_COMPARE_EEPROM_CFG);
        stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
        if(!(stat&0x8000)) {
            rt_kprintf("%s TDC EEPROM write failed!\n", tdc_gp21->parent.parent.name);
            return RT_ERROR;
        }
    }

    /* set ref clk.
       res = (2*(2^ANZ_PER_CALRES/32768))/(1/4000000)
       as we set ANZ_PER_CALRES = 0(RES0 22 23 bits)
       correct value = 244.140625.
       so, calibrated Hzref_4M = res*(32768/2*10^6)
    */
    tdc_gp21_write_cmd(tdc_gp21, GP21_START_CAL_RESONATOR);
    reg = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    tdc_gp21->Hzref_4M = tdc_gp21_reg_to_double(reg)*0.016384;

    return RT_EOK;
}

static rt_err_t
tdc_gp21_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct spi_tdc_gp21_temp_data temp;
    struct spi_tdc_gp21_tof_data tof;
    RT_ASSERT(dev != RT_NULL);

    tdc_gp21_measure_temp((struct spi_tdc_gp21*)dev, &temp);
    tdc_gp21_measure_tof2((struct spi_tdc_gp21*)dev, &tof);

    return RT_EOK;
}

static rt_err_t
tdc_gp21_close(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_err_t
tdc_gp21_control(rt_device_t dev, rt_uint8_t cmd, void* args)
{
    struct spi_tdc_gp21* tdc_gp21 = (struct spi_tdc_gp21*)dev;
    RT_ASSERT(dev != RT_NULL);

    if(!(dev->open_flag & RT_DEVICE_OFLAG_OPEN)) {
        TDC_TRACE("TDC device %d does not open!\r\n", dev->device_id);
        return -RT_EIO;
    }

    switch(cmd) {
        case SPI_TDC_GP21_CTRL_MEASURE_TEMP:
            tdc_gp21_measure_temp(tdc_gp21, (struct spi_tdc_gp21_temp_data*)args);
            break;
        case SPI_TDC_GP21_CTRL_MEASURE_TOF2:
            tdc_gp21_measure_tof2(tdc_gp21, (struct spi_tdc_gp21_tof_data*)args);
            break;
        default:
            TDC_TRACE("TDC gp21 control cmd 0x%02x does not support!\r\n", cmd);
    }

    return RT_EOK;
}

#ifdef RT_USING_TDC_GP21
static struct spi_tdc_gp21 tdc_gp21;

#endif /* RT_USING_TDC_GP21 */

/*
    global functions
*/
rt_err_t
tdc_gp21_register(const char* tdc_device_name, const char* spi_device_name)
{
    struct rt_spi_device* spi_dev = RT_NULL;
    rt_err_t ret = RT_EOK;

    spi_dev = (struct rt_spi_device*)rt_device_find(spi_device_name);
    if(RT_NULL == spi_dev) {
        TDC_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    tdc_gp21.spi_dev = spi_dev;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
        cfg.max_hz = 2000000;
        ret = rt_spi_configure(tdc_gp21.spi_dev, &cfg);
        if(RT_EOK != ret) {
            return ret;
        }
    }

    /* 3.register device */
    tdc_gp21.parent.type        = RT_Device_Class_Miscellaneous;
    tdc_gp21.parent.init        = tdc_gp21_init;
    tdc_gp21.parent.open        = tdc_gp21_open;
    tdc_gp21.parent.close       = tdc_gp21_close;
    tdc_gp21.parent.control     = tdc_gp21_control;
    tdc_gp21.parent.user_data   = RT_NULL;
    ret = rt_device_register(&tdc_gp21.parent, tdc_device_name, RT_NULL);
    if(RT_EOK != ret) {
        return ret;
    }

    return RT_EOK;
}
#if 0
#ifdef RT_USING_TDC_GP21
void
EXTI0_1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if(EXTI_GetITStatus(TDC_GPIO_IT_EXTI_LINE) != RESET) {
        //tdc_gp21.busy = RT_FALSE;
        EXTI_ClearFlag(TDC_GPIO_IT_EXTI_LINE);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_TDC_GP21 */
#endif

