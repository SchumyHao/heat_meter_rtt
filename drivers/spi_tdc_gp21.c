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
#define TDC_DEBUG

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
#define TDC_GPIO_NSS_PIN                    GPIO_Pin_10
#define TDC_GPIO_NSS_PIN_GROUP              GPIOB
#define TDC_GPIO_NSS_PIN_RCC                RCC_AHBPeriph_GPIOB

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

/* config value */
#define GP21_CONFIG_VALUE_ID_H              (0x00345678U)
#define GP21_CONFIG_VALUE_ID_L              (0x12345678U)
#define GP21_CONFIG_VALUE_STOPMASK_DELAY_US (200)
#define GP21_CONFIG_VALUE_ANZ_FIRE          (10)
#define GP21_CONFIG_VALUE_FIRE_DIV          (3)
/*
    config register0 :
    ANZ_FIRE = 31        DIV_FIRE = 3      ANZ_PER_CALRES = 3
    DIV_CLKHS = 0        START_CLKHS = 2   ANZ_PORT = 1
    TCYCLE = 1           ANZ_FAKE = 0      SEL_ECLK_TMP = 1
    CALIBRATE = 1        NO_CAL_AUTO = 0   MESSB2 = 1
    NEG_STOP/NEGSTART=0  ID0 = 0x10
*/
#define GP21_CONFIG_VALUE_ANZ_FIRE_L(v)     (((v)&0x0F)<<28)
#define GP21_CONFIG_VALUE_DIV_FIRE(v)       (((v)&0x0F)<<24)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_MASK (3<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_2  (0<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_4  (1<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_8  (2<<22)
#define GP21_CONFIG_VALUE_ANZ_PER_CALRES_16 (3<<22)
#define GP21_CONFIG_VALUE_DIV_CLKHS_MASK    (3<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_1       (0<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_2       (1<<20)
#define GP21_CONFIG_VALUE_DIV_CLKHS_4       (2<<20)
#define GP21_CONFIG_VALUE_START_CLKHS_L(v)  (((v)&0x03)<<18)
#define GP21_CONFIG_VALUE_START_CLKHS_OFF   (0)
#define GP21_CONFIG_VALUE_START_CLKHS_ON    (1)
#define GP21_CONFIG_VALUE_START_CLKHS_480US (2)
#define GP21_CONFIG_VALUE_START_CLKHS_1MS   (3)
#define GP21_CONFIG_VALUE_START_CLKHS_2MS   (4)
#define GP21_CONFIG_VALUE_START_CLKHS_5MS   (5)
#define GP21_CONFIG_VALUE_ANZ_PORT_MASK     (1<<17)
#define GP21_CONFIG_VALUE_ANZ_PORT_2        (0<<17)
#define GP21_CONFIG_VALUE_ANZ_PORT_4        (1<<17)
#define GP21_CONFIG_VALUE_TCYCLE_MASK       (1<<16)
#define GP21_CONFIG_VALUE_TCYCLE_128US      (0<<16)
#define GP21_CONFIG_VALUE_TCYCLE_512US      (1<<16)
#define GP21_CONFIG_VALUE_ANZ_FAKE_MASK     (1<<15)
#define GP21_CONFIG_VALUE_ANZ_FAKE_2        (0<<15)
#define GP21_CONFIG_VALUE_ANZ_FAKE_7        (1<<15)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_MASK (1<<14)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_L    (0<<14)
#define GP21_CONFIG_VALUE_SEL_ECLK_TMP_H    (1<<14)
#define GP21_CONFIG_VALUE_CALIBRATE_MASK    (1<<13)
#define GP21_CONFIG_VALUE_CALIBRATE_OFF     (0<<13)
#define GP21_CONFIG_VALUE_CALIBRATE_ON      (1<<13)
#define GP21_CONFIG_VALUE_NO_CAL_AUTO_MASK  (1<<12)
#define GP21_CONFIG_VALUE_CAL_AUTO_ON       (0<<12)
#define GP21_CONFIG_VALUE_CAL_AUTO_OFF      (1<<12)
#define GP21_CONFIG_VALUE_MRANGE_MASK       (1<<11)
#define GP21_CONFIG_VALUE_MRANGE_1          (0<<11)
#define GP21_CONFIG_VALUE_MRANGE_2          (1<<11)
#define GP21_CONFIG_VALUE_STOP2_EDGE_MASK   (1<<10)
#define GP21_CONFIG_VALUE_STOP2_EDGE_RISE   (0<<10)
#define GP21_CONFIG_VALUE_STOP2_EDGE_FALL   (1<<10)
#define GP21_CONFIG_VALUE_STOP1_EDGE_MASK   (1<<9)
#define GP21_CONFIG_VALUE_STOP1_EDGE_RISE   (0<<9)
#define GP21_CONFIG_VALUE_STOP1_EDGE_FALL   (1<<9)
#define GP21_CONFIG_VALUE_START_EDGE_MASK   (1<<8)
#define GP21_CONFIG_VALUE_START_EDGE_RISE   (0<<8)
#define GP21_CONFIG_VALUE_START_EDGE_FALL   (1<<8)
#define GP21_CONFIG_VALUE_ID0(v)            (((v)>>0)&0xFF)

/*
    config register1 :
    HIT2 = 2             HIT1 = 1          EN_FAST_INIT = 0
    HITIN2 = 0           HITIN1 = 4        CURR32K = 0
    SEL_START_FIRE = 1   SEL_TSTO2 = 2     SEL_TSTO1 = 7
    ID1 = 0x12
*/
#define GP21_CONFIG_VALUE_HIT2_MASK         (15<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_1 (2<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_2 (3<<28)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_3 (4<<28)
#define GP21_CONFIG_VALUE_HIT1_MASK         (15<<24)
#define GP21_CONFIG_VALUE_HIT1_MR2_CH_START (1<<24)
#define GP21_CONFIG_VALUE_FAST_INIT_MASK    (3<<23)
#define GP21_CONFIG_VALUE_FAST_INIT_OFF     (1<<23)
#define GP21_CONFIG_VALUE_FAST_INIT_ON      (3<<23)
#define GP21_CONFIG_VALUE_HITIN_SP2_MASK    (7<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_OFF     (0<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_1       (1<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_2       (2<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_3       (3<<19)
#define GP21_CONFIG_VALUE_HITIN_SP2_4       (4<<19)
#define GP21_CONFIG_VALUE_HITIN_SP1_MASK    (7<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_OFF     (0<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_1       (1<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_2       (2<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_3       (3<<16)
#define GP21_CONFIG_VALUE_HITIN_SP1_4       (4<<16)
#define GP21_CONFIG_VALUE_CURR32_MASK       (1<<15)
#define GP21_CONFIG_VALUE_CURR32_L          (0<<15)
#define GP21_CONFIG_VALUE_CURR32_H          (1<<15)
#define GP21_CONFIG_VALUE_SEL_FIRE_MASK     (1<<14)
#define GP21_CONFIG_VALUE_SEL_FIRE_OUT      (0<<14)
#define GP21_CONFIG_VALUE_SEL_FIRE_INTER    (1<<14)
#define GP21_CONFIG_VALUE_ENSTART_FN_MASK   (7<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_IN     (0<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_START  (1<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_SP1    (2<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_SP2    (3<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_TEMP_SP (4<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_TOF_ST (5<<11)
#define GP21_CONFIG_VALUE_ENSTART_FN_4KHZ   (7<<11)
#define GP21_CONFIG_VALUE_FIREIN_FN_MASK    (7<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_IN      (0<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_START   (1<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_SP1     (2<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_SP2     (3<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_TEMP_SP (4<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_ENSP    (5<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_COMP    (6<<8)
#define GP21_CONFIG_VALUE_FIREIN_FN_32KHZ   (7<<8)
#define GP21_CONFIG_VALUE_ID1(v)            (((v)>>8)&0xFF)

/*
    config register2 :
    EN_INT = B1111       RFEDGE1 = 0       RFEDGE2 = 0
    DELVAL1 = 400        ID2 = 0x12
*/
#define GP21_CONFIG_VALUE_TIMEOUT_INT_MASK  (1<<31)
#define GP21_CONFIG_VALUE_TIMEOUT_INT_ON    (1<<31)
#define GP21_CONFIG_VALUE_TIMEOUT_INT_OFF   (0<<31)
#define GP21_CONFIG_VALUE_HITEND_INT_MASK   (1<<30)
#define GP21_CONFIG_VALUE_HITEND_INT_ON     (1<<30)
#define GP21_CONFIG_VALUE_HITEND_INT_OFF    (0<<30)
#define GP21_CONFIG_VALUE_ALU_INT_MASK      (1<<29)
#define GP21_CONFIG_VALUE_ALU_INT_ON        (1<<29)
#define GP21_CONFIG_VALUE_ALU_INT_OFF       (0<<29)
#define GP21_CONFIG_VALUE_CH2_EDGE_MASK     (1<<28)
#define GP21_CONFIG_VALUE_CH2_EDGE_BOTH     (1<<28)
#define GP21_CONFIG_VALUE_CH2_EDGE_ONE      (0<<28)
#define GP21_CONFIG_VALUE_CH1_EDGE_MASK     (1<<27)
#define GP21_CONFIG_VALUE_CH1_EDGE_BOTH     (1<<27)
#define GP21_CONFIG_VALUE_CH1_EDGE_ONE      (0<<27)
#define GP21_CONFIG_VALUE_DELVAL1_MASK      (0x07FFFF<<8)
#define GP21_CONFIG_VALUE_DELVAL1(v)        ((v)<<(7-(GP21_CONFIG_VALUE_DIV_CLKHS_2>>20)+8))
#define GP21_CONFIG_VALUE_ID2(v)            (((v)>>16)&0xFF)

/*
    config register3 :
    EN_ERR_VAL = 0       SEL_TIMO_MB2 = 1  DELVAL2 = 408
    ID3 = 0x34
*/
#define GP21_CONFIG_VALUE_EN_ERR_VAL_MASK   (1<<29)
#define GP21_CONFIG_VALUE_EN_ERR_VAL_OFF    (0<<29)
#define GP21_CONFIG_VALUE_EN_ERR_VAL_ON     (1<<29)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_MASK (3<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_64   (0<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_256  (1<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_1024 (2<<27)
#define GP21_CONFIG_VALUE_SEL_TIMO_MB2_4096 (3<<27)
#define GP21_CONFIG_VALUE_DELVAL2_MASK      (0x07FFFF<<8)
#define GP21_CONFIG_VALUE_DELVAL2(v)        ((GP21_CONFIG_VALUE_DELVAL1(v))+ \
        (4<<(5+(GP21_CONFIG_VALUE_DIV_CLKHS_2>>20)+8)))
#define GP21_CONFIG_VALUE_ID3(v)            (((v)>>24)&0xFF)

/*
    config register4 :
    DELVAL3 = 416        ID4 = 0x56
*/
#define GP21_CONFIG_VALUE_DELVAL3_MASK      (0x07FFFF<<8)
#define GP21_CONFIG_VALUE_DELVAL3(v)        ((GP21_CONFIG_VALUE_DELVAL2(v))+ \
        (4<<(5+(GP21_CONFIG_VALUE_DIV_CLKHS_2>>20)+8)))
#define GP21_CONFIG_VALUE_ID4(v)            (((v)>>0)&0xFF)

/*
    config register5 :
    CON_FIRE = 0         EN_STARTNOISE = 0 DIS_PHASESHIFT = 0
    REPEAT_FIRE = 0      PHASE_FIRE = 0    ID5 = 0x78
*/
#define GP21_CONFIG_VALUE_CONF_FIRE_MASK    (7<<29)
#define GP21_CONFIG_VALUE_CONF_FIRE_BOTH    (1<<31)
#define GP21_CONFIG_VALUE_CONF_FIRE_EN_UP   (1<<30)
#define GP21_CONFIG_VALUE_CONF_FIRE_EN_DOWN (1<<29)
#define GP21_CONFIG_VALUE_STARTNOISE_MASK   (1<<28)
#define GP21_CONFIG_VALUE_STARTNOISE_ON     (1<<28)
#define GP21_CONFIG_VALUE_STARTNOISE_OFF    (0<<28)
#define GP21_CONFIG_VALUE_DIS_PH_NOISE_MASK (1<<27)
#define GP21_CONFIG_VALUE_PH_NOISE_ON       (0<<27)
#define GP21_CONFIG_VALUE_PH_NOISE_OFF      (1<<27)
#define GP21_CONFIG_VALUE_REPEAT_FIRE_MASK  (7<<24)
#define GP21_CONFIG_VALUE_REPEAT_FIRE_TIMES(v) (((v)&0x07)<<24)
#define GP21_CONFIG_VALUE_PH_FIRE_MASK      (0xFFFF<<8)
#define GP21_CONFIG_VALUE_PH_FIRE_INV(ch)   (1<<(8+(ch)))
#define GP21_CONFIG_VALUE_ID5(v)            (((v)>>8)&0xFF)

/*
    config register6 :
    EN_ANALOG = 1        NEG_STOP_TEMP = 1 TW2 = 1
    EN_INT = b1111       START_CLKHS = 2   CYCLE_TEMP = 1
    CYCLE_TOF = 0        HZ60 = 0          FIREO_DEF = 1
    QUAD_RES = 0         DOUBLE_RES = 1    TEMP_PORTDIR = 0
    ANZ_FIRE = 31        ID6 = 0x90
*/
#define GP21_CONFIG_VALUE_EN_ANALOG_MASK    (1<<31)
#define GP21_CONFIG_VALUE_ANALOG_ON         (1<<31)
#define GP21_CONFIG_VALUE_ANALOG_OFF        (0<<31)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_MASK (1<<30)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_EX  (0<<30)
#define GP21_CONFIG_VALUE_NEG_STOP_TEMP_INTER (1<<30)
#define GP21_CONFIG_VALUE_COMP_OFFSET_MASK  (15<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_0MV   (0<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_1MV   (1<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_2MV   (2<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_3MV   (3<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_4MV   (4<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_5MV   (5<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_6MV   (6<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_7MV   (7<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M1MV  (15<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M2MV  (14<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M3MV  (13<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M4MV  (12<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M5MV  (11<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M6MV  (10<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M7MV  (9<<25)
#define GP21_CONFIG_VALUE_COMP_OFFSET_M8MV  (8<<25)
#define GP21_CONFIG_VALUE_RC_TIME_MASK      (3<<22)
#define GP21_CONFIG_VALUE_RC_TIME_90US      (0<<22)
#define GP21_CONFIG_VALUE_RC_TIME_120US     (1<<22)
#define GP21_CONFIG_VALUE_RC_TIME_150US     (2<<22)
#define GP21_CONFIG_VALUE_RC_TIME_300US     (3<<22)
#define GP21_CONFIG_VALUE_EEPROM_INT_MASK   (1<<21)
#define GP21_CONFIG_VALUE_EEPROM_INT_ON     (1<<21)
#define GP21_CONFIG_VALUE_EEPROM_INT_OFF    (0<<21)
#define GP21_CONFIG_VALUE_START_CLKHS_H(v)  ((((v)>>2)&0x01)<<20)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_MASK   (3<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_1      (0<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_1_5    (1<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_2      (2<<18)
#define GP21_CONFIG_VALUE_CYCLE_TEMP_2_5    (3<<18)
#define GP21_CONFIG_VALUE_CYCLE_TOF_MASK    (3<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_1       (0<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_1_5     (1<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_2       (2<<16)
#define GP21_CONFIG_VALUE_CYCLE_TOF_2_5     (3<<16)
#define GP21_CONFIG_VALUE_HZ60_MASK         (1<<15)
#define GP21_CONFIG_VALUE_HZ60              (1<<15)
#define GP21_CONFIG_VALUE_HZ50              (0<<15)
#define GP21_CONFIG_VALUE_FIREO_DEF_MASK    (1<<14)
#define GP21_CONFIG_VALUE_FIREO_HZ          (0<<14)
#define GP21_CONFIG_VALUE_FIREO_L           (1<<14)
#define GP21_CONFIG_VALUE_QUAD_RES_MASK     (1<<13)
#define GP21_CONFIG_VALUE_QUAD_RES_ON       (1<<13)
#define GP21_CONFIG_VALUE_QUAD_RES_OFF      (0<<13)
#define GP21_CONFIG_VALUE_DOUBLE_RES_MASK   (1<<12)
#define GP21_CONFIG_VALUE_DOUBLE_RES_ON     (1<<12)
#define GP21_CONFIG_VALUE_DOUBLE_RES_OFF    (0<<12)
#define GP21_CONFIG_VALUE_TEMP_PORTDIR_MASK (1<<11)
#define GP21_CONFIG_VALUE_TEMP_PORT_1_4     (0<<11)
#define GP21_CONFIG_VALUE_TEMP_PORT_4_1     (1<<11)
#define GP21_CONFIG_VALUE_ANZ_FIRE_H(v)     ((((v)>>4)&0x07)<<8)
#define GP21_CONFIG_VALUE_ID6(v)            (((v)>>16)&0xFF)

#define GP21_CONFIG_VALUE_REG0              (\
        GP21_CONFIG_VALUE_ANZ_FIRE_L(GP21_CONFIG_VALUE_ANZ_FIRE)|\
        GP21_CONFIG_VALUE_DIV_FIRE(GP21_CONFIG_VALUE_FIRE_DIV)|\
        GP21_CONFIG_VALUE_ANZ_PER_CALRES_2  |\
        GP21_CONFIG_VALUE_DIV_CLKHS_2       |\
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

#define GP21_CONFIG_VALUE_REG1              (\
        GP21_CONFIG_VALUE_HIT2_MR2_CH_SPCH1_1|\
        GP21_CONFIG_VALUE_HIT1_MR2_CH_START |\
        GP21_CONFIG_VALUE_FAST_INIT_OFF     |\
        GP21_CONFIG_VALUE_HITIN_SP2_OFF     |\
        GP21_CONFIG_VALUE_HITIN_SP1_4       |\
        GP21_CONFIG_VALUE_CURR32_L          |\
        GP21_CONFIG_VALUE_SEL_FIRE_INTER    |\
        GP21_CONFIG_VALUE_ENSTART_FN_4KHZ   |\
        GP21_CONFIG_VALUE_FIREIN_FN_32KHZ   |\
        GP21_CONFIG_VALUE_ID1(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG2              (\
        GP21_CONFIG_VALUE_TIMEOUT_INT_ON    |\
        GP21_CONFIG_VALUE_HITEND_INT_ON     |\
        GP21_CONFIG_VALUE_ALU_INT_ON        |\
        GP21_CONFIG_VALUE_CH2_EDGE_ONE      |\
        GP21_CONFIG_VALUE_CH1_EDGE_ONE      |\
        GP21_CONFIG_VALUE_DELVAL1(GP21_CONFIG_VALUE_STOPMASK_DELAY_US)|\
        GP21_CONFIG_VALUE_ID2(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG3              (\
        GP21_CONFIG_VALUE_EN_ERR_VAL_ON     |\
        GP21_CONFIG_VALUE_SEL_TIMO_MB2_4096 |\
        GP21_CONFIG_VALUE_DELVAL2(GP21_CONFIG_VALUE_STOPMASK_DELAY_US)|\
        GP21_CONFIG_VALUE_ID3(GP21_CONFIG_VALUE_ID_L))

#define GP21_CONFIG_VALUE_REG4              (\
        GP21_CONFIG_VALUE_DELVAL3(GP21_CONFIG_VALUE_STOPMASK_DELAY_US)|\
        GP21_CONFIG_VALUE_ID4(GP21_CONFIG_VALUE_ID_H))

#define GP21_CONFIG_VALUE_REG5              (\
        GP21_CONFIG_VALUE_CONF_FIRE_EN_UP   |\
        GP21_CONFIG_VALUE_STARTNOISE_OFF    |\
        GP21_CONFIG_VALUE_PH_NOISE_ON       |\
        GP21_CONFIG_VALUE_REPEAT_FIRE_TIMES(0)|\
        GP21_CONFIG_VALUE_ID5(GP21_CONFIG_VALUE_ID_H))

#define GP21_CONFIG_VALUE_REG6              (\
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
        GP21_CONFIG_VALUE_DOUBLE_RES_OFF    |\
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

rt_inline rt_uint16_t
tdc_gp21_read_register16(struct spi_tdc_gp21* tdc_gp21,
                         const rt_uint8_t opcode)
{
    rt_uint8_t recv_buf[2];
    rt_uint32_t recv_data = 0;
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
    rt_uint8_t* ptr = (rt_uint8_t*)&send_data;
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

void tdc_gp21_nss_init(struct stm32_spi_dev_cs* cs)
{
    GPIO_InitTypeDef TDC_GPIO;
    RT_ASSERT(cs != RT_NULL);

    RCC_AHBPeriphClockCmd(TDC_GPIO_NSS_PIN_RCC, ENABLE);
    GPIO_StructInit(&TDC_GPIO);
    TDC_GPIO.GPIO_Pin = TDC_GPIO_NSS_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TDC_GPIO_NSS_PIN_GROUP, &TDC_GPIO);
    GPIO_SetBits(TDC_GPIO_NSS_PIN_GROUP, TDC_GPIO_NSS_PIN);
}
void tdc_gp21_nss_take(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_ResetBits(TDC_GPIO_NSS_PIN_GROUP, TDC_GPIO_NSS_PIN);
}
void tdc_gp21_nss_release(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_SetBits(TDC_GPIO_NSS_PIN_GROUP, TDC_GPIO_NSS_PIN);
}

static void tdc_gp21_error_print(struct spi_tdc_gp21* tdc_gp21,
                                 const rt_uint16_t stat)
{
    if(stat & GP21_ERR_MASK_TDC_TIMEOUT) {
        rt_kprintf("%s TDC overflow!\n", tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_PRE_CNT_TIMEOUT) {
        rt_kprintf("%s 14 bit precounter in MR2 overflow!\n", tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_TEMP_SENSER_OPEN) {
        rt_kprintf("%s temperature sensor open!\n", tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_TEMP_SENSER_SHORT) {
        rt_kprintf("%s temperature sensor short!\n", tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_EEPROM_ERR) {
        rt_kprintf("%s has single error in EEPROM and been corrected!\n", tdc_gp21->parent.parent.name);
    }
    if(stat & GP21_ERR_MASK_EEPROM_ERRS) {
        rt_kprintf("%s has multiple errors in EEPROM!\n", tdc_gp21->parent.parent.name);
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
    tdc_gp21->busy = RT_TRUE;

    while(tdc_gp21->busy) {
        /* set system into sleep mode until interrupt occered on gp21 initpin */
        PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
    }
}

static void
tdc_gp21_write_cmd(struct spi_tdc_gp21* tdc_gp21, rt_uint8_t opcode)
{
    rt_spi_send(tdc_gp21->spi_dev, &opcode, 1);
    /* some cmd need wait interrupt pin */
    if((opcode == GP21_WRITE_CFG_TO_EEPROM)  ||
       (opcode == GP21_WRITE_EEPROM_TO_CFG)  ||
       (opcode == GP21_COMPARE_EEPROM_CFG)   ||
       (opcode == GP21_START_TOF)            ||
       (opcode == GP21_START_TEMP)           ||
       (opcode == GP21_START_CAL_RESONATOR)  ||
       (opcode == GP21_START_TOF_RESTART)    ||
       (opcode == GP21_START_TEMP_RESTART)) {
        tdc_gp21_busy_wait(tdc_gp21);
    }
}

rt_inline void tdc_gp21_reset_wait(void)
{
    rt_uint8_t tmp = 0xff;
    while(--tmp > 0);
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

static rt_err_t
tdc_gp21_init(rt_device_t dev)
{
    struct spi_tdc_gp21* tdc_gp21 = (struct spi_tdc_gp21*)dev;
    GPIO_InitTypeDef TDC_GPIO;
    EXTI_InitTypeDef TDC_EXTI;
    NVIC_InitTypeDef TDC_NVIC;
    RT_ASSERT(dev != RT_NULL);

    /* parameter init */
    tdc_gp21->busy = RT_FALSE;

    /* set tdc interrupt pin */
    tdc_gp21->intpin.GPIOx = TDC_GPIO_IT_PIN_GROUP;
    tdc_gp21->intpin.GPIO_Pin = TDC_GPIO_IT_PIN;

    RCC_AHBPeriphClockCmd(TDC_GPIO_IT_PIN_RCC |
                          TDC_GPIO_NRST_PIN_RCC,ENABLE);
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
    SYSCFG_EXTILineConfig(TDC_GPIO_IT_PIN_PORT_SOURCE, TDC_GPIO_IT_PIN_PIN_SOURCE);
    TDC_EXTI.EXTI_Line = TDC_GPIO_IT_EXTI_LINE;
    TDC_EXTI.EXTI_LineCmd = ENABLE;
    TDC_EXTI.EXTI_Mode = EXTI_Mode_Interrupt;
    TDC_EXTI.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&TDC_EXTI);
    TDC_NVIC.NVIC_IRQChannel = TDC_GPIO_IT_EXTI_IRQN;
    TDC_NVIC.NVIC_IRQChannelCmd = ENABLE;
    TDC_NVIC.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&TDC_NVIC);

    /* config GP21 */
    tdc_gp21_reset(tdc_gp21);
    tdc_gp21_write_cmd(tdc_gp21, GP21_WRITE_EEPROM_TO_CFG);
    /* check version */
    if(!tdc_gp21_check_id(tdc_gp21, GP21_CONFIG_VALUE_ID_L, GP21_CONFIG_VALUE_ID_H)) {
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
    }

    return RT_EOK;
}

static rt_err_t
tdc_gp21_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_uint32_t reg = 0;
    struct spi_tdc_gp21* tdc_gp21 = (struct spi_tdc_gp21*)dev;
    RT_ASSERT(dev != RT_NULL);

    /* set corr_factor */
    tdc_gp21_write_cmd(tdc_gp21, GP21_START_CAL_RESONATOR);
    reg = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    tdc_gp21->corr_factor = 488.28125/reg; //High speed corr_factor
    return RT_EOK;
}

static rt_err_t
tdc_gp21_close(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static void
tdc_gp21_measure_temp(struct spi_tdc_gp21* tdc_gp21,
                      struct spi_tdc_gp21_temp_scales* args)
{
    rt_uint16_t stat = 0;
    rt_uint32_t res[4] = {0,0,0,0};
    struct spi_tdc_gp21_temp_scales temp = {0,0};

    tdc_gp21_write_cmd(tdc_gp21, GP21_START_TEMP_RESTART);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    res[0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[3] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    temp.hot  = (float)res[0]/(float)res[2];
    temp.cold = (float)res[1]/(float)res[3];
    /* wait for next measure finish */
    tdc_gp21_busy_wait(tdc_gp21);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    res[0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[3] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    args->hot  = (temp.hot  + ((float)res[0]/(float)res[2]))/2;
    args->cold = (temp.cold + ((float)res[1]/(float)res[3]))/2;
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
    rt_uint32_t res[3] = {0,0,0};

    tdc_gp21_write_cmd(tdc_gp21, GP21_INITIATE_TDC);
    tdc_gp21_write_cmd(tdc_gp21, GP21_START_TOF_RESTART);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
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
    res[0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    args->up = (res[0]+res[1]+res[2])/3;

    /* wait for next measure finish */
	//todo: whether should set reg1 to cal stop1_1?
    tdc_gp21_busy_wait(tdc_gp21);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
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
    res[0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    args->down = (res[0]+res[1]+res[2])/3;
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
            tdc_gp21_measure_temp(tdc_gp21, (struct spi_tdc_gp21_temp_scales*)args);
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
static struct stm32_spi_dev_cs tdc_nss_pin = {
    tdc_gp21_nss_init,
    tdc_gp21_nss_take,
    tdc_gp21_nss_release
};
#endif /* RT_USING_TDC_GP21 */

/*
    global functions
*/
rt_err_t
tdc_gp21_register(const char* tdc_device_name, const char* spi_bus_name)
{
    struct rt_spi_configuration cfg;
    struct rt_spi_bus* spi_bus = RT_NULL;
    struct rt_spi_device* spi_dev = RT_NULL;
    rt_err_t ret = RT_EOK;

    /* 1. find spi bus */
    spi_bus = (struct rt_spi_bus*)rt_device_find(spi_bus_name);
    if(spi_bus == RT_NULL) {
        TDC_TRACE("spi bus %s not found!\r\n", spi_bus_name);
        return -RT_ENOSYS;
    }
    if(!(spi_bus->parent.open_flag & RT_DEVICE_OFLAG_OPEN)) {
        if(RT_EOK != rt_device_open(&spi_bus->parent, RT_DEVICE_OFLAG_RDWR)) {
            TDC_TRACE("spi bus %s open failed!\r\n", spi_bus_name);
            return -RT_ERROR;
        }
    }
    spi_dev = (struct rt_spi_device*)rt_malloc(sizeof(*spi_dev));
    RT_ASSERT(spi_dev != RT_NULL);
    if(RT_EOK != rt_spi_bus_attach_device(spi_dev, "spitdc", spi_bus->parent.parent.name, &tdc_nss_pin)) {
        TDC_TRACE("tdc spi device attach to spi bus %s failed!\r\n", spi_bus_name);
        return -RT_ERROR;
    }
    tdc_gp21.spi_dev = spi_dev;

    /* 2.config spi device */
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 2000000;
    ret = rt_spi_configure(tdc_gp21.spi_dev, &cfg);
    if(RT_EOK != ret) {
        return ret;
    }

    /* 3.register device */
    tdc_gp21.parent.type        = RT_Device_Class_Miscellaneous;
    tdc_gp21.parent.init        = tdc_gp21_init;
    tdc_gp21.parent.open        = tdc_gp21_open;
    tdc_gp21.parent.close       = tdc_gp21_close;
    tdc_gp21.parent.control     = tdc_gp21_control;
    tdc_gp21.parent.user_data   = RT_NULL;
    ret = rt_device_register(&(tdc_gp21.parent), "tdc1", RT_NULL);
    if(RT_EOK != ret) {
        return ret;
    }

    return RT_EOK;
}

#ifdef RT_USING_TDC_GP21
void
EXTI0_1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if(EXTI_GetITStatus(TDC_GPIO_IT_EXTI_LINE) != RESET) {
        tdc_gp21.busy = RT_FALSE;
        EXTI_ClearFlag(TDC_GPIO_IT_EXTI_LINE);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_TDC_GP21 */

