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
/*
    config register0 :
    ANZ_FIRE = 31        DIV_FIRE = 3      ANZ_PER_CALRES = 3
    DIV_CLKHS = 0        START_CLKHS = 2   ANZ_PORT = 1
    TCYCLE = 1           ANZ_FAKE = 0      SEL_ECLK_TMP = 1
    CALIBRATE = 1        NO_CAL_AUTO = 0   MESSB2 = 1
    NEG_STOP/NEGSTART=0  ID0 = 0x10
*/
#define GP21_CONFIG_VALUE_REG0              (0xF3CB6810)
/*
    config register1 :
    HIT2 = 2             HIT1 = 1          EN_FAST_INIT = 0
    HITIN2 = 0           HITIN1 = 4        CURR32K = 0
    SEL_START_FIRE = 1   SEL_TSTO2 = 2     SEL_TSTO1 = 7
    ID1 = 0x12
*/
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SP1   (0x20000000)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SP2   (0x30000000)
#define GP21_CONFIG_VALUE_HIT2_MR2_CH_SP3   (0x40000000)
#define GP21_CONFIG_VALUE_HIT2_MASK         (0xF0000000)
#define GP21_CONFIG_VALUE_HIT1_MR2_CH_START (0x01000000)
#define GP21_CONFIG_VALUE_HIT1_MASK         (0x0F000000)
#define GP21_CONFIG_VALUE_REG1              (\
        GP21_CONFIG_VALUE_HIT2_MR2_CH_SP1 |\
        GP21_CONFIG_VALUE_HIT1_MR2_CH_START |\
        0x00445712)
/*
    config register2 :
    EN_INT = B1111       RFEDGE1 = 0       RFEDGE2 = 0
    DELVAL1 = 400        ID2 = 0x12
*/
#define GP21_CONFIG_VALUE_REG2              (0xE0320012)
/*
    config register3 :
    EN_ERR_VAL = 0       SEL_TIMO_MB2 = 1  DELVAL2 = 408
    ID3 = 0x34
*/
#define GP21_CONFIG_VALUE_REG3              (0x08330034)
/*
    config register4 :
    DELVAL3 = 416        ID4 = 0x56
*/
#define GP21_CONFIG_VALUE_REG4              (0x20340056)
/*
    config register5 :
    CON_FIRE = 0         EN_STARTNOISE = 0 DIS_PHASESHIFT = 0
    REPEAT_FIRE = 0      PHASE_FIRE = 0    ID5 = 0x78
*/
#define GP21_CONFIG_VALUE_REG5              (0x00000078)
/*
    config register6 :
    EN_ANALOG = 1        NEG_STOP_TEMP = 1 TW2 = 1
    EN_INT = b1111       START_CLKHS = 2   CYCLE_TEMP = 1
    CYCLE_TOF = 0        HZ60 = 0          FIREO_DEF = 1
    QUAD_RES = 0         DOUBLE_RES = 1    TEMP_PORTDIR = 0
    ANZ_FIRE = 31        ID6 = 0x90
*/
#define GP21_CONFIG_VALUE_REG6              (0xC0645190)


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
tdc_gp21_check_id(struct spi_tdc_gp21* tdc_gp21, const rt_uint32_t id)
{
    rt_uint8_t tmp[8];
    const rt_uint8_t opcode = GP21_READ_ID;

    rt_spi_send_then_recv(tdc_gp21->spi_dev,
                          &opcode, 1,
                          tmp, 8);
    if(id == *(rt_uint32_t*)tmp) {
        return RT_TRUE;
    }
    else {
        return RT_FALSE;
    }

}

rt_inline void
tdc_gp21_busy_wait(struct spi_tdc_gp21* tdc_gp21)
{
    rt_uint16_t stat = 0;
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
    if(!tdc_gp21_check_id(tdc_gp21, 0x34120110)) {
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
    tdc_gp21->corr_factor = 488.28125/reg;
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
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x31400000);
    tdc_gp21_wait_for_alu();
    /* ALU cal STOP3-START and store answer to res2 by send command below */
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x41400000);
    tdc_gp21_wait_for_alu();
    res[0] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = tdc_gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    args->up = (res[0]+res[1]+res[2])/3;

    /* wait for next measure finish */
    tdc_gp21_busy_wait(tdc_gp21);
    stat = tdc_gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0xFE00) {
        tdc_gp21_error_print(tdc_gp21, stat);
        return;
    }
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x31400000);
    tdc_gp21_wait_for_alu();
    tdc_gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x41400000);
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
        if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
            TDC_TRACE("spi bus %s open failed!\r\n", spi_bus_name);
            return -RT_ERROR;
        }
    }
    spi_dev = (struct rt_spi_device*)rt_malloc(sizeof(*spi_dev));
    RT_ASSERT(spi_dev != RT_NULL);
    if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spitdc", spi_bus->parent.name, &tdc_nss_pin)) {
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

