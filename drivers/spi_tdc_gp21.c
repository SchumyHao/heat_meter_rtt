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

/*
    local define
*/
#define TDC_DEBUG

#ifdef TDC_DEBUG
#define TDC_TRACE       rt_kprintf
#else
#define TDC_TRACE(...)
#endif

#define TDC_GPIO_IT_PIN                     GPIO_Pin_4
#define TDC_GPIO_IT_PIN_RCC                 RCC_AHBPeriph_GPIOF
#define TDC_GPIO_IT_PIN_GROUP               GPIOF
#define TDC_GPIO_IT_PIN_PIN_SOURCE          EXTI_PinSource4
#define TDC_GPIO_IT_PIN_PORT_SOURCE         EXTI_PortSourceGPIOF
#define TDC_GPIO_IT_EXTI_LINE               EXTI_Line4
#define TDC_GPIO_IT_EXTI_IRQN               EXTI4_15_IRQn

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
#define GP21_COMPARE_EEPROM_CFG             (0xC5)
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

/*
    local functions
*/
rt_inline rt_uint16_t
gp21_read_register16(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode)
{
    rt_uint16_t recv_buf = 0;
    rt_spi_send_then_recv((struct rt_spi_device*)tdc_gp21,
                          &opcode, 1,
                          &recv_buf, 2);
    return recv_buf;
}

rt_inline rt_uint32_t
gp21_read_register32(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode)
{
    rt_uint32_t recv_buf = 0;
    rt_spi_send_then_recv((struct rt_spi_device*)tdc_gp21,
                          &opcode, 1,
                          &recv_buf, 4);
    return recv_buf;
}

rt_inline void
gp21_write_register16(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode,
                      uint16_t send_buf)
{
    rt_spi_send_then_send((struct rt_spi_device*)tdc_gp21,
                          &opcode, 1,
                          &send_buf, 2);
}

rt_inline void
gp21_write_register24(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode,
                      uint32_t send_buf)
{
    rt_spi_send_then_send((struct rt_spi_device*)tdc_gp21,
                          &opcode, 1,
                          &send_buf, 3);
}

rt_inline void
gp21_write_register32(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode,
                      uint32_t send_buf)
{
    rt_spi_send_then_send((struct rt_spi_device*)tdc_gp21,
                          &opcode, 1,
                          &send_buf, 4);
}

rt_inline void
busy_wait(struct spi_tdc_gp21* tdc_gp21)
{
    tdc_gp21->busy = RT_TRUE;
    while(tdc_gp21->busy) {
        /* set system into sleep mode until interrupt occered on gp21 initpin */
        PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
    }
}

static void
gp21_write_cmd(struct spi_tdc_gp21* tdc_gp21, uint8_t opcode)
{
    rt_spi_send((struct rt_spi_device*)tdc_gp21, &opcode, 1);
    /* some cmd need wait interrupt pin */
    if((opcode == GP21_WRITE_CFG_TO_EEPROM)  ||
       (opcode == GP21_WRITE_EEPROM_TO_CFG)  ||
       (opcode == GP21_COMPARE_EEPROM_CFG)   ||
       (opcode == GP21_START_TOF)            ||
       (opcode == GP21_START_TEMP)           ||
       (opcode == GP21_START_CAL_RESONATOR)  ||
       (opcode == GP21_START_TOF_RESTART)    ||
       (opcode == GP21_START_TEMP_RESTART)) {
        busy_wait(tdc_gp21);
    }
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

    RCC_AHBPeriphClockCmd(TDC_GPIO_IT_PIN_RCC, ENABLE);
    GPIO_StructInit(&TDC_GPIO);
    TDC_GPIO.GPIO_Pin = TDC_GPIO_IT_PIN;
    TDC_GPIO.GPIO_Mode = GPIO_Mode_IN;
    TDC_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    TDC_GPIO.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(tdc_gp21->intpin.GPIOx, &TDC_GPIO);
    SYSCFG_EXTILineConfig(TDC_GPIO_IT_PIN_PORT_SOURCE, TDC_GPIO_IT_PIN_PIN_SOURCE);
    TDC_EXTI.EXTI_Line = TDC_GPIO_IT_EXTI_LINE;
    TDC_EXTI.EXTI_LineCmd = ENABLE;
    TDC_EXTI.EXTI_Mode = EXTI_Mode_Interrupt;
    TDC_EXTI.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&TDC_EXTI);
    TDC_NVIC.NVIC_IRQChannel = TDC_GPIO_IT_EXTI_IRQN;
    TDC_NVIC.NVIC_IRQChannelCmd = ENABLE;
    TDC_NVIC.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&TDC_NVIC);

    /* config GP21 */
    /*
        config register0 :
        ANZ_FIRE = 31        DIV_FIRE = 3      ANZ_PER_CALRES = 3
        DIV_CLKHS = 0        START_CLKHS = 2   ANZ_PORT = 1
        TCYCLE = 1           ANZ_FAKE = 0      SEL_ECLK_TMP = 1
        CALIBRATE = 1        NO_CAL_AUTO = 0   MESSB2 = 1
        NEG_STOP/NEGSTART=0  ID0 = 0x10
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG0_REGISTER, 0xF3CB6810);
    /*
        config register1 :
        HIT1 = 2             HIT1 = 1          EN_FAST_INIT = 0
        HITIN2 = 0           HITIN1 = 4        CURR32K = 0
        SEL_START_FIRE = 1   SEL_TSTO2 = 2     SEL_TSTO1 = 7
        ID1 = 0x12
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x21445712);
    /*
        config register2 :
        EN_INT = B1101       RFEDGE1 = 0       RFEDGE2 = 0
        DELVAL1 = 400        ID2 = 0x12
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG2_REGISTER, 0xA0320012);
    /*
        config register3 :
        EN_ERR_VAL = 0       SEL_TIMO_MB2 = 1  DELVAL2 = 408
        ID3 = 0x34
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG3_REGISTER, 0x08330034);
    /*
        config register4 :
        DELVAL3 = 416        ID4 = 0x56
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG4_REGISTER, 0x20340056);
    /*
        config register5 :
        CON_FIRE = 0         EN_STARTNOISE = 0 DIS_PHASESHIFT = 0
        REPEAT_FIRE = 0      PHASE_FIRE = 0    ID5 = 0x78
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG5_REGISTER, 0x00000078);
    /*
        config register6 :
        EN_ANALOG = 1        NEG_STOP_TEMP = 1 TW2 = 1
        EN_INT = b1101       START_CLKHS = 2   CYCLE_TEMP = 1
        CYCLE_TOF = 0        HZ60 = 0          FIREO_DEF = 1
        QUAD_RES = 0         DOUBLE_RES = 1    TEMP_PORTDIR = 0
        ANZ_FIRE = 31        ID6 = 0x90
    */
    gp21_write_register32(tdc_gp21, GP21_WRITE_REG6_REGISTER, 0xC0645190);

    /*
        write config registers to eeprom
    */
    gp21_write_cmd(tdc_gp21, GP21_WRITE_CFG_TO_EEPROM);

    return RT_EOK;
}

static rt_err_t
tdc_gp21_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_uint32_t reg = 0;
    struct spi_tdc_gp21* tdc_gp21 = (struct spi_tdc_gp21*)dev;
    RT_ASSERT(dev != RT_NULL);

    /* set corr_factor */
    gp21_write_cmd(tdc_gp21, GP21_POWER_ON_RESET);
    gp21_write_cmd(tdc_gp21, GP21_WRITE_EEPROM_TO_CFG);
    gp21_write_cmd(tdc_gp21, GP21_START_CAL_RESONATOR);
    reg = gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
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
    uint16_t stat = 0;
    uint32_t res[4] = {0,0,0,0};
    struct spi_tdc_gp21_temp_scales temp = {0,0};

    gp21_write_cmd(tdc_gp21, GP21_START_TEMP_RESTART);
    stat = gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0x1E00) {
        TDC_TRACE("TDC gp21 measure temperature error=0x%04x !\r\n", stat);
        return;
    }
    res[0] = gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[3] = gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    temp.hot  = (float)res[0]/(float)res[1];
    temp.cold = (float)res[3]/(float)res[2];
    /* wait for next measure finish */
    busy_wait(tdc_gp21);
    stat = gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0x1E00) {
        TDC_TRACE("TDC gp21 measure temperature error=0x%04x !\r\n", stat);
        return;
    }
    res[0] = gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    res[3] = gp21_read_register32(tdc_gp21, GP21_READ_RES3_REGISTER);
    args->hot  = (temp.hot  + ((float)res[0]/(float)res[1]))/2;
    args->cold = (temp.cold + ((float)res[3]/(float)res[2]))/2;
}

rt_inline void
wait_for_alu(void)
{
    /* ALU calculate need no more than 4.6us */
    uint32_t i = 0x00FFFFFF;
    while(i--);
}

static void
tdc_gp21_measure_tof2(struct spi_tdc_gp21* tdc_gp21,
                      struct spi_tdc_gp21_tof_data* args)
{
    uint16_t stat = 0;
    uint32_t res[3] = {0,0,0};

    gp21_write_cmd(tdc_gp21, GP21_INITIATE_TDC);
    gp21_write_cmd(tdc_gp21, GP21_START_TOF_RESTART);
    stat = gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0x0600) {
        TDC_TRACE("TDC gp21 measure ToF error=0x%04x !\r\n", stat);
        return;
    }
    /* ALU will automatically cal STOP1-START and store answer to res0 */
    /* ALU cal STOP2-START and store answer to res1 by send command below */
    gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x31400000);
    wait_for_alu();
    /* ALU cal STOP3-START and store answer to res2 by send command below */
    gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x41400000);
    wait_for_alu();
    res[0] = gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
    args->up = (res[0]+res[1]+res[2])/3;

    /* wait for next measure finish */
    busy_wait(tdc_gp21);
    stat = gp21_read_register16(tdc_gp21, GP21_READ_STAT_REGISTER);
    if(stat & 0x0600) {
        TDC_TRACE("TDC gp21 measure ToF error=0x%04x !\r\n", stat);
        return;
    }
    gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x31400000);
    wait_for_alu();
    gp21_write_register24(tdc_gp21, GP21_WRITE_REG1_REGISTER, 0x41400000);
    wait_for_alu();
    res[0] = gp21_read_register32(tdc_gp21, GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(tdc_gp21, GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(tdc_gp21, GP21_READ_RES2_REGISTER);
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
#endif /* RT_USING_TDC_GP21 */

/*
    global functions
*/
rt_err_t
tdc_register(const char* tdc_device_name, const char* spi_dev_name)
{
    struct rt_spi_configuration cfg;
    struct rt_spi_device* spi_device = RT_NULL;
    rt_err_t ret = RT_EOK;

    /* 1. find spi device */
    spi_device = (struct rt_spi_device*)rt_device_find(spi_dev_name);
    if(spi_device == RT_NULL) {
        TDC_TRACE("spi device %s not found!\r\n", spi_dev_name);
        return -RT_ENOSYS;
    }
    tdc_gp21.spi_dev = spi_device;

    /* 2.config spi device */
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 20000000;
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
EXTI4_15_IRQHandler(void)
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

