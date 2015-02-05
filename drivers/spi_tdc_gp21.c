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
    local variables
*/
static struct spi_tdc_gp21 spi_tdc_gp21;
static struct spi_tdc_gp21_userdata spi_tdc_gp21_userdata;

/*
    local functions
*/
static uint16_t gp21_read_register16(uint8_t opcode)
{
    uint16_t recv_buf = 0;
    rt_spi_send_then_recv(spi_tdc_gp21.rt_spi_device,
                          &opcode, 1,
                          &recv_buf, 2);
    return recv_buf;
}

static uint32_t gp21_read_register32(uint8_t opcode)
{
    uint32_t recv_buf = 0;
    rt_spi_send_then_recv(spi_tdc_gp21.rt_spi_device,
                          &opcode, 1,
                          &recv_buf, 4);
    return recv_buf;
}

static void gp21_write_register16(uint8_t opcode, uint16_t send_buf)
{
    rt_spi_send_then_send(spi_tdc_gp21.rt_spi_device,
                          &opcode, 1,
                          &send_buf, 2);
}

static void gp21_write_register24(uint8_t opcode, uint32_t send_buf)
{
    rt_spi_send_then_send(spi_tdc_gp21.rt_spi_device,
                          &opcode, 1,
                          &send_buf, 3);
}

static void gp21_write_register32(uint8_t opcode, uint32_t send_buf)
{
    rt_spi_send_then_send(spi_tdc_gp21.rt_spi_device,
                          &opcode, 1,
                          &send_buf, 4);
}

static void busy_wait(void)
{
    /* set system into sleep mode until interrupt occered on gp21 initpin */
    static struct spi_tdc_gp21_userdata* userdata = NULL;
    static uint32_t EXTI_Line = 0;
    FlagStatus int_pin_status = RESET;

    if(NULL == userdata) {
        userdata = (struct spi_tdc_gp21_userdata*)spi_tdc_gp21.tdc_device.user_data;
    }
    if(0 == EXTI_Line) {
        switch(userdata->intpin->GPIO_Pin) {
            case GPIO_Pin_0:
                EXTI_Line = EXTI_Line0;
                break;
            case GPIO_Pin_1:
                EXTI_Line = EXTI_Line1;
                break;
            case GPIO_Pin_2:
                EXTI_Line = EXTI_Line2;
                break;
            case GPIO_Pin_3:
                EXTI_Line = EXTI_Line3;
                break;
            case GPIO_Pin_4:
                EXTI_Line = EXTI_Line4;
                break;
            case GPIO_Pin_5:
                EXTI_Line = EXTI_Line5;
                break;
            case GPIO_Pin_6:
                EXTI_Line = EXTI_Line6;
                break;
            case GPIO_Pin_7:
                EXTI_Line = EXTI_Line7;
                break;
            case GPIO_Pin_8:
                EXTI_Line = EXTI_Line8;
                break;
            case GPIO_Pin_9:
                EXTI_Line = EXTI_Line9;
                break;
            case GPIO_Pin_10:
                EXTI_Line = EXTI_Line10;
                break;
            case GPIO_Pin_11:
                EXTI_Line = EXTI_Line11;
                break;
            case GPIO_Pin_12:
                EXTI_Line = EXTI_Line12;
                break;
            case GPIO_Pin_13:
                EXTI_Line = EXTI_Line13;
                break;
            case GPIO_Pin_14:
                EXTI_Line = EXTI_Line14;
                break;
            case GPIO_Pin_15:
                EXTI_Line = EXTI_Line15;
                break;
            default:
                TDC_TRACE("TDC Interrupt pin 0x%04x not found!\r\n", userdata->intpin->GPIO_Pin);
        }
    }

    while(RESET == int_pin_status) {
        PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
        int_pin_status = EXTI_GetFlagStatus(EXTI_Line);
    }
    EXTI_ClearFlag(EXTI_Line);
}

static void gp21_write_cmd(uint8_t opcode)
{
    rt_spi_send(spi_tdc_gp21.rt_spi_device, &opcode, 1);
    /* some cmd need wait interrupt pin */
    if((opcode == GP21_WRITE_CFG_TO_EEPROM)  ||
       (opcode == GP21_WRITE_EEPROM_TO_CFG)  ||
       (opcode == GP21_COMPARE_EEPROM_CFG)   ||
       (opcode == GP21_START_TOF)            ||
       (opcode == GP21_START_TEMP)           ||
       (opcode == GP21_START_CAL_RESONATOR)  ||
       (opcode == GP21_START_TOF_RESTART)    ||
       (opcode == GP21_START_TEMP_RESTART)) {
        busy_wait();
    }
}

static rt_err_t tdc_gp21_init(rt_device_t dev)
{
    /*
        config register0 :
        ANZ_FIRE = 31        DIV_FIRE = 3      ANZ_PER_CALRES = 3
        DIV_CLKHS = 0        START_CLKHS = 2   ANZ_PORT = 1
        TCYCLE = 1           ANZ_FAKE = 0      SEL_ECLK_TMP = 1
        CALIBRATE = 1        NO_CAL_AUTO = 0   MESSB2 = 1
        NEG_STOP/NEGSTART=0  ID0 = 0x10
    */
    gp21_write_register32(GP21_WRITE_REG0_REGISTER, 0xF3CB6810);
    /*
        config register1 :
        HIT1 = 2             HIT1 = 1          EN_FAST_INIT = 0
        HITIN2 = 0           HITIN1 = 4        CURR32K = 0
        SEL_START_FIRE = 1   SEL_TSTO2 = 2     SEL_TSTO1 = 7
        ID1 = 0x12
    */
    gp21_write_register32(GP21_WRITE_REG1_REGISTER, 0x21445712);
    /*
        config register2 :
        EN_INT = B1101       RFEDGE1 = 0       RFEDGE2 = 0
        DELVAL1 = 400        ID2 = 0x12
    */
    gp21_write_register32(GP21_WRITE_REG2_REGISTER, 0xA0320012);
    /*
        config register3 :
        EN_ERR_VAL = 0       SEL_TIMO_MB2 = 1  DELVAL2 = 408
        ID3 = 0x34
    */
    gp21_write_register32(GP21_WRITE_REG3_REGISTER, 0x08330034);
    /*
        config register4 :
        DELVAL3 = 416        ID4 = 0x56
    */
    gp21_write_register32(GP21_WRITE_REG4_REGISTER, 0x20340056);
    /*
        config register5 :
        CON_FIRE = 0         EN_STARTNOISE = 0 DIS_PHASESHIFT = 0
        REPEAT_FIRE = 0      PHASE_FIRE = 0    ID5 = 0x78
    */
    gp21_write_register32(GP21_WRITE_REG5_REGISTER, 0x00000078);
    /*
        config register6 :
        EN_ANALOG = 1        NEG_STOP_TEMP = 1 TW2 = 1
        EN_INT = b1101       START_CLKHS = 2   CYCLE_TEMP = 1
        CYCLE_TOF = 0        HZ60 = 0          FIREO_DEF = 1
        QUAD_RES = 0         DOUBLE_RES = 1    TEMP_PORTDIR = 0
        ANZ_FIRE = 31        ID6 = 0x90
    */
    gp21_write_register32(GP21_WRITE_REG6_REGISTER, 0xC0645190);

    /*
        write config registers to eeprom
    */
    gp21_write_cmd(GP21_WRITE_CFG_TO_EEPROM);

    return RT_EOK;
}

static rt_err_t tdc_gp21_open(rt_device_t dev, rt_uint16_t oflag)
{
    uint32_t reg = 0;
    struct spi_tdc_gp21_userdata* userdata = NULL;
    RT_ASSERT(dev != RT_NULL);

    if(dev->open_flag & RT_DEVICE_OFLAG_OPEN) {
        TDC_TRACE("TDC device %d reopen!\r\n", dev->device_id);
    }

    userdata = (struct spi_tdc_gp21_userdata*)dev->user_data;

    gp21_write_cmd(GP21_POWER_ON_RESET);
    gp21_write_cmd(GP21_WRITE_EEPROM_TO_CFG);
    gp21_write_cmd(GP21_START_CAL_RESONATOR);
    reg = gp21_read_register32(GP21_READ_RES0_REGISTER);
    userdata->corr_factor = 488.28125/reg;

    dev->ref_count ++;
    dev->open_flag |= RT_DEVICE_OFLAG_OPEN;
    return RT_EOK;
}

static rt_err_t tdc_gp21_close(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);

    if(--dev->ref_count == 0) {
        dev->open_flag = RT_DEVICE_OFLAG_CLOSE;
    }

    return RT_EOK;
}

static void tdc_gp21_measure_temp(rt_device_t dev, struct spi_tdc_gp21_temp_scales* args)
{
    uint16_t stat = 0;
    uint32_t res[4] = {0,0,0,0};
    struct spi_tdc_gp21_temp_scales temp = {0,0};

    gp21_write_cmd(GP21_START_TEMP_RESTART);
    stat = gp21_read_register16(GP21_READ_STAT_REGISTER);
    if(stat & 0x1E00) {
        TDC_TRACE("TDC gp21 measure temperature error=0x%04x !\r\n", stat);
        return;
    }
    res[0] = gp21_read_register32(GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(GP21_READ_RES2_REGISTER);
    res[3] = gp21_read_register32(GP21_READ_RES3_REGISTER);
    temp.hot  = (float)res[0]/(float)res[1];
    temp.cold = (float)res[3]/(float)res[2];
    /* wait for next measure finish */
    busy_wait();
    stat = gp21_read_register16(GP21_READ_STAT_REGISTER);
    if(stat & 0x1E00) {
        TDC_TRACE("TDC gp21 measure temperature error=0x%04x !\r\n", stat);
        return;
    }
    res[0] = gp21_read_register32(GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(GP21_READ_RES2_REGISTER);
    res[3] = gp21_read_register32(GP21_READ_RES3_REGISTER);
    args->hot  = (temp.hot  + ((float)res[0]/(float)res[1]))/2;
    args->cold = (temp.cold + ((float)res[3]/(float)res[2]))/2;
}

static void wait_for_alu(void)
{
    /* ALU calculate need no more than 4.6us */
    uint32_t i = 0x00FFFFFF;
    while(i--);
}

static void tdc_gp21_measure_tof2(rt_device_t dev, struct spi_tdc_gp21_tof_data* args)
{
    uint16_t stat = 0;
    uint32_t res[3] = {0,0,0};

    gp21_write_cmd(GP21_INITIATE_TDC);
    gp21_write_cmd(GP21_START_TOF_RESTART);
    stat = gp21_read_register16(GP21_READ_STAT_REGISTER);
    if(stat & 0x0600) {
        TDC_TRACE("TDC gp21 measure ToF error=0x%04x !\r\n", stat);
        return;
    }
    /* ALU will automatically cal STOP1-START and store answer to res0 */
    /* ALU cal STOP2-START and store answer to res1 by send command below */
    gp21_write_register24(GP21_WRITE_REG1_REGISTER, 0x31400000);
    wait_for_alu();
    /* ALU cal STOP3-START and store answer to res2 by send command below */
    gp21_write_register24(GP21_WRITE_REG1_REGISTER, 0x41400000);
    wait_for_alu();
    res[0] = gp21_read_register32(GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(GP21_READ_RES2_REGISTER);
    args->up = (res[0]+res[1]+res[2])/3;

    /* wait for next measure finish */
    busy_wait();
    stat = gp21_read_register16(GP21_READ_STAT_REGISTER);
    if(stat & 0x0600) {
        TDC_TRACE("TDC gp21 measure ToF error=0x%04x !\r\n", stat);
        return;
    }
    gp21_write_register24(GP21_WRITE_REG1_REGISTER, 0x31400000);
    wait_for_alu();
    gp21_write_register24(GP21_WRITE_REG1_REGISTER, 0x41400000);
    wait_for_alu();
    res[0] = gp21_read_register32(GP21_READ_RES0_REGISTER);
    res[1] = gp21_read_register32(GP21_READ_RES1_REGISTER);
    res[2] = gp21_read_register32(GP21_READ_RES2_REGISTER);
    args->down = (res[0]+res[1]+res[2])/3;
}

static rt_err_t tdc_gp21_control(rt_device_t dev, rt_uint8_t cmd, void* args)
{
    RT_ASSERT(dev != RT_NULL);

    if(!(dev->open_flag & RT_DEVICE_OFLAG_OPEN)) {
        TDC_TRACE("TDC device %d does not open!\r\n", dev->device_id);
        return -RT_EIO;
    }

    switch(cmd) {
        case SPI_TDC_GP21_CTRL_MEASURE_TEMP:
            tdc_gp21_measure_temp(dev, (struct spi_tdc_gp21_temp_scales*)args);
            break;
        case SPI_TDC_GP21_CTRL_MEASURE_TOF2:
            tdc_gp21_measure_tof2(dev, (struct spi_tdc_gp21_tof_data*)args);
            break;
        default:
            TDC_TRACE("TDC gp21 control cmd 0x%02x does not support!\r\n", cmd);
    }

    return RT_EOK;
}

static rt_size_t tdc_gp21_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    TDC_TRACE("TDC device %d does not support read, use control!\r\n", dev->device_id);
    return 0;
}

static rt_size_t tdc_gp21_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    TDC_TRACE("TDC device %d does not support write, use control!\r\n", dev->device_id);
    return 0;
}

/*
    global functions
*/
rt_err_t gp21_register(const char* tdc_device_name,
                       const char* spi_device_name)
{
    struct rt_spi_device* rt_spi_device;

    /* 1.find spi device */
    rt_spi_device = (struct rt_spi_device*)rt_device_find(spi_device_name);
    if(RT_NULL == rt_spi_device) {
        TDC_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_tdc_gp21.rt_spi_device = rt_spi_device;

    /* 2.config spi device */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB | RT_SPI_SLAVE;
        cfg.max_hz = 20000000;
        rt_spi_configure(spi_tdc_gp21.rt_spi_device, &cfg);
    }

    /* 3.register device */
    spi_tdc_gp21.tdc_device.type        = RT_Device_Class_Miscellaneous;
    spi_tdc_gp21.tdc_device.init        = tdc_gp21_init;
    spi_tdc_gp21.tdc_device.open        = tdc_gp21_open;
    spi_tdc_gp21.tdc_device.close       = tdc_gp21_close;
    spi_tdc_gp21.tdc_device.control     = tdc_gp21_control;
    spi_tdc_gp21.tdc_device.read        = tdc_gp21_read;
    spi_tdc_gp21.tdc_device.write       = tdc_gp21_write;
    spi_tdc_gp21.tdc_device.user_data   = (void*)&spi_tdc_gp21_userdata;
    rt_device_register(&spi_tdc_gp21.tdc_device, tdc_device_name, RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
