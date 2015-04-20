#include "gde021a1_device.h"

#define dbg_print         rt_kprintf

static struct rt_spi_device epd_gde_spi_dev;

#define TICKS_PER_MS   (SystemCoreClock/1000)
rt_inline void
epd_gde_ms_delay(rt_uint32_t ms)
{
    rt_uint32_t delta;
    ms = ms*TICKS_PER_MS;
    delta = SysTick->VAL;
    while((delta - SysTick->VAL) < ms);
}

void
EPD_Delay (uint32_t Delay)
{
    epd_gde_ms_delay(Delay);
}

void
EPD_IO_WriteData(uint8_t RegValue)
{
    /* Set EPD data/command line DC to High */
    EPD_DC_HIGH();

    rt_spi_transfer(&epd_gde_spi_dev, &RegValue, NULL, sizeof(RegValue));
}

void
EPD_IO_WriteReg(uint8_t Reg)
{
    /* Set EPD data/command line DC to Low */
    EPD_DC_LOW();

    rt_spi_transfer(&epd_gde_spi_dev, &Reg, NULL, sizeof(Reg));
}

uint16_t
EPD_IO_ReadData(void)
{
    rt_uint8_t recv_buf[2];
    rt_uint16_t recv_data = 0;
    rt_uint8_t* ptr = (rt_uint8_t*)&recv_data;
    rt_int8_t i = 1;

    rt_spi_transfer(&epd_gde_spi_dev, NULL, recv_buf, sizeof(recv_data));
    for(; i>=0; i--) {
        *ptr = recv_buf[i];
        ptr++;
    }

    return recv_data;
}

void
EPD_IO_Init(void)
{
    GPIO_InitTypeDef  epd_gpio;

    /* EPD_CS_GPIO and EPD_DC_GPIO Periph clock enable */
    RCC_AHBPeriphClockCmd(EPD_GPIO_DC_PIN_RCC|
                          EPD_GPIO_RESET_PIN_RCC|
                          EPD_GPIO_BUSY_PIN_RCC|
                          EPD_GPIO_PWR_PIN_RCC, ENABLE);

    /* Configure EPD_DC_PIN pin: EPD Card DC pin */
    epd_gpio.GPIO_Pin = EPD_GPIO_DC_PIN;
    epd_gpio.GPIO_Mode = GPIO_Mode_OUT;
    epd_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    epd_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EPD_GPIO_DC_PIN_GROUP, &epd_gpio);
    /* Configure EPD_RESET_PIN pin */
    epd_gpio.GPIO_Pin = EPD_GPIO_RESET_PIN;
    GPIO_Init(EPD_GPIO_RESET_PIN_GROUP, &epd_gpio);
    /* Configure EPD_PWR_PIN pin */
    epd_gpio.GPIO_Pin = EPD_GPIO_PWR_PIN;
    GPIO_Init(EPD_GPIO_PWR_PIN_GROUP, &epd_gpio);
    /* Configure EPD_BUSY_PIN pin */
    epd_gpio.GPIO_Pin = EPD_GPIO_BUSY_PIN;
    epd_gpio.GPIO_Mode = GPIO_Mode_IN;
    epd_gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(EPD_GPIO_BUSY_PIN_GROUP, &epd_gpio);

    /* Enbale Display */
    EPD_PWR_LOW();

    /* EPD reset pin mamagement */
    EPD_RESET_HIGH();
    EPD_Delay(10);
}

static rt_err_t
epd_gde_init(rt_device_t dev)
{
    gde021a1_Init();
    return RT_EOK;
}

static rt_err_t
epd_gde_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t
epd_gde_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t
epd_gde_control(rt_device_t dev, rt_uint8_t cmd, void* args)
{
    switch (cmd) {
        case RTGRAPHIC_CTRL_GET_INFO: {
            struct rt_device_graphic_info* info;

            info = (struct rt_device_graphic_info*) args;
            RT_ASSERT(info != RT_NULL);

            info->bits_per_pixel = 2;
            info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_GRAY4;
            info->framebuffer = RT_NULL;
            info->width = GDE021A1_EPD_PIXEL_WIDTH;
            info->height = GDE021A1_EPD_PIXEL_HEIGHT;
        }
        break;

        case RTGRAPHIC_CTRL_RECT_UPDATE:
            /* nothong to be done */
            break;

        default:
            break;
    }

    return RT_EOK;
}

static void
epd_gde_nss_init(struct stm32_spi_dev_cs* cs)
{
    GPIO_InitTypeDef EPD_GPIO;
    RT_ASSERT(cs != RT_NULL);

    RCC_AHBPeriphClockCmd(EPD_GPIO_NSS_PIN_RCC, ENABLE);
    GPIO_StructInit(&EPD_GPIO);
    EPD_GPIO.GPIO_Pin = EPD_GPIO_NSS_PIN;
    EPD_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    EPD_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    EPD_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(EPD_GPIO_NSS_PIN_GROUP, &EPD_GPIO);
    GPIO_SetBits(EPD_GPIO_NSS_PIN_GROUP, EPD_GPIO_NSS_PIN);
}

static void
epd_gde_nss_take(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_ResetBits(EPD_GPIO_NSS_PIN_GROUP, EPD_GPIO_NSS_PIN);
}

static void
epd_gde_nss_release(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_SetBits(EPD_GPIO_NSS_PIN_GROUP, EPD_GPIO_NSS_PIN);
}

static struct stm32_spi_dev_cs epd_nss_pin = {
    epd_gde_nss_init,
    epd_gde_nss_take,
    epd_gde_nss_release
};

static struct rt_device epd_gde_dev;

int 
rt_hw_epd_init(void)
{
    struct rt_spi_configuration cfg;
    rt_device_t spi_bus = RT_NULL;
    struct rt_spi_device* spi_dev = &epd_gde_spi_dev;
    rt_err_t ret = RT_EOK;

    /* 1. find spi bus */
    spi_bus = rt_device_find(SPI_BUS_NAME);
    if(spi_bus == RT_NULL) {
        dbg_print("spi bus %s not found!\r\n", SPI_BUS_NAME);
        return -RT_ENOSYS;
    }
    if(!(spi_bus->open_flag & RT_DEVICE_OFLAG_OPEN)) {
        if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
            dbg_print("spi bus %s open failed!\r\n", SPI_BUS_NAME);
            return -RT_ERROR;
        }
    }
    if(RT_EOK != rt_spi_bus_attach_device(spi_dev, "spiepd", SPI_BUS_NAME, &epd_nss_pin)) {
        dbg_print("epd spi device attach to spi bus %s failed!\r\n", SPI_BUS_NAME);
        return -RT_ERROR;
    }

    /* 2.config spi device */
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 2000000;
    ret = rt_spi_configure(spi_dev, &cfg);
    if(RT_EOK != ret) {
        return ret;
    }

    /* 3.register epd device */
    epd_gde_dev.type = RT_Device_Class_Graphic;
    epd_gde_dev.init = epd_gde_init;
    epd_gde_dev.open = epd_gde_open;
    epd_gde_dev.close = epd_gde_close;
    epd_gde_dev.control = epd_gde_control;
    epd_gde_dev.read = RT_NULL;
    epd_gde_dev.write = RT_NULL;
    epd_gde_dev.user_data = RT_NULL;
    rt_device_register(&epd_gde_dev, "epd",
                       RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_STANDALONE);
		
		return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_epd_init);
