#include "board.h"
#include "stm32f0xx.h"
#include "epd.h"

#define dbg_print         rt_kprintf
#define SPI_BUS_NAME      "spi1"
/**
  * @brief  GDE021A1 Pins
  */
#define EPD_GPIO_NSS_PIN                    GPIO_Pin_4
#define EPD_GPIO_NSS_PIN_GROUP              GPIOA
#define EPD_GPIO_NSS_PIN_RCC                RCC_AHBPeriph_GPIOA
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
#define EPD_PWR_LOW()     GPIO_ResetBits(EPD_GPIO_PWR_PIN_GROUP, EPD_GPIO_PWR_PIN)
#define EPD_PWR_HIGH()    GPIO_SetBits(EPD_GPIO_PWR_PIN_GROUP, EPD_GPIO_PWR_PIN)
#define EPD_RESET_LOW()   GPIO_ResetBits(EPD_GPIO_RESET_PIN_GROUP, EPD_GPIO_RESET_PIN)
#define EPD_RESET_HIGH()  GPIO_SetBits(EPD_GPIO_RESET_PIN_GROUP, EPD_GPIO_RESET_PIN)
#define EPD_DC_LOW()      GPIO_ResetBits(EPD_GPIO_DC_PIN_GROUP, EPD_GPIO_DC_PIN)
#define EPD_DC_HIGH()     GPIO_SetBits(EPD_GPIO_DC_PIN_GROUP, EPD_GPIO_DC_PIN)

/**
  * @brief  GDE021A1 Registers
  */
#define EPD_REG_0             0x00   /* Status Read */
#define EPD_REG_1             0x01   /* Driver Output Control */
#define EPD_REG_3             0x03   /* Gate driving voltage control */
#define EPD_REG_4             0x04   /* Source driving coltage control */
#define EPD_REG_7             0x07   /* Display Control */
#define EPD_REG_11            0x0B   /* Gate and Sorce non overlap period COntrol */
#define EPD_REG_15            0x0F   /* Gate scan start */
#define EPD_REG_16            0x10   /* Deep Sleep mode setting */
#define EPD_REG_17            0x11   /* Data Entry Mode Setting */
#define EPD_REG_18            0x12   /* SWRESET */
#define EPD_REG_26            0x1A   /* Temperature Sensor Control (Write to Temp Register) */
#define EPD_REG_27            0x1B   /* Temperature Sensor Control(Read from Temp Register) */
#define EPD_REG_28            0x1C   /* Temperature Sensor Control(Write Command  to Temp sensor) */
#define EPD_REG_29            0x1D   /* Temperature Sensor Control(Load temperature register with temperature sensor reading) */
#define EPD_REG_32            0x20   /* Master activation */
#define EPD_REG_33            0x21   /* Display update */
#define EPD_REG_34            0x22   /* Display update control 2 */
#define EPD_REG_36            0x24   /* write RAM */
#define EPD_REG_37            0x25   /* Read RAM */
#define EPD_REG_40            0x28   /* VCOM sense */
#define EPD_REG_41            0x29   /* VCOM Sense duration */
#define EPD_REG_42            0x2A   /* VCOM OTP program */
#define EPD_REG_44            0x2C   /* Write VCOMregister */
#define EPD_REG_45            0x2D   /* Read OTP registers */
#define EPD_REG_48            0x30   /* Program WS OTP */
#define EPD_REG_50            0x32   /* Write LUT register */
#define EPD_REG_51            0x33   /* Read LUT register */
#define EPD_REG_54            0x36   /* Program OTP selection */
#define EPD_REG_55            0x37   /* Proceed OTP selection */
#define EPD_REG_58            0x3A   /* Set dummy line pulse period */
#define EPD_REG_59            0x3B   /* Set Gate line width */
#define EPD_REG_60            0x3C   /* Select Border waveform */
#define EPD_REG_68            0x44   /* Set RAM X - Address Start / End Position */
#define EPD_REG_69            0x45   /* Set RAM Y - Address Start / End Position */
#define EPD_REG_78            0x4E   /* Set RAM X Address Counter */
#define EPD_REG_79            0x4F   /* Set RAM Y Address Counter */
#define EPD_REG_240           0xF0   /* Booster Set Internal Feedback Selection */
#define EPD_REG_255           0xFF   /* NOP */

/**
  * @brief  GDE021A1 Private Variables
  */
const unsigned char WF_LUT[]= {
    0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,
    0xAA,0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,
    0x55,0xAA,0xAA,0x00,0x55,0x55,0x55,0x55,
    0xAA,0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,
    0xAA,0xAA,0xAA,0xAA,0x15,0x15,0x15,0x15,
    0x05,0x05,0x05,0x05,0x01,0x01,0x01,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x41,0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,
    0x00,0x00
};

#define TICKS_PER_MS   (SystemCoreClock/1000)
static void
epd_gde_ms_delay(rt_uint32_t ms)
{
    rt_uint32_t delta;
    ms = ms*TICKS_PER_US;
    delta = SysTick->VAL;
    while((delta - SysTick->VAL) < ms);
}

rt_inline void
EPD_Delay (uint32_t Delay)
{
    epd_gde_ms_delay(Delay);
}


static struct rt_spi_device epd_gde_spi_dev;

static void
EPD_IO_WriteData(uint16_t RegValue)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t* ptr = (rt_uint8_t*)&RegValue;
    rt_int8_t i = 1;

    /* Set EPD data/command line DC to High */
    EPD_DC_HIGH();

    for(; i>=0; i--) {
        send_buf[i] = *ptr;
        ptr++;
    }
    rt_spi_transfer(&epd_gde_spi_dev, send_buf, NULL, sizeof(RegValue))
}

static void
EPD_IO_WriteReg(uint8_t Reg)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t* ptr = (rt_uint8_t*)&Reg;
    rt_int8_t i = 1;

    /* Set EPD data/command line DC to Low */
    EPD_DC_LOW();

    for(; i>=0; i--) {
        send_buf[i] = *ptr;
        ptr++;
    }
    rt_spi_transfer(&epd_gde_spi_dev, send_buf, NULL, sizeof(Reg))
}

static uint16_t
EPD_IO_ReadData(void)
{
    rt_uint8_t recv_buf[2];
    rt_uint16_t recv_data = 0;
    rt_uint8_t* ptr = (rt_uint8_t*)&recv_data;
    rt_int8_t i = 1;

    rt_spi_transfer(&epd_gde_spi_dev, NULL, recv_buf, sizeof(recv_data))
    for(; i>=0; i--) {
        *ptr = recv_buf[i];
        ptr++;
    }

    return recv_data;
}

static void
_epd_init(void){
	rt_device_t epd = RT_NULL;

    epd = rt_device_find("epd");
    if(epd == RT_NULL) {
        dbg_print("epd device not found!\r\n");
        return ;
    }
	rt_device_init(epd);
}

static void
epd_gde_write_pixel(uint8_t HEX_Code)
{
  /* Prepare the register to write data on the RAM */
  EPD_IO_WriteReg(EPD_REG_36);

  /* Send the data to write */
  EPD_IO_WriteData(HEX_Code);
}

static void
epd_gde_get_pixel(char* pixel, int x, int y)
{

}

static void
epd_gde_draw_hline(const char* pixel, int x1, int x2, int y)
{

}

static void
epd_gde_draw_vline(const char* pixel, int x, int y1, int y2)
{

}

static void
epd_gde_draw_blit_line(const char* pixels, int x, int y, rt_size_t size)
{

}

static EPD_DrvTypeDef epd_gde_ops = {
	_epd_init,
	epd_gde_write_pixel,
	
    epd_gde_set_pixel,
    epd_gde_get_pixel,
    epd_gde_draw_hline,
    epd_gde_draw_vline,
    epd_gde_draw_blit_line
};

static void
epd_io_init(void)
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
    EPD_RESET_LOW();
    EPD_Delay(1);
    EPD_RESET_HIGH();
    EPD_Delay(10);
}

static rt_err_t
epd_gde_init(rt_device_t dev)
{
    rt_uint8_t nb_bytes = 0;
    rt_err_t ret = RT_EOK;

    epd_io_init();

    EPD_IO_WriteReg(EPD_REG_16);  /* Deep sleep mode disable */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_17);  /* Data Entry Mode Setting */
    EPD_IO_WriteData(0x03);
    EPD_IO_WriteReg(EPD_REG_68);  /* Set the RAM X start/end address */
    EPD_IO_WriteData(0x00);       /* RAM X address start = 00h */
    EPD_IO_WriteData(0x11);       /* RAM X adress end = 11h (17 * 4pixels by address = 72 pixels) */
    EPD_IO_WriteReg(EPD_REG_69);  /* Set the RAM Y start/end address */
    EPD_IO_WriteData(0x00);       /* RAM Y address start = 0 */
    EPD_IO_WriteData(0xAB);       /* RAM Y adress end = 171 */
    EPD_IO_WriteReg(EPD_REG_78);  /* Set RAM X Address counter */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_79);  /* Set RAM Y Address counter */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_240); /* Booster Set Internal Feedback Selection */
    EPD_IO_WriteData(0x1F);
    EPD_IO_WriteReg(EPD_REG_33);  /* Disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3 */
    EPD_IO_WriteData(0x03);
    EPD_IO_WriteReg(EPD_REG_44);  /* Write VCOMregister */
    EPD_IO_WriteData(0xA0);
    EPD_IO_WriteReg(EPD_REG_60);  /* Border waveform */
    EPD_IO_WriteData(0x64);
    EPD_IO_WriteReg(EPD_REG_50);  /* Write LUT register */

    for (nb_bytes=0; nb_bytes<90; nb_bytes++) {
        EPD_IO_WriteData(WF_LUT[nb_bytes]);
    }
}

static rt_err_t
epd_gde_open(rt_device_t dev, rt_uint16_t oflag) {}

static rt_err_t
epd_gde_close(rt_device_t dev) {}

static rt_err_t
epd_gde_control(rt_device_t dev, rt_uint8_t cmd, void* args) {

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

static rt_device epd_gde_dev;

void rt_hw_epd_init(void)
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
    epd_gde_dev.user_data = &epd_gde_ops;
    rt_device_register(rt_device_t dev, "epd",
                       RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_STANDALONE);
}
