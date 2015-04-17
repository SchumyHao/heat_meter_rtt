#include "board.h"
#include "stm32f0xx.h"
#include "epd.h"

#define dbg_print         rt_kprintf

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



static void
epd_gde_set_pixel(const char* pixel, int x, int y)
{

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
epd_gde_draw_blit_line(const char* pixels, int x, int y, rt_size_t size) {}

static struct rt_device_graphic_ops epd_gde_ops = {
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


    /* Configure EPD_CS_PIN pin: EPD Card CS pin */
    GPIO_InitStruct.Pin = EPD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(EPD_CS_GPIO_PORT, &GPIO_InitStruct);

    /* Configure EPD_DC_PIN pin: EPD Card DC pin */
    GPIO_InitStruct.Pin = EPD_DC_PIN;
    HAL_GPIO_Init(EPD_DC_GPIO_PORT, &GPIO_InitStruct);

    /* Configure EPD_RESET_PIN pin */
    GPIO_InitStruct.Pin = EPD_RESET_PIN;
    HAL_GPIO_Init(EPD_PWR_GPIO_PORT, &GPIO_InitStruct);

    /* Configure EPD_RESET_PIN pin */
    GPIO_InitStruct.Pin = EPD_PWR_PIN;
    HAL_GPIO_Init(EPD_RESET_GPIO_PORT, &GPIO_InitStruct);

    /* Configure EPD_BUSY_PIN pin */
    GPIO_InitStruct.Pin = EPD_BUSY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(EPD_BUSY_GPIO_PORT, &GPIO_InitStruct);

    /* Enbale Display */
    EPD_PWR_LOW();

    /* Set or Reset the control line */
    EPD_CS_LOW();
    EPD_CS_HIGH();

    /* EPD reset pin mamagement */
    EPD_RESET_HIGH();
    EPD_Delay(10);

    /* SPI Configuration */
    SPIx_Init();
}

static rt_err_t
epd_gde_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;

    epd_io_init();
}

static rt_err_t
epd_gde_open(rt_device_t dev, rt_uint16_t oflag) {}

static rt_err_t
epd_gde_close(rt_device_t dev) {}

static rt_err_t
epd_gde_control(rt_device_t dev, rt_uint8_t cmd, void* args) {}

void epd_gde_nss_init(struct stm32_spi_dev_cs* cs)
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
void epd_gde_nss_take(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_ResetBits(EPD_GPIO_NSS_PIN_GROUP, EPD_GPIO_NSS_PIN);
}
void epd_gde_nss_release(struct stm32_spi_dev_cs* cs)
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
