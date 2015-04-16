#include <rtthread.h>

#include "tc_comm.h"
#include "spi_flash_w25qxx.h"
#include "rt_stm32f0xx_spi.h"

#define FLASH_GPIO_NSS_PIN                    GPIO_Pin_5
#define FLASH_GPIO_NSS_PIN_GROUP              GPIOC
#define FLASH_GPIO_NSS_PIN_RCC                RCC_AHBPeriph_GPIOC

void flash_nss_init(struct stm32_spi_dev_cs* cs)
{
    GPIO_InitTypeDef FLASH_GPIO;
    RT_ASSERT(cs != RT_NULL);

    RCC_AHBPeriphClockCmd(FLASH_GPIO_NSS_PIN_RCC, ENABLE);
    GPIO_StructInit(&FLASH_GPIO);
    FLASH_GPIO.GPIO_Pin = FLASH_GPIO_NSS_PIN;
    FLASH_GPIO.GPIO_Mode = GPIO_Mode_OUT;
    FLASH_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    FLASH_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FLASH_GPIO_NSS_PIN_GROUP, &FLASH_GPIO);
    GPIO_SetBits(FLASH_GPIO_NSS_PIN_GROUP, FLASH_GPIO_NSS_PIN);
}
void flash_nss_take(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_ResetBits(FLASH_GPIO_NSS_PIN_GROUP, FLASH_GPIO_NSS_PIN);
}
void flash_nss_release(struct stm32_spi_dev_cs* cs)
{
    RT_ASSERT(cs != RT_NULL);

    GPIO_SetBits(FLASH_GPIO_NSS_PIN_GROUP, FLASH_GPIO_NSS_PIN);
}
static struct stm32_spi_dev_cs flash_nss_pin = {
    flash_nss_init,
    flash_nss_take,
    flash_nss_release
};

static void
_tc_cleanup(void)
{
    tc_done(TC_STAT_PASSED);
}

int
test_flash_w25qxx(void)
{
    rt_device_t spi_bus = RT_NULL;
    rt_device_t flash = RT_NULL;
    struct rt_spi_device* spi_dev = RT_NULL;
    const char* spi_bus_name = "spi1";

    spi_bus = rt_device_find(spi_bus_name);
    if(spi_bus == RT_NULL) {
        rt_kprintf("spi bus %s not found!\r\n", spi_bus_name);
        return -RT_ENOSYS;
    }
    if(!(spi_bus->open_flag & RT_DEVICE_OFLAG_OPEN)) {
        if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
            rt_kprintf("spi bus %s open failed!\r\n", spi_bus_name);
            return -RT_ERROR;
        }
    }
    spi_dev = (struct rt_spi_device*)rt_malloc(sizeof(*spi_dev));
    RT_ASSERT(spi_dev != RT_NULL);
    if(RT_EOK != rt_spi_bus_attach_device(spi_dev, "spiflas", spi_bus_name, &flash_nss_pin)) {
        rt_kprintf("spi flash device attach to spi bus %s failed!\r\n", spi_bus_name);
        return -RT_ERROR;
    }
    w25qxx_init("flash","spiflas");
    flash =rt_device_find("flash");
    rt_device_open(flash,RT_DEVICE_OFLAG_RDWR);
    rt_device_write(flash, 0, "0123456789", sizeof("0123456789"));
    {
        char temp[5];
        rt_device_read(flash, 0, temp, 5);
        rt_device_read(flash, 5, temp, 5);
    }

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_flash_w25qxx, test flash);

int
_tc_test_flash_w25qxx(void)
{
    tc_cleanup(_tc_cleanup);

    if(RT_EOK != test_flash_w25qxx()) {
        tc_stat(TC_STAT_FAILED);
    }
    return 100;
}
FINSH_FUNCTION_EXPORT(_tc_test_flash_w25qxx, TC);

