#include <rtthread.h>
#include <drivers/spi.h>

#include "tc_comm.h"
#if 1
static void _tc_cleanup(void)
{
    tc_done(TC_STAT_PASSED);
}

int test_spi_dev(void)
{
    struct rt_spi_device spi_dev;
    struct rt_spi_configuration cfg;
    rt_device_t spi_bus = RT_NULL;
    rt_uint8_t tmp[40];
    const rt_uint8_t data[40] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                                 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                                 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                                 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                                 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88
                                };

#ifdef RT_USING_SPI1
    spi_bus = rt_device_find("spi1");
    if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spidev", "spi1", RT_NULL)) {
        return -RT_ERROR;
    }
#else
    spi_bus = rt_device_find("spi2");
    if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spidev", "spi2", RT_NULL)) {
        return -RT_ERROR;
    }
#endif /* RT_USING_SPI1 */
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 20000000;
    if(RT_EOK != rt_spi_configure(&spi_dev, &cfg)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_spi_take_bus(&spi_dev)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_spi_take(&spi_dev)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_spi_release(&spi_dev)) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_transfer(&spi_dev, data, tmp, 8) ||
       (0 != memcmp(tmp,data,8))) {
        return -RT_ERROR;
    }
    memset(tmp,0,sizeof(tmp));
    if(sizeof(data) != rt_spi_transfer(&spi_dev, data, tmp, sizeof(data)) ||
       (0 != memcmp(tmp,data,sizeof(data)))) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_recv(&spi_dev, tmp, 8)) {
        return -RT_ERROR;
    }
    if(sizeof(tmp) != rt_spi_recv(&spi_dev, tmp, sizeof(tmp))) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_send(&spi_dev, data, 8)) {
        return -RT_ERROR;
    }
    if(sizeof(data) != rt_spi_send(&spi_dev, data, sizeof(data))) {
        return -RT_ERROR;
    }
    cfg.data_width = 16;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 20000000;
    if(RT_EOK != rt_spi_configure(&spi_dev, &cfg)) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_transfer(&spi_dev, data, tmp, 8) ||
       (0 != memcmp(tmp,data,8))) {
        return -RT_ERROR;
    }
    memset(tmp,0,sizeof(tmp));
    if(sizeof(data) != rt_spi_transfer(&spi_dev, data, tmp, sizeof(data)) ||
       (0 != memcmp(tmp,data,sizeof(data)))) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_recv(&spi_dev, tmp, 8)) {
        return -RT_ERROR;
    }
    if(sizeof(tmp) != rt_spi_recv(&spi_dev, tmp, sizeof(tmp))) {
        return -RT_ERROR;
    }
    if(8 != rt_spi_send(&spi_dev, data, 8)) {
        return -RT_ERROR;
    }
    if(sizeof(data) != rt_spi_send(&spi_dev, data, sizeof(data))) {
        return -RT_ERROR;
    }

    if(RT_EOK != rt_device_unregister(&(spi_dev.parent))) {
        return -RT_ERROR;
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_spi_dev, test spi dev);

int _tc_test_spi_dev(void)
{
    tc_cleanup(_tc_cleanup);

    if(RT_EOK != test_spi_dev()) {
        tc_stat(TC_STAT_FAILED);
    }
    return 100;
}
FINSH_FUNCTION_EXPORT(_tc_test_spi_dev, TC);
#endif
