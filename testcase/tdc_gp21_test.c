#include <rtthread.h>

#include "tc_comm.h"
#include "spi_tdc_gp21.h"

static void
_tc_cleanup(void)
{
    tc_done(TC_STAT_PASSED);
}

int
test_tdc_gp21(void)
{
    struct rt_spi_device spi_dev;
    rt_device_t spi_bus = RT_NULL;
    rt_device_t tdc = RT_NULL;
    struct spi_tdc_gp21_tof_data tof_data;
    struct spi_tdc_gp21_temp_scales temp_scales;

    spi_bus = rt_device_find("spi1");
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spitdc", spi_bus->parent.name, RT_NULL)) {
        return -RT_ERROR;
    }

    if(RT_EOK != tdc_register("tdc1", spi_dev.parent.parent.name)) {
        return -RT_ERROR;
    }

    tdc = rt_device_find("tdc1");
    if(RT_NULL == tdc) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(tdc, RT_NULL)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_control(tdc,
                                   SPI_TDC_GP21_CTRL_MEASURE_TEMP,
                                   &temp_scales)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_control(tdc,
                                   SPI_TDC_GP21_CTRL_MEASURE_TOF2,
                                   &tof_data)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_close(tdc)) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_unregister(&(spi_dev.parent))) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_unregister(tdc)) {
        return -RT_ERROR;
    }

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_tdc_gp21, test tdc gp21);

int
_tc_test_tdc_gp21(void)
{
    tc_cleanup(_tc_cleanup);

    if(RT_EOK != test_tdc_gp21()) {
        tc_stat(TC_STAT_FAILED);
    }
    return 100;
}
FINSH_FUNCTION_EXPORT(_tc_test_tdc_gp21, TC);
