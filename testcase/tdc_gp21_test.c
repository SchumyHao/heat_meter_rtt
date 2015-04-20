#include <rtthread.h>
#if 0
#include "finsh.h"
#include "spi_tdc_gp21.h"

int
test_tdc_gp21(void)
{
    rt_device_t tdc = RT_NULL;
#if 0
    struct spi_tdc_gp21_tof_data tof_data;
    struct spi_tdc_gp21_temp_scales temp_scales;
#endif

    if(RT_EOK != tdc_gp21_register()) {
        return -RT_ERROR;
    }

    tdc = rt_device_find("tdc1");
    if(RT_NULL == tdc) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(tdc, RT_NULL)) {
        return -RT_ERROR;
    }
#if 0
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
    if(RT_EOK != rt_device_unregister(tdc)) {
        return -RT_ERROR;
    }
#endif
		return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_tdc_gp21, test tdc gp21);


int
test_tdc_gp21_tof(void)
{
    rt_device_t tdc = RT_NULL;
    struct spi_tdc_gp21_tof_data tof_data;

    tdc = rt_device_find("tdc1");
    if(RT_NULL == tdc) {
        return -RT_ERROR;
    }

    if(RT_EOK != rt_device_control(tdc,
                                   SPI_TDC_GP21_CTRL_MEASURE_TOF2,
                                   &tof_data)) {
        return -RT_ERROR;
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_tdc_gp21_tof, test tdc gp21 tof);
#endif
