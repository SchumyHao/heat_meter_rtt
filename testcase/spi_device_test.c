#include <rtthread.h>
#include <drivers/spi.h>

#include "tc_comm.h"

static void _tc_cleanup(void)
{
    tc_done(TC_STAT_PASSED);
}

int test_spi_dev(void)
{
	struct rt_spi_device spi_dev;
	struct rt_spi_configuration cfg;
	rt_device_t spi_bus = RT_NULL;

#ifdef RT_USING_SPI1
	spi_bus = rt_device_find("spi1");
	if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spidev", "spi1", RT_NULL)){
		return -RT_ERROR;
	}
#else
	spi_bus = rt_device_find("spi2");
	if(RT_EOK != rt_spi_bus_attach_device(&spi_dev, "spidev", "spi2", RT_NULL)){
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
	cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
	cfg.max_hz = 20000000;
	if(RT_EOK != rt_spi_configure(&spi_dev, &cfg)){
		return -RT_ERROR;
	}
	if(RT_EOK != rt_spi_take_bus(&spi_dev)){
		return -RT_ERROR;
	}
	if(RT_EOK != rt_spi_take(&spi_dev)){
		return -RT_ERROR;
	}
	if(RT_EOK != rt_spi_release(&spi_dev)){
		return -RT_ERROR;
	}
	if(RT_EOK != rt_spi_sendrecv8(&spi_dev, 0xff)){
		return -RT_ERROR;
	}
	if(RT_EOK != rt_device_unregister(&(spi_dev.parent))){
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
