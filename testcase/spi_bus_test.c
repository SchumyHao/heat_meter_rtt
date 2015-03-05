#include <rtthread.h>
#include "tc_comm.h"

static void _tc_cleanup(void)
{
    tc_done(TC_STAT_PASSED);
}

int test_spi_bus_register(void)
{
    rt_device_t spi_bus = RT_NULL;

#ifdef RT_USING_SPI1
    spi_bus = rt_device_find("spi1");
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if((spi_bus->write == RT_NULL) ||
       (spi_bus->close == RT_NULL) ||
       (spi_bus->open == RT_NULL) ||
       (spi_bus->read == RT_NULL) ||
       (spi_bus->init == RT_NULL)) {
        return -RT_ERROR;
    }
#if ((defined RT_USING_SPI1_TX_DMA) && (defined RT_USING_SPI1_RX_DMA))
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_DMA_RX|
                         RT_DEVICE_FLAG_DMA_TX)) {
        return -RT_ERROR;
    }
#elif (defined RT_USING_SPI1_TX_DMA)
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_INT_RX|
                         RT_DEVICE_FLAG_DMA_TX)) {
        return -RT_ERROR;
    }
#elif (defined RT_USING_SPI1_RX_DMA)
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_DMA_RX|
                         RT_DEVICE_FLAG_INT_TX)) {
        return -RT_ERROR;
    }
#else
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_INT_RX|
                         RT_DEVICE_FLAG_INT_TX)) {
        return -RT_ERROR;
    }
#endif /* defined(RT_USING_SPI1_TX_DMA)&&defined(RT_USING_SPI1_RX_DMA) */
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
#endif /* RT_USING_SPI1 */


#ifdef RT_USING_SPI2
    spi_bus = rt_device_find("spi2");
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if((spi_bus->write == RT_NULL) ||
       (spi_bus->close == RT_NULL) ||
       (spi_bus->open == RT_NULL) ||
       (spi_bus->read == RT_NULL) ||
       (spi_bus->init == RT_NULL)) {
        return -RT_ERROR;
    }
#if ((defined RT_USING_SPI2_TX_DMA) && (defined RT_USING_SPI2_RX_DMA))
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_DMA_RX|
                         RT_DEVICE_FLAG_DMA_TX)) {
        return -RT_ERROR;
    }
#elif (defined RT_USING_SPI2_TX_DMA)
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_INT_RX|
                         RT_DEVICE_FLAG_DMA_TX)) {
        return -RT_ERROR;
    }
#elif (defined RT_USING_SPI2_RX_DMA)
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_DMA_RX|
                         RT_DEVICE_FLAG_INT_TX)) {
        return -RT_ERROR;
    }
#else
    if(spi_bus->flag != (RT_DEVICE_FLAG_RDWR|
                         RT_DEVICE_FLAG_ACTIVATED|
                         RT_DEVICE_FLAG_INT_RX|
                         RT_DEVICE_FLAG_INT_TX)) {
        return -RT_ERROR;
    }
#endif /* defined(RT_USING_SPI2_TX_DMA)&&defined(RT_USING_SPI2_RX_DMA) */
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
#endif /* RT_USING_SPI2 */

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_spi_bus_register, test spi bus register);

int _tc_test_spi_bus_register(void)
{
    tc_cleanup(_tc_cleanup);

    if(RT_EOK != test_spi_bus_register()) {
        tc_stat(TC_STAT_FAILED);
    }
    return 100;
}
FINSH_FUNCTION_EXPORT(_tc_test_spi_bus_register, TC);

int test_spi_bus_open_close(void)
{
    rt_device_t spi_bus = RT_NULL;

#ifdef RT_USING_SPI1
    spi_bus = rt_device_find("spi1");
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag != (RT_DEVICE_OFLAG_OPEN | RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 1) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 2) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 1) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag == RT_DEVICE_OFLAG_CLOSE) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag != RT_DEVICE_OFLAG_CLOSE) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 0) {
        return -RT_ERROR;
    }
#endif /* RT_USING_SPI1 */

#ifdef RT_USING_SPI2
    spi_bus = rt_device_find("spi2");
    if(spi_bus == RT_NULL) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag != (RT_DEVICE_OFLAG_OPEN | RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 1) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_open(spi_bus, RT_DEVICE_OFLAG_RDWR)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 2) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 1) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag == RT_DEVICE_OFLAG_CLOSE) {
        return -RT_ERROR;
    }
    if(RT_EOK != rt_device_close(spi_bus)) {
        return -RT_ERROR;
    }
    if(spi_bus->open_flag != RT_DEVICE_OFLAG_CLOSE) {
        return -RT_ERROR;
    }
    if(spi_bus->ref_count != 0) {
        return -RT_ERROR;
    }
#endif /* RT_USING_SPI2 */

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(test_spi_bus_open_close, test spi bus open and close);

int _tc_test_spi_bus_open_close(void)
{
    tc_cleanup(_tc_cleanup);

    if(RT_EOK != test_spi_bus_open_close()) {
        tc_stat(TC_STAT_FAILED);
    }
    return 100;
}
FINSH_FUNCTION_EXPORT(_tc_test_spi_bus_open_close, TC);

