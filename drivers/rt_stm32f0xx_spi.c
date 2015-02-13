/*
 * File      : rt_stm32f0xx_spi.c
 * COPYRIGHT (C) 2014-2015, Schumy Hao
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-12-19     Schumy       the first version
 */

#include <rtdevice.h>
#include "stm32f0xx.h"
#include "rt_stm32f0xx_spi.h"

#define SPI_DUMP 0xff
#define SPI_IT_XFER_ONEC_MAX_LEN (32)

/* SPI1 */
#define SPI1_GPIO_MISO_PIN       GPIO_Pin_6
#define SPI1_GPIO_MISO_SOURCE    GPIO_PinSource6
#define SPI1_GPIO_MOSI_PIN       GPIO_Pin_7
#define SPI1_GPIO_MOSI_SOURCE    GPIO_PinSource7
#define SPI1_GPIO_SCLK_PIN       GPIO_Pin_5
#define SPI1_GPIO_SCLK_SOURCE    GPIO_PinSource5
#define SPI1_GPIO_NSS_PIN        GPIO_Pin_4
#define SPI1_GPIO_NSS_SOURCE     GPIO_PinSource4
#define SPI1_GPIO_PIN_GROUP      GPIOA
#define SPI1_GPIO_PIN_RCC        RCC_AHBPeriph_GPIOA
#define SPI1_GPIO_PIN_AF         GPIO_AF_0
#define SPI1_RCC                 RCC_APB2Periph_SPI1
#define SPI1_DMA_RCC             RCC_AHBPeriph_DMA1
#ifdef RT_USING_SPI1_TX_DMA
#define SPI1_DMA_TX              DMA1_Channel3
#else
#define SPI1_DMA_TX              RT_NULL
static uint8_t stm32_spi1_tx_buf[SPI_XFER_ONEC_MAX_LEN+1];
static struct rt_ringbuffer stm32_spi1_tx_rb;
#endif /* RT_USING_SPI1_TX_DMA */
#ifdef RT_USING_SPI1_RX_DMA
#define SPI1_DMA_RX              DMA1_Channel2
#else
#define SPI1_DMA_RX              RT_NULL
static uint8_t stm32_spi1_rx_buf[SPI_XFER_ONEC_MAX_LEN+1];
static struct rt_ringbuffer stm32_spi1_rx_rb;
#endif /* RT_USING_SPI1_RX_DMA */

/* SPI2 */
#define SPI2_GPIO_MISO_PIN       GPIO_Pin_14
#define SPI2_GPIO_MISO_SOURCE    GPIO_PinSource14
#define SPI2_GPIO_MOSI_PIN       GPIO_Pin_15
#define SPI2_GPIO_MOSI_SOURCE    GPIO_PinSource15
#define SPI2_GPIO_SCLK_PIN       GPIO_Pin_13
#define SPI2_GPIO_SCLK_SOURCE    GPIO_PinSource13
#define SPI2_GPIO_NSS_PIN        GPIO_Pin_12
#define SPI2_GPIO_NSS_SOURCE     GPIO_PinSource12
#define SPI2_GPIO_PIN_GROUP      GPIOB
#define SPI2_GPIO_PIN_RCC        RCC_AHBPeriph_GPIOB
#define SPI2_GPIO_PIN_AF         GPIO_AF_0
#define SPI2_RCC                 RCC_APB1Periph_SPI2
#define SPI2_DMA_RCC             RCC_AHBPeriph_DMA1
#ifdef RT_USING_SPI2_TX_DMA
#define SPI2_DMA_TX              DMA1_Channel5
#else
#define SPI2_DMA_TX              RT_NULL
static uint8_t stm32_spi2_tx_buf[SPI_XFER_ONEC_MAX_LEN+1];
static struct rt_ringbuffer stm32_spi2_tx_rb;
#endif /* RT_USING_SPI2_TX_DMA */
#ifdef RT_USING_SPI2_RX_DMA
#define SPI2_DMA_RX              DMA1_Channel4
static uint8_t stm32_spi2_rx_dma_buf[SPI_XFER_ONEC_MAX_LEN* SPI_RX_DMA_FRAME_NUM];
#else
#define SPI2_DMA_RX              RT_NULL
static uint8_t stm32_spi2_rx_buf[SPI_XFER_ONEC_MAX_LEN+1];
static struct rt_ringbuffer stm32_spi2_rx_rb;
#endif /* RT_USING_SPI2_RX_DMA */

struct stm32_spi_cs {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};

struct stm32_spi_bus {
    /* must be first */
    struct rt_spi_bus parent;

    SPI_TypeDef* spix;
    SPI_InitTypeDef init;
    struct stm32_spi_cs* cs;
    void* tx_buf;
    void* rx_buf;
    void (*TxISR)(struct stm32_spi_bus* bus);
    void (*RxISR)(struct stm32_spi_bus* bus);
    DMA_Channel_TypeDef* tx_dma;
    DMA_Channel_TypeDef* rx_dma;
    void (*TxDMAISR)(struct stm32_spi_bus* bus);
    void (*RxDMAISR)(struct stm32_spi_bus* bus);
    uint8 volatile tx_dma_tc_flag;
    uint8 volatile rx_dma_tc_flag;
};

rt_inline uint16_t stm32_spi_bus_get_DataSize(rt_uint8_t data_width)
{
    uint16_t SPI_DataSize = 0;

    RT_ASSERT((data_width == 8) ||
              (data_width == 16));

    if(data_width <= 8) {
        SPI_DataSize = SPI_DataSize_8b;
    }
    else {
        SPI_DataSize = SPI_DataSize_16b;
    }

    return SPI_DataSize;
}

rt_inline uint16_t stm32_spi_bus_get_BaudRatePrescaler(rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler = 0;

    RT_ASSERT((max_hz <= SystemCoreClock/2) ||
              (max_hz >= SystemCoreClock/256))

    /* STM32F0xx SPI MAX fpclk/2 = 24Mhz */
    if(max_hz == SystemCoreClock/2) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if(max_hz >= SystemCoreClock/4) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if(max_hz >= SystemCoreClock/8) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if(max_hz >= SystemCoreClock/16) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if(max_hz >= SystemCoreClock/32) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if(max_hz >= SystemCoreClock/64) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if(max_hz >= SystemCoreClock/128) {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }

    return SPI_BaudRatePrescaler;
}

rt_inline uint16_t stm32_spi_bus_get_CPOL(rt_uint8_t mode)
{
    uint16_t SPI_CPOL = 0;

    if(mode & RT_SPI_CPOL) {
        SPI_CPOL = SPI_CPOL_High;
    }
    else {
        SPI_CPOL = SPI_CPOL_Low;
    }

    return SPI_CPOL;
}

rt_inline uint16_t stm32_spi_bus_get_CPHA(rt_uint8_t mode)
{
    uint16_t SPI_CPHA = 0;

    if(mode & RT_SPI_CPHA) {
        SPI_CPHA = SPI_CPHA_2Edge;
    }
    else {
        SPI_CPHA = SPI_CPHA_1Edge;
    }

    return SPI_CPHA;
}

rt_inline uint16_t stm32_spi_bus_get_FirstBit(rt_uint8_t mode)
{
    uint16_t SPI_FirstBit = 0;

    if(mode & RT_SPI_MSB) {
        SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else {
        SPI_FirstBit = SPI_FirstBit_LSB;
    }

    return SPI_FirstBit;
}

rt_inline uint16_t stm32_spi_bus_get_Mode(rt_uint8_t mode)
{
    uint16_t SPI_Mode = 0;

    if(mode & RT_SPI_SLAVE) {
        SPI_Mode = SPI_Mode_Slave;
    }
    else {
        SPI_Mode = SPI_Mode_Master;
    }

    return SPI_Mode;
}

rt_inline void stm32_spi_bus_get_InitTypeDef_from_configuration(SPI_InitTypeDef* SPI_Init, struct rt_spi_configuration* cfg)
{
    SPI_Init->SPI_DataSize = stm32_spi_bus_get_DataSize(cfg->data_width);
    SPI_Init->SPI_BaudRatePrescaler = stm32_spi_bus_get_BaudRatePrescaler(cfg->max_hz);
    SPI_Init->SPI_CPOL = stm32_spi_bus_get_CPOL(cfg->mode);
    SPI_Init->SPI_CPHA = stm32_spi_bus_get_CPHA(cfg->mode);
    SPI_Init->SPI_FirstBit = stm32_spi_bus_get_FirstBit(cfg->mode);
    SPI_Init->SPI_Mode = stm32_spi_bus_get_Mode(cfg->mode);
}

rt_inline rt_bool_t stm32_spi_bus_is_master(struct stm32_spi_bus* bus)
{
    return (SPI_Mode_Master==bus->init->SPI_DataSize)? RT_TRUE: RT_FALSE;
}

rt_inline rt_bool_t stm32_spi_bus_is_8bits(struct stm32_spi_bus* bus)
{
    return (SPI_DataSize_8b==bus->init->SPI_Mode)? RT_TRUE: RT_FALSE;
}

rt_inline rt_bool_t stm32_spi_bus_is_tx_dma(struct stm32_spi_bus* bus)
{
    return (RT_NULL==bus->tx_dma)? RT_FALSE: RT_TRUE;
}

rt_inline rt_bool_t stm32_spi_bus_is_rx_dma(struct stm32_spi_bus* bus)
{
    return (RT_NULL==bus->rx_dma)? RT_FALSE: RT_TRUE;
}

static void stm32_spi_bus_cs_pin_init(struct stm32_spi_bus* bus)
{
    GPIO_InitTypeDef SPI_GPIO;
    GPIO_StructInit(&SPI_GPIO);
    if(bus->spix == SPI1) {
        SPI_GPIO.GPIO_Pin = SPI1_GPIO_NSS_PIN;
        SPI_GPIO.GPIO_Mode = GPIO_Mode_AF;
        SPI_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
        SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SPI1_GPIO_PIN_GROUP, &SPI_GPIO);
        GPIO_PinAFConfig(SPI1_GPIO_PIN_GROUP, SPI1_GPIO_NSS_SOURCE, SPI1_GPIO_PIN_AF);
    }
    else if(bus->spix == SPI2) {
        SPI_GPIO.GPIO_Pin = SPI2_GPIO_NSS_PIN;
        SPI_GPIO.GPIO_Mode = GPIO_Mode_AF;
        SPI_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
        SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SPI2_GPIO_PIN_GROUP, &SPI_GPIO);
        GPIO_PinAFConfig(SPI2_GPIO_PIN_GROUP, SPI2_GPIO_NSS_SOURCE, SPI2_GPIO_PIN_AF);
    }
}

static rt_err_t stm32_spi_bus_configure(struct rt_spi_device* dev, struct rt_spi_configuration* cfg)
{
    struct stm32_spi_bus* spi_bus = (struct stm32_spi_bus*)dev->bus;
    SPI_TypeDef* SPIx = spi_bus->spix;

    RT_ASSERT(cfg != RT_NULL);

    SPI_StructInit(&spi_bus->init);
    stm32_spi_bus_get_InitTypeDef_from_configuration(&spi_bus->init, cfg);
    SPI_Init->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_Init->SPI_NSS  = SPI_NSS_Soft;
    if(dev->parent.user_data != RT_NULL) {
        struct stm32_spi_bus_cs* cs = (struct stm32_spi_bus_cs*)dev->parent.user_data;
        cs->init(cs);
    }
    else {
        stm32_spi_bus_cs_pin_init(spi_bus);
    }
    SPI_I2S_DeInit(SPIx);
    SPI_Init(SPIx, &spi_bus->init);
    SPI_CalculateCRC(SPIx, DISABLE);

    return RT_EOK;
}

rt_inline void stm32_spi_bus_take_cs(struct stm32_spi_bus* bus)
{
    if(bus->parent->owner->parent->user_data != RT_NULL) {
        struct stm32_spi_bus_cs* cs = (struct stm32_spi_bus_cs*)bus->parent->owner->parent->user_data;
        cs->take(cs);
    }
    else {
        SPI_NSSInternalSoftwareConfig(bus->spix, SPI_NSSInternalSoft_Reset);
    }
}

rt_inline void stm32_spi_bus_release_cs(struct stm32_spi_bus* bus)
{
    if(bus->parent->owner->parent->user_data != RT_NULL) {
        struct stm32_spi_bus_cs* cs = (struct stm32_spi_bus_cs*)bus->parent->owner->parent->user_data;
        cs->release(cs);
    }
    else {
        SPI_NSSInternalSoftwareConfig(bus->spix, SPI_NSSInternalSoft_Set);
    }
}

rt_inline rt_size_t stm32_spi_bus_set_bus_tx_buf(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf, rt_size_t len)
{
    rt_size_t n = 0;
    if(stm32_spi_bus_is_tx_dma(bus)) {
        bus->tx_buf = tx_buf;
        return len;
    }
    else {
        n = rt_ringbuffer_put((struct rt_ringbuffer*)bus->tx_buf, tx_buf, len);
        return n;
    }
}

rt_inline void stm32_spi_bus_set_bus_rx_buf(struct stm32_spi_bus* bus, rt_uint8_t* rx_buf)
{
    if(stm32_spi_bus_is_rx_dma(bus)) {
        bus->rx_buf = rx_buf;
    }
}

rt_inline rt_size_t stm32_spi_bus_get_bus_rx_buf(struct stm32_spi_bus* bus, rt_uint8_t* rx_buf, rt_size_t len)
{
    rt_size_t n = 0;
    if(stm32_spi_bus_is_rx_dma(bus)) {
        return len;
    }
    else {
        n = rt_ringbuffer_get((struct rt_ringbuffer*)bus->rx_buf, rx_buf, len);
        return n;
    }
}

rt_inline rt_err_t _stm32_spi_bus_transmit_dma_receive_dma(struct stm32_spi_bus* bus, rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;
    DMA_InitTypeDef SPI_DMA;
    rt_size_t dma_count = len;

    if(stm32_spi_bus_is_8bits(bus)) {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralDST;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->tx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->tx_dma, &SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
        SPI_DMA.DMA_MemoryBaseAddr = bus->rx_buf;
        DMA_Init(bus->rx_dma, &SPI_DMA);
    }
    else {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralDST;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count>>1;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->tx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->tx_dma, &SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
        SPI_DMA.DMA_MemoryBaseAddr = bus->rx_buf;
        DMA_Init(bus->rx_dma, &SPI_DMA);
    }
    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    DMA_ITConfig(bus->rx_dma, DMA_IT_TC, ENABLE);
    DMA_ITConfig(bus->tx_dma, DMA_IT_TC, ENABLE);
    DMA_Cmd(bus->rx_dma, ENABLE);
    DMA_Cmd(bus->tx_dma, ENABLE);

    while((bus->tx_dma_tc_flag==RT_FALSE)||(bus->rx_dma_tc_flag==RT_FALSE));
    bus->tx_dma_tc_flag==RT_FALSE;
    bus->rx_dma_tc_flag==RT_FALSE;

    return RT_EOK;
}

rt_inline rt_err_t _stm32_spi_bus_transmit_it_receive_dma(struct stm32_spi_bus* bus, rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;
    DMA_InitTypeDef SPI_DMA;
    rt_size_t dma_count = len;

    if(stm32_spi_bus_is_8bits(bus)) {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->rx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->rx_dma, &SPI_DMA);
    }
    else {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count>>1;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->rx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->rx_dma, &SPI_DMA);
    }
    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);
    DMA_ITConfig(bus->rx_dma, DMA_IT_TC, ENABLE);
    DMA_Cmd(bus->rx_dma, ENABLE);
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);

    while((rt_ringbuffer_data_len((struct rt_ringbuffer*)bus->tx_buf)!=0)||
          (bus->rx_dma_tc_flag==RT_FALSE));
    bus->rx_dma_tc_flag==RT_FALSE;

    return RT_EOK;
}

rt_inline rt_err_t _stm32_spi_bus_transmit_dma_receive_it(struct stm32_spi_bus* bus, rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;
    DMA_InitTypeDef SPI_DMA;
    rt_size_t dma_count = len;

    if(stm32_spi_bus_is_8bits(bus)) {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralDST;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->tx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->tx_dma, &SPI_DMA);
    }
    else {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
        DMA_StructInit(&SPI_DMA);
        SPI_DMA.DMA_DIR = DMA_DIR_PeripheralDST;
        SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
        SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        SPI_DMA.DMA_Mode = DMA_Mode_Normal;
        SPI_DMA.DMA_Priority = DMA_Priority_Low;
        SPI_DMA.DMA_BufferSize = dma_count>>1;
        SPI_DMA.DMA_M2M = DMA_M2M_Disable;
        SPI_DMA.DMA_MemoryBaseAddr = bus->tx_buf;
        SPI_DMA.DMA_PeripheralBaseAddr = SPIx->DR;
        DMA_Init(bus->tx_dma, &SPI_DMA);
    }
    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
    DMA_ITConfig(bus->tx_dma, DMA_IT_TC, ENABLE);
    DMA_Cmd(bus->tx_dma, ENABLE);

    while((rt_ringbuffer_data_len((struct rt_ringbuffer*)bus->rx_buf)!=len)||
          (bus->tx_dma_tc_flag==RT_FALSE));
    bus->tx_dma_tc_flag==RT_FALSE;

    return RT_EOK;
}

rt_inline rt_err_t _stm32_spi_bus_transmit_it_receive_it(struct stm32_spi_bus* bus, rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;

    if(stm32_spi_bus_is_8bits(bus)) {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
    }
    else {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
    }
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE | SPI_I2S_IT_RXNE, ENABLE);

    while((rt_ringbuffer_data_len((struct rt_ringbuffer*)bus->rx_buf)!=len)||
          (rt_ringbuffer_data_len((struct rt_ringbuffer*)bus->tx_buf)!=0));

    return RT_EOK;
}

rt_inline rt_err_t __stm32_spi_bus_transmit_receive(struct stm32_spi_bus* bus, rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;

    /* check SPI bus is busy or not */
    if(RESET != SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY)) {
        return -RT_EBUSY;
    }

    /* send data though dma/interrupt */
    if(stm32_spi_bus_is_tx_dma(bus) && stm32_spi_bus_is_rx_dma(bus)) {
        _stm32_spi_bus_transmit_dma_receive_dma(bus, len);
    }
    else if(!stm32_spi_bus_is_tx_dma(bus) && stm32_spi_bus_is_rx_dma(bus)) {
        _stm32_spi_bus_transmit_it_receive_dma(bus, len);
    }
    else if(stm32_spi_bus_is_tx_dma(bus) && !stm32_spi_bus_is_rx_dma(bus)) {
        _stm32_spi_bus_transmit_dma_receive_it(bus, len);
    }
    else {
        _stm32_spi_bus_transmit_it_receive_it(bus, len);
    }

    return RT_EOK;
}

static rt_err_t _stm32_spi_bus_transmit_receive(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf, rt_uint8_t* rx_buf)
{
    rt_size_t len = 0;
    rt_size_t xfern = 0;
    rt_uint8_t dump_tx_buf[SPI_XFER_ONEC_MAX_LEN];
    rt_uint8_t* tx_ptr = tx_buf;
    rt_uint8_t* rx_ptr = rx_buf;

    len = bus->xfer_size;

    while(len>0) {
        if(tx_buf==RT_NULL) {
            rt_memset(dump_tx_buf ,SPI_DUMP ,sizeof(dump_tx_buf));
            xfern = stm32_spi_bus_set_bus_tx_buf(bus, dump_tx_buf, len);
        }
        else {
            xfern = stm32_spi_bus_set_bus_tx_buf(bus, tx_ptr, len);
            tx_ptr += xfern;
        }
        stm32_spi_bus_set_bus_rx_buf(bus, rx_ptr);
        __stm32_spi_bus_transmit_receive(bus, xfern);
        if(rx_buf != RT_NULL) {
            stm32_spi_bus_get_bus_rx_buf(bus, rx_ptr, xfern);
            rx_ptr += xfern;
        }
        len -= xfern;
    }

    return RT_EOK;
}

rt_inline rt_err_t stm32_spi_bus_transmit_receive(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf, rt_uint8_t* rx_buf)
{
    if((bus->parent.parent.open_flag & RT_DEVICE_OFLAG_RDWR)==RT_DEVICE_OFLAG_RDWR) {
        return _stm32_spi_bus_transmit_receive(bus, tx_buf, rx_buf);
    }
    return -RT_EIO;
}

rt_inline rt_err_t stm32_spi_bus_transmit(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf)
{
    if(bus->parent.parent.open_flag & RT_DEVICE_OFLAG_WRONLY) {
        return _stm32_spi_bus_transmit_receive(bus, tx_buf, RT_NULL);
    }
    return -RT_EIO;
}

rt_inline rt_err_t stm32_spi_bus_receive(struct stm32_spi_bus* bus, rt_uint8_t* rx_buf)
{
    if(bus->parent.parent.open_flag & RT_DEVICE_OFLAG_RDONLY) {
        return _stm32_spi_bus_transmit_receive(bus, RT_NULL, rx_buf);
    }
    return -RT_EIO;
}

static rt_uint32_t stm32_spi_bus_xfer(struct rt_spi_device* dev, struct rt_spi_message* msg)
{
    rt_uint32_t ret = 0;
    struct stm32_spi_bus* spi_bus = (struct stm32_spi_bus*)dev->bus;
    SPI_TypeDef* SPIx = spi_bus->spix;

    /* enable spi bus */
    SPI_Cmd(SPIx, ENABLE);

    if(stm32_spi_bus_is_master(spi_bus) && msg->cs_take) {
        stm32_spi_bus_take_cs(spi_bus);
    }

    spi_bus->xfer_size = (stm32_spi_bus_is_8bits(spi_bus))? msg->length: msg->length<<1;

    if((RT_NULL!=msg->send_buf)&&(RT_NULL!=msg->recv_buf)) {
        if(RT_EOK !=stm32_spi_bus_transmit_receive(spi_bus,
                (const rt_uint8_t*)msg->send_buf,
                (rt_uint8_t*)msg->recv_buf)) {
            ret = 0;
            goto out;
        }
    }
    else {
        if(RT_NULL!=msg->send_buf) {
            if(RT_EOK !=stm32_spi_bus_transmit(spi_bus,
                                               (const rt_uint8_t*)msg->send_buf)) {
                ret = 0;
                goto out;
            }
        }
        if(RT_NULL!=msg->recv_buf) {
            if(RT_EOK !=stm32_spi_bus_receive(spi_bus,
                                              (rt_uint8_t*)msg->recv_buf)) {
                ret = 0;
                goto out;
            }
        }
    }
    ret = msg->length;

out:
    if(stm32_spi_bus_is_master(spi_bus) && msg->cs_release) {
        stm32_spi_bus_release_cs(spi_bus);
    }
    /* disable spi bus */
    SPI_Cmd(SPIx, DISABLE);

    return ret;
};

static struct rt_spi_ops stm32_spi_ops = {
    stm32_spi_bus_configure,
    stm32_spi_bus_xfer
};

static rt_err_t stm32_spi_bus_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;

    if(dev->type == RT_Device_Class_SPIBUS) {
        struct stm32_spi_bus* spi_bus = (struct stm32_spi_bus*)dev;
        if(spi_bus->spix == SPI1) {
            ret = stm32_spi_bus_device_init(spi_bus, "spi1");
            if(RT_EOK != ret) {
                return ret;
            }
        }

        if(spi_bus->spix == SPI2) {
            ret = stm32_spi_bus_device_init(spi_bus, "spi2");
            if(RT_EOK != ret) {
                return ret;
            }
        }
    }
    else {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t stm32_spi_bus_open(rt_device_t dev, rt_uint16_t oflag)
{
    if(dev->type == RT_Device_Class_SPIBUS) {
        /* not open as read or write, return error */
        if(!(oflag & RT_DEVICE_OFLAG_RDWR)) {
            return -RT_ERROR;
        }
        return RT_EOK;
    }
    else {
        return -RT_ERROR;
    }
}

static rt_err_t stm32_spi_bus_close(rt_device_t dev)
{
    if(dev->type == RT_Device_Class_SPIBUS) {
        struct stm32_spi_bus* bus = (struct stm32_spi_bus*)dev;
        if(RT_NULL != bus->rx_dma) { /* use dma */
            DMA_ITConfig(bus->rx_dma, DMA_IT_TC, DISABLE);
            DMA_Cmd(bus->rx_dma, DISABLE);
        }
        else { /* use interrupt */
            SPI_I2S_ITConfig(bus->spix, SPI_I2S_IT_RXNE, DISABLE);
        }
        if(RT_NULL != bus->tx_dma) { /* use dma */
            DMA_ITConfig(bus->tx_dma, DMA_IT_TC, DISABLE);
            DMA_Cmd(bus->tx_dma, DISABLE);
        }
        else { /* use interrupt */
            SPI_I2S_ITConfig(bus->spix, SPI_I2S_IT_TXE, DISABLE);
        }
        SPI_Cmd(bus->spix, DISABLE);
    }
    else {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t stm32_spi_bus_register(struct rt_spi_bus* spi_bus, const char* spi_bus_name)
{
    rt_err_t ret = RT_EOK;

    ret = rt_spi_bus_register(spi_bus, spi_bus_name, &stm32_spi_ops)
    if(RT_EOK != ret) {
        return ret;
    }
    ret = rt_mutex_take(&(spi_bus->lock), RT_WAITING_FOREVER);
    if (ret == RT_EOK) {
        spi_bus->parent.init = stm32_spi_bus_init;
        spi_bus->parent.open = stm32_spi_bus_open;
        spi_bus->parent.close = stm32_spi_bus_close;
        rt_mutex_release(&(spi_bus->lock));
    }

    return RT_EOK;
}

static void STM32_spi_bus_rx_dma_isr(struct stm32_spi_bus* bus)
{
    bus->rx_dma_tc_flag = RT_TRUE;
}

static void STM32_spi_bus_tx_dma_isr(struct stm32_spi_bus* bus)
{
    bus->tx_dma_tc_flag = RT_TRUE;
}

static void STM32_spi_bus_rx_isr(struct stm32_spi_bus* bus)
{
    if(stm32_spi_bus_is_8bits(bus)) {
        uint8_t data = SPI_ReceiveData8(bus->spix);
        rt_ringbuffer_put((struct rt_ringbuffer*)bus->rx_buf, &data, sizeof(uint8_t));
    }
    else {
        uint16_t data = SPI_I2S_ReceiveData16(bus->spix);
        rt_ringbuffer_put((struct rt_ringbuffer*)bus->rx_buf, &data, sizeof(uint16_t));
    }
}

static void STM32_spi_bus_tx_isr(struct stm32_spi_bus* bus)
{
    if(stm32_spi_bus_is_8bits(bus)) {
        uint8_t data = 0;
        rt_ringbuffer_get((struct rt_ringbuffer*)bus->tx_buf, &data, sizeof(uint8_t));
        SPI_SendData8(bus->spix, data);
    }
    else {
        uint16_t data = 0;
        rt_ringbuffer_get((struct rt_ringbuffer*)bus->tx_buf, &data, sizeof(uint16_t));
        SPI_I2S_SendData16(bus->spix, data);
    }
}

static rt_err_t stm32_spi_bus_device_init(struct stm32_spi_bus* spi_bus, const char* spi_bus_name)
{
    rt_err_t ret = RT_EOK;
    GPIO_InitTypeDef SPI_GPIO;

    ret = stm32_spi_bus_register(&(spi_bus->parent), spi_bus_name);
    if(RT_EOK != ret) {
        return ret;
    }

    if(spi_bus->spix == SPI1) {
        /* SPI1 RCC */
        RCC_APB2PeriphClockCmd(SPI1_RCC, ENABLE);

        /* SPI1 GPIO */
        RCC_AHBPeriphClockCmd(SPI1_GPIO_PIN_RCC, ENABLE);
        GPIO_StructInit(&SPI_GPIO);
        SPI_GPIO.GPIO_Pin = SPI1_GPIO_MISO_PIN | SPI1_GPIO_MOSI_PIN | SPI1_GPIO_SCLK_PIN;
        SPI_GPIO.GPIO_Mode = GPIO_Mode_AF;
        SPI_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
        SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SPI1_GPIO_PIN_GROUP, &SPI_GPIO);
        GPIO_PinAFConfig(SPI1_GPIO_PIN_GROUP, SPI1_GPIO_MISO_SOURCE, SPI1_GPIO_PIN_AF);
        GPIO_PinAFConfig(SPI1_GPIO_PIN_GROUP, SPI1_GPIO_MOSI_SOURCE, SPI1_GPIO_PIN_AF);
        GPIO_PinAFConfig(SPI1_GPIO_PIN_GROUP, SPI1_GPIO_SCLK_SOURCE, SPI1_GPIO_PIN_AF);

        /* SPI1 Rx DMA */
        if(RT_NULL != SPI1_DMA_RX) {
            RCC_AHBPeriphClockCmd(SPI1_DMA_RCC, ENABLE);
            spi_bus->rx_dma = SPI1_DMA_RX;
            spi_bus->RxDMAISR = STM32_spi_bus_rx_dma_isr;
            spi_bus->rx_dma_tc_flag = RT_FALSE;
            spi_bus->RxISR = RT_NULL;
        }
        else {
            spi_bus->rx_dma = RT_NULL;
            spi_bus->RxDMAISR = RT_NULL;
            spi_bus->rx_dma_tc_flag = RT_FALSE;
            spi_bus->RxISR = STM32_spi_bus_rx_isr;
            rt_ringbuffer_init(&stm32_spi1_rx_rb, stm32_spi1_rx_buf, SPI_XFER_ONEC_MAX_LEN);
            spi_bus->rx_buf = &stm32_spi1_rx_rb;
        }

        /* SPI1 Tx DMA */
        if(RT_NULL != SPI1_DMA_TX) {
            RCC_AHBPeriphClockCmd(SPI1_DMA_RCC, ENABLE);
            spi_bus->tx_dma = SPI1_DMA_TX;
            spi_bus->TxDMAISR = STM32_spi_bus_tx_dma_isr;
            spi_bus->tx_dma_tc_flag = RT_FALSE;
            spi_bus->TxISR = RT_NULL;
        }
        else {
            spi_bus->tx_dma = RT_NULL;
            spi_bus->TxDMAISR = RT_NULL;
            spi_bus->tx_dma_tc_flag = RT_FALSE;
            spi_bus->TxISR = STM32_spi_bus_tx_isr;
            rt_ringbuffer_init(&stm32_spi1_tx_rb, stm32_spi1_tx_buf, SPI_XFER_ONEC_MAX_LEN);
            spi_bus->tx_buf = &stm32_spi1_tx_rb;
        }
    }
    else if(spi_bus->spix == SPI2) {
        /* SPI2 RCC */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

        /* SPI2 GPIO */
        RCC_AHBPeriphClockCmd(SPI2_GPIO_PIN_RCC, ENABLE);
        GPIO_StructInit(&SPI_GPIO);
        SPI_GPIO.GPIO_Pin = SPI2_GPIO_MISO_PIN | SPI2_GPIO_MOSI_PIN | SPI2_GPIO_SCLK_PIN;
        SPI_GPIO.GPIO_Mode = GPIO_Mode_AF;
        SPI_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
        SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SPI2_GPIO_PIN_GROUP, &SPI_GPIO);
        GPIO_PinAFConfig(SPI2_GPIO_PIN_GROUP, SPI2_GPIO_MISO_SOURCE, SPI2_GPIO_PIN_AF);
        GPIO_PinAFConfig(SPI2_GPIO_PIN_GROUP, SPI2_GPIO_MOSI_SOURCE, SPI2_GPIO_PIN_AF);
        GPIO_PinAFConfig(SPI2_GPIO_PIN_GROUP, SPI2_GPIO_SCLK_SOURCE, SPI2_GPIO_PIN_AF);

        /* SPI2 Rx DMA */
        if(RT_NULL != SPI2_DMA_RX) {
            RCC_AHBPeriphClockCmd(SPI2_DMA_RCC, ENABLE);
            spi_bus->rx_dma = SPI2_DMA_RX;
            spi_bus->RxDMAISR = STM32_spi_bus_rx_dma_isr;
            spi_bus->rx_dma_tc_flag = RT_FALSE;
            spi_bus->RxISR = RT_NULL;
        }
        else {
            spi_bus->rx_dma = RT_NULL;
            spi_bus->RxDMAISR = RT_NULL;
            spi_bus->rx_dma_tc_flag = RT_FALSE;
            spi_bus->RxISR = STM32_spi_bus_rx_isr;
            rt_ringbuffer_init(&stm32_spi2_rx_rb, stm32_spi2_rx_buf, SPI_XFER_ONEC_MAX_LEN);
            spi_bus->rx_buf = &stm32_spi2_rx_rb;
        }

        /* SPI2 Tx DMA */
        if(RT_NULL != SPI2_DMA_TX) {
            RCC_AHBPeriphClockCmd(SPI2_DMA_RCC, ENABLE);
            spi_bus->tx_dma = SPI2_DMA_TX;
            spi_bus->TxDMAISR = STM32_spi_bus_tx_dma_isr;
            spi_bus->tx_dma_tc_flag = RT_FALSE;
            spi_bus->TxISR = RT_NULL;
        }
        else {
            spi_bus->tx_dma = RT_NULL;
            spi_bus->TxDMAISR = RT_NULL;
            spi_bus->tx_dma_tc_flag = RT_FALSE;
            spi_bus->TxISR = STM32_spi_bus_tx_isr;
            rt_ringbuffer_init(&stm32_spi2_tx_rb, stm32_spi2_tx_buf, SPI_XFER_ONEC_MAX_LEN);
            spi_bus->tx_buf = &stm32_spi2_tx_rb;
        }
    }
    else {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}

rt_err_t rt_hw_spi_bus_init(void)
{
    rt_err_t ret = RT_EOK;

#ifdef RT_USING_SPI1
    stm32_spi_bus_1.spix = SPI1;
    ret = stm32_spi_bus_device_init(stm32_spi_bus_1, "spi1");
    if(RT_EOK != ret) {
        return ret;
    }
    stm32_spi_bus_1.parent.parent.flag |= RT_DEVICE_FLAG_ACTIVATED;
#endif /* RT_USING_SPI1 */

#ifdef RT_USING_SPI2
    stm32_spi_bus_2.spix = SPI2;
    ret = stm32_spi_bus_device_init(stm32_spi_bus_2, "spi2");
    if(RT_EOK != ret) {
        return ret;
    }
    stm32_spi_bus_2.parent.parent.flag |= RT_DEVICE_FLAG_ACTIVATED;
#endif /* RT_USING_SPI2 */

    return RT_EOK;
}

#ifdef USING_SPI1
static struct stm32_spi_bus stm32_spi_bus_1;
void SPI1_IRQHandler(void)
{
    struct stm32_spi_bus* bus = &stm32_spi_bus_1;

    /* enter interrupt */
    rt_interrupt_enter();

    if(SPI_I2S_GetITStatus(bus->spix, SPI_I2S_IT_TXE) != RESET) {
        bus->TxISR(bus);
    }
    if(SPI_I2S_GetITStatus(bus->spix, SPI_I2S_IT_RXNE) != RESET) {
        bus->RxISR(bus);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
void DMA1_Channel2_3_IRQHandler(void)
{
    struct stm32_spi_bus* bus = &stm32_spi_bus_1;

    /* enter interrupt */
    rt_interrupt_enter();

    if(DMA_GetITStatus(DMA1_IT_TC2) != RESET) {
        bus->RxDMAISR(bus);
        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
    if(DMA_GetITStatus(DMA1_IT_TC3) != RESET) {
        bus->TxDMAISR(bus);
        DMA_ClearITPendingBit(DMA1_IT_TC3);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct stm32_spi_bus stm32_spi_bus_2;
void SPI2_IRQHandler(void)
{
    struct stm32_spi_bus* bus = &stm32_spi_bus_2;

    /* enter interrupt */
    rt_interrupt_enter();

    if(SPI_I2S_GetITStatus(bus->spix, SPI_I2S_IT_TXE) != RESET) {
        bus->TxISR(bus);
    }
    if(SPI_I2S_GetITStatus(bus->spix, SPI_I2S_IT_RXNE) != RESET) {
        bus->RxISR(bus);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
void DMA1_Channel4_5_IRQHandler(void)
{
    struct stm32_spi_bus* bus = &stm32_spi_bus_2;

    /* enter interrupt */
    rt_interrupt_enter();

    if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) {
        bus->RxDMAISR(bus);
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
    if(DMA_GetITStatus(DMA1_IT_TC5) != RESET) {
        bus->TxDMAISR(bus);
        DMA_ClearITPendingBit(DMA1_IT_TC5);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* #ifdef USING_SPI2 */

