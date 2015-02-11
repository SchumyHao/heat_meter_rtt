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
#define SPI_XFER_ONEC_MAX_LEN (32)
#define SPI_RX_DMA_FRAME_NUM  (4)

/* SPI1 */
#define SPI1_GPIO_MISO_PIN       GPIO_Pin_6
#define SPI1_GPIO_MISO_SOURCE    GPIO_PinSource6
#define SPI1_GPIO_MOSI_PIN       GPIO_Pin_7
#define SPI1_GPIO_MOSI_SOURCE    GPIO_PinSource7
#define SPI1_GPIO_SCLK_PIN       GPIO_Pin_5
#define SPI1_GPIO_SCLK_SOURCE    GPIO_PinSource5
#define SPI1_GPIO_PIN_GROUP      GPIOA
#define SPI1_GPIO_PIN_AF         GPIO_AF_0
#define SPI1_GPIO_NSS_PIN        GPIO_Pin_4
#define SPI1_GPIO_NSS_GROUP      GPIOA
#ifdef RT_USING_SPI1_TX_DMA
#define SPI1_DMA_TX              DMA1_Channel3
#else
#define SPI1_DMA_TX              RT_NULL
#endif /* RT_USING_SPI1_TX_DMA */
#ifdef RT_USING_SPI1_RX_DMA
#define SPI1_DMA_RX              DMA1_Channel2
static uint8_t stm32_spi1_rx_dma_buf[SPI_XFER_ONEC_MAX_LEN*SPI_RX_DMA_FRAME_NUM];
#else
#define SPI1_DMA_RX              RT_NULL
#endif /* RT_USING_SPI1_RX_DMA */

/* SPI2 */
#define SPI2_GPIO_MISO_PIN       GPIO_Pin_14
#define SPI2_GPIO_MISO_SOURCE    GPIO_PinSource14
#define SPI2_GPIO_MOSI_PIN       GPIO_Pin_15
#define SPI2_GPIO_MOSI_SOURCE    GPIO_PinSource15
#define SPI2_GPIO_SCLK_PIN       GPIO_Pin_13
#define SPI2_GPIO_SCLK_SOURCE    GPIO_PinSource13
#define SPI2_GPIO_PIN_GROUP      GPIOB
#define SPI2_GPIO_PIN_AF         GPIO_AF_0
#define SPI2_GPIO_NSS_PIN        GPIO_Pin_12
#define SPI2_GPIO_NSS_GROUP      GPIOB
#ifdef RT_USING_SPI2_TX_DMA
#define SPI2_DMA_TX              DMA1_Channel5
#else
#define SPI2_DMA_TX              RT_NULL
#endif /* RT_USING_SPI2_TX_DMA */
#ifdef RT_USING_SPI2_RX_DMA
#define SPI2_DMA_RX              DMA1_Channel4
static uint8_t stm32_spi2_rx_dma_buf[SPI_XFER_ONEC_MAX_LEN*SPI_RX_DMA_FRAME_NUM];
#else
#define SPI2_DMA_RX              RT_NULL
#endif /* RT_USING_SPI2_RX_DMA */

struct stm32_spi_cs {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};

struct stm32_spi_bus {
	struct rt_spi_bus parent;

    SPI_TypeDef* spix;
    SPI_InitTypeDef init;
    struct stm32_spi_cs* cs;
    void* tx_buf;
    //uint32_t tx_xfer_size;
    //uint32_t tx_buf_count;
    void* rx_buf;
    uint32_t xfer_size;
    //uint32_t rx_xfer_size;
    //uint32_t rx_buf_count;
    void (*TxISR)(struct stm32_spi_bus* bus);
    void (*RxISR)(struct stm32_spi_bus* bus);
    struct rt_dma_device* tx_dma;
    struct rt_dma_device* rx_dma;

};

rt_inline uint16_t get_spi_DataSize(rt_uint8_t data_width)
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

rt_inline uint16_t get_spi_BaudRatePrescaler(rt_uint32_t max_hz)
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

rt_inline uint16_t get_spi_CPOL(rt_uint8_t mode)
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

rt_inline uint16_t get_spi_CPHA(rt_uint8_t mode)
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

rt_inline uint16_t get_spi_FirstBit(rt_uint8_t mode)
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

rt_inline uint16_t get_spi_Mode(rt_uint8_t mode)
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

rt_inline void get_spi_InitTypeDef_from_configuration(SPI_InitTypeDef* SPI_Init, struct rt_spi_configuration* cfg)
{
    SPI_Init->SPI_DataSize = get_spi_DataSize(cfg->data_width);
    SPI_Init->SPI_BaudRatePrescaler = get_spi_BaudRatePrescaler(cfg->max_hz);
    SPI_Init->SPI_CPOL = get_spi_CPOL(cfg->mode);
    SPI_Init->SPI_CPHA = get_spi_CPHA(cfg->mode);
    SPI_Init->SPI_FirstBit = get_spi_FirstBit(cfg->mode);
    SPI_Init->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_Init->SPI_Mode = get_spi_Mode(cfg->mode);
    SPI_Init->SPI_NSS  = SPI_NSS_Soft;
}

rt_inline rt_bool_t is_spi_master(struct stm32_spi_bus* bus)
{
    return (SPI_Mode_Master==bus->init->SPI_DataSize)? RT_TRUE: RT_FALSE;
}

rt_inline rt_bool_t is_spi_8bits(struct stm32_spi_bus* bus)
{
    return (SPI_DataSize_8b==bus->init->SPI_Mode)? RT_TRUE: RT_FALSE;
}

rt_inline rt_bool_t is_spi_tx_dma(struct stm32_spi_bus* bus)
{
    return (RT_NULL==bus->tx_dma)? RT_FALSE: RT_TRUE;
}

rt_inline rt_bool_t is_spi_rx_dma(struct stm32_spi_bus* bus)
{
    return (RT_NULL==bus->rx_dma)? RT_FALSE: RT_TRUE;
}

static rt_err_t configure(struct rt_spi_device* dev, struct rt_spi_configuration* cfg)
{
    struct stm32_spi_bus* spi_bus = NULL;
    SPI_TypeDef* SPIx = NULL;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(dev->bus != RT_NULL);
    RT_ASSERT(dev->bus->parent.user_data != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    spi_bus = (struct stm32_spi_bus*)dev->bus->parent.user_data;
    SPIx = spi_bus->spix;

    SPI_StructInit(&spi_bus->init);
    get_spi_InitTypeDef_from_configuration(&spi_bus->init, cfg);

    /* init SPI bus */
    SPI_I2S_DeInit(SPIx);
    SPI_Init(SPIx, &spi_bus->init);
    /* Enable SPI */
    SPI_CalculateCRC(SPIx, DISABLE);
    if(!is_spi_master(spi_bus)) {
        SPI_Cmd(SPIx, ENABLE);
    }

    return RT_EOK;
}

rt_inline void take_cs(struct stm32_spi_bus* bus)
{
    RT_ASSERT(bus->cs != RT_NULL);
    GPIO_ResetBits(bus->cs->GPIOx, bus->cs->GPIO_Pin);
}

rt_inline void release_cs(struct stm32_spi_bus* bus)
{
    RT_ASSERT(bus->cs != RT_NULL);
    GPIO_SetBits(bus->cs->GPIOx, bus->cs->GPIO_Pin);
}

rt_inline rt_size_t stm32_spi_set_tx_buf(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf, rt_size_t len)
{
    rt_size_t n = 0;
    if(is_spi_tx_dma(bus)) {
        bus->tx_buf = tx_buf;
        return (len>SPI_XFER_ONEC_MAX_LEN)? SPI_XFER_ONEC_MAX_LEN: len;
    }
    else {
        n = rt_ringbuffer_put(bus->tx_buf, tx_buf, len);
        return n;
    }
}

rt_inline rt_size_t stm32_spi_set_rx_buf(struct stm32_spi_bus* bus, rt_uint8_t* rx_buf, rt_size_t len)
{
    rt_size_t n = 0;
    if(is_spi_rx_dma(bus)) {
        bus->rx_buf = rx_buf;
        return (len>SPI_XFER_ONEC_MAX_LEN)? SPI_XFER_ONEC_MAX_LEN: len;
    }
    else {
        n = rt_ringbuffer_get(bus->rx_buf, rx_buf, len);
        return n;
    }
}

rt_inline rt_err_t _stm32_spi_8bit_transmit(struct stm32_spi_bus* bus, const rt_size_t len)
{
    SPI_TypeDef* SPIx = bus->spix;
    rt_size_t txn,rxn = 0;
    uint16_t tx16,rx16 = 0;
    uint8_t tx8,rx8 = 0;

    if(is_spi_tx_dma(bus)) { /* tx dma mode */
        while(len > txn) {

        }
    }
    else { /* tx interrupt mode */
        while(len > txn) {
            if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)) {
                if(len-txn > 1) {
                    rt_ringbuffer_get(bus->tx_buf, (rt_uint8_t*)&tx16, sizeof(uint16_t));
                    SPI_I2S_SendData16(SPIx, tx16);
                    txn += sizeof(uint16_t);
                }
                else {
                    rt_ringbuffer_get(bus->tx_buf, (rt_uint8_t*)&tx8, sizeof(uint8_t));
                    SPI_SendData8(SPIx, tx8);
                    txn += sizeof(uint8_t);
                }
            }
        }

    }
}

rt_inline rt_err_t _stm32_spi_transmit_dma_receive_dma(struct stm32_spi_bus* bus)
{
    SPI_TypeDef* SPIx = bus->spix;
    rt_size_t dma_count = bus->xfer_size;

    if(is_spi_8bits(bus)) {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        if((bus->xfer_size & 0x1) == 0x0)) {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxEvenRxEven);
            dma_count = dma_count >> 1;
        }
        else {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxOddRxOdd);
            dma_count = (dma_count >> 1) + 1;
        }
    }
    else {
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
    }

    bus->rx_dma->xfer_half_cplt_callback = stm32_spi_dma_half_rx_cplt;
    bus->rx_dma->xfer_cplt_callback = stm32_spi_dma_rx_cplt;
    bus->rx_dma->xfer_error_callback = stm32_spi_dma_rx_error;
    stm32_dma_start_it(bus->rx_dma, &SPIx->DR, bus->rx_buf, dma_count);

    bus->tx_dma->xfer_error_callback = stm32_spi_dma_tx_error;
    stm32_dma_start_it(bus->tx_dma, bus->rx_buf, &SPIx->DR, dma_count);

    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx|SPI_I2S_DMAReq_Rx, ENABLE);
}

rt_inline rt_err_t _stm32_spi_transmit_it_receive_dma(struct stm32_spi_bus* bus)
{
    SPI_TypeDef* SPIx = bus->spix;
    rt_size_t dma_count = bus->xfer_size;

    if(is_spi_8bits(bus)) {
        bus->TxISR = stm32_spi_tx_isr_8bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        if((bus->xfer_size & 0x1) == 0x0)) {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxEvenRxEven);
            dma_count = dma_count >> 1;
        }
        else {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxOddRxOdd);
            dma_count = (dma_count >> 1) + 1;
        }
    }
    else {
        bus->TxISR = stm32_spi_tx_isr_16bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
    }

    bus->rx_dma->xfer_half_cplt_callback = stm32_spi_dma_half_rx_cplt;
    bus->rx_dma->xfer_cplt_callback = stm32_spi_dma_rx_cplt;
    bus->rx_dma->xfer_error_callback = stm32_spi_dma_rx_error;
    stm32_dma_start_it(bus->rx_dma, &SPIx->DR, bus->rx_buf, dma_count);

    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE|SPI_I2S_IT_ERR, ENABLE);
}

rt_inline rt_err_t _stm32_spi_transmit_dma_receive_it(struct stm32_spi_bus* bus)
{
    SPI_TypeDef* SPIx = bus->spix;
    rt_size_t dma_count = bus->xfer_size;

    if(is_spi_8bits(bus)) {
        bus->RxISR = stm32_spi_rx_isr_8bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
        if((bus->xfer_size & 0x1) == 0x0)) {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxEvenRxEven);
            dma_count = dma_count >> 1;
        }
        else {
            SPI_LastDMATransferCmd(SPIx, SPI_LastDMATransfer_TxOddRxOdd);
            dma_count = (dma_count >> 1) + 1;
        }
    }
    else {
        bus->RxISR = stm32_spi_rx_isr_16bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
    }

    bus->tx_dma->xfer_error_callback = stm32_spi_dma_tx_error;
    stm32_dma_start_it(bus->tx_dma, bus->rx_buf, &SPIx->DR, dma_count);

    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, ENABLE);
    SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);
}

rt_inline rt_err_t _stm32_spi_transmit_it_receive_it(struct stm32_spi_bus* bus)
{
    SPI_TypeDef* SPIx = bus->spix;

    if(is_spi_8bits(bus)) {
        bus->TxISR = stm32_spi_tx_isr_8bit;
        bus->RxISR = stm32_spi_rx_isr_8bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
    }
    else {
        bus->TxISR = stm32_spi_tx_isr_16bit;
        bus->RxISR = stm32_spi_rx_isr_16bit;
        SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_HF);
    }

    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, ENABLE);
}

rt_inline rt_err_t _stm32_spi_transmit_receive(struct stm32_spi_bus* bus)
{
    SPI_TypeDef* SPIx = bus->spix;

    /* check SPI bus is busy or not */
    if(RESET != SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY)) {
        return -RT_EBUSY;
    }

    /* send data though dma/interrupt */
    if(is_spi_tx_dma(bus) && is_spi_rx_dma(bus)) {
        _stm32_spi_transmit_dma_receive_dma(bus);
    }
    else if(!is_spi_tx_dma(bus) && is_spi_rx_dma(bus)) {
        _stm32_spi_transmit_it_receive_dma(bus);
    }
    else if(is_spi_tx_dma(bus) && !is_spi_rx_dma(bus)) {
        _stm32_spi_transmit_dma_receive_it(bus);
    }
    else {
        _stm32_spi_transmit_it_receive_it(bus);
    }

    return RT_EOK;
}

static rt_err_t stm32_spi_transmit_receive(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf, rt_uint8_t* rx_buf)
{
    rt_size_t len = 0;
    rt_size_t xfern = 0;
    rt_uint8_t dump_tx_buf[SPI_XFER_ONEC_MAX_LEN];
    rt_uint8_t* tx_ptr = tx_buf;
    rt_uint8_t* rx_ptr = rx_buf;

    len = bus->xfer_size;

    if(tx_buf==RT_NULL) {
        memset(dump_tx_buf ,SPI_DUMP ,sizeof(dump_tx_buf[0])*SPI_XFER_ONEC_MAX_LEN);
        //len = bus->rx_xfer_size;
    }
    //else {
    //    len = bus->tx_xfer_size;
    //}

    /* slave mode, spi bus is enable all time, master mode enable if is required */
    if(is_spi_master(bus)) {
        SPI_Cmd(SPIx, ENABLE);
    }

    while(len>0) {
        if(tx_buf==RT_NULL) {
            xfern = stm32_spi_set_tx_buf(bus, dump_tx_buf, len);
        }
        else {
            xfern = stm32_spi_set_tx_buf(bus, tx_ptr, len);
            tx_ptr += xfern;
        }
        bus->rx_buf = rx_ptr;
        _stm32_spi_transmit_receive(bus);
        if(rx_buf == RT_NULL) {
            ;
        }
        else {
            stm32_spi_set_rx_buf(bus, rx_ptr, xfern);
            rx_ptr += xfern;
        }
        len -= xfern;
    }

    /* turn off spi bus if in master mode. */
    if(is_spi_master(bus)) {
        SPI_Cmd(SPIx, DISABLE);
    }

    return RT_EOK;
}

rt_inline rt_err_t stm32_spi_transmit(struct stm32_spi_bus* bus, const rt_uint8_t* tx_buf)
{
    return stm32_spi_transmit_receive(bus, tx_buf, RT_NULL);
}

rt_inline rt_err_t stm32_spi_receive(struct stm32_spi_bus* bus, rt_uint8_t* rx_buf)
{
    return stm32_spi_transmit_receive(bus, RT_NULL, rx_buf);
}

static rt_uint32_t xfer(struct rt_spi_device* dev, struct rt_spi_message* msg)
{
    struct stm32_spi_bus* spi_bus = NULL;
    SPI_TypeDef* SPIx = NULL;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(dev->bus != RT_NULL);
    RT_ASSERT(dev->bus->parent.user_data != RT_NULL);
    RT_ASSERT(msg != RT_NULL);

    spi_bus = (struct stm32_spi_bus*)dev->bus->parent.user_data;
    SPIx = spi_bus->spix;

    if(is_spi_master(spi_bus) && msg->cs_take) {
        take_cs(spi_bus);
    }

    spi_bus->xfer_size = (is_spi_8bits(spi_bus))? msg->length: msg->length<<1;

    if((RT_NULL!=msg->send_buf)&&(RT_NULL!=msg->recv_buf)) {
        //spi_bus->rx_xfer_size = (is_spi_8bits(spi_bus))? msg->length: msg->length<<1;
        //spi_bus->tx_xfer_size = (is_spi_8bits(spi_bus))? msg->length: msg->length<<1;
        stm32_spi_transmit_receive(spi_bus, (const rt_uint8_t*)msg->send_buf, (rt_uint8_t*)msg->recv_buf);
    }
    else {
        if(RT_NULL!=msg->send_buf) {
            //spi_bus->rx_xfer_size = 0;
            //spi_bus->tx_xfer_size = (is_spi_8bits(spi_bus))? msg->length: msg->length<<1;

            stm32_spi_transmit(spi_bus, (const rt_uint8_t*)msg->send_buf);
        }
        if(RT_NULL!=msg->recv_buf) {
            //spi_bus->rx_xfer_size = (is_spi_8bits(spi_bus))? msg->length: msg->length<<1;
            //spi_bus->tx_xfer_size = 0;
            stm32_spi_receive(spi_bus, (rt_uint8_t*)msg->recv_buf);
        }
    }

    if(is_spi_master(spi_bus) && msg->cs_release) {
        release_cs(spi_bus);
    }

    return msg->length;
};

static struct rt_spi_ops stm32_spi_ops = {
    configure,
    xfer
};

#ifdef USING_SPI1
static struct stm32_spi_bus stm32_spi_bus_1;
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct stm32_spi_bus stm32_spi_bus_2;
#endif /* #ifdef USING_SPI2 */

static rt_err_t stm32_spi_bus_device_init(struct rt_spi_bus* spi_bus, const char* spi_bus_name){
	rt_err_t ret = RT_EOK;
	
	ret = rt_spi_bus_register(spi_bus, spi_bus_name, &stm32_spi_ops)
	if(RT_EOK != ret){
		return ret;
	}
	ret = rt_mutex_take(&(spi_bus->lock), RT_WAITING_FOREVER);
    if (ret == RT_EOK){
		spi_bus->parent.init = stm32_spi_bus_init;
		spi_bus->parent.open = stm32_spi_bus_open;
		spi_bus->parent.close = stm32_spi_bus_close;
        rt_mutex_release(&(spi_bus->lock));
    }

	return RT_EOK;
}

static rt_err_t stm32_spi_bus_register(struct stm32_spi_bus* spi_bus, const char* spi_bus_name)
{
    rt_err_t ret = RT_EOK;
	GPIO_InitTypeDef SPI_GPIO;
	DMA_InitTypeDef SPI_DMA;  

	ret = stm32_spi_bus_device_init(&(spi_bus->parent), spi_bus_name);
	if(RT_EOK != ret){
		return ret;
	}

    if(spi_bus->spix == SPI1) {
		/* SPI1 GPIO */
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
		if(RT_NULL != SPI1_DMA_RX){
			DMA_StructInit(&SPI_DMA);
			SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
			SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
			SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
			SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			SPI_DMA.DMA_Mode = DMA_Mode_Circular;
			SPI_DMA.DMA_Priority = DMA_Priority_Low;
			SPI_DMA.DMA_BufferSize = SPI_XFER_ONEC_MAX_LEN*SPI_RX_DMA_FRAME_NUM;
			SPI_DMA.DMA_M2M = DMA_M2M_Disable;
			SPI_DMA.DMA_MemoryBaseAddr = stm32_spi1_rx_dma_buf;
			SPI_DMA.DMA_PeripheralBaseAddr = spi_bus->spix->DR;
			DMA_Init(SPI1_DMA_RX, &SPI_DMA);
		}
    }
    else if(spi_bus->spix == SPI2) {
		/* SPI2 GPIO */
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
		if(RT_NULL != SPI2_DMA_RX){
			DMA_StructInit(&SPI_DMA);
			SPI_DMA.DMA_DIR = DMA_DIR_PeripheralSRC;
			SPI_DMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			SPI_DMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
			SPI_DMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
			SPI_DMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			SPI_DMA.DMA_Mode = DMA_Mode_Circular;
			SPI_DMA.DMA_Priority = DMA_Priority_Low;
			SPI_DMA.DMA_BufferSize = SPI_XFER_ONEC_MAX_LEN*SPI_RX_DMA_FRAME_NUM;
			SPI_DMA.DMA_M2M = DMA_M2M_Disable;
			SPI_DMA.DMA_MemoryBaseAddr = stm32_spi2_rx_dma_buf;
			SPI_DMA.DMA_PeripheralBaseAddr = spi_bus->spix->DR;
			DMA_Init(SPI2_DMA_RX, &SPI_DMA);
		}
    }
    else {
        return RT_ENOSYS;
    }

    return RT_EOK;
}

rt_err_t rt_hw_spi_bus_init(void){
	rt_err_t ret = RT_EOK;
	
#ifdef RT_USING_SPI1
	stm32_spi_bus_1.spix = SPI1;
	ret = stm32_spi_bus_register(stm32_spi_bus_1, "spi1");
	if(RT_EOK != ret){
		return ret;
	}
#endif /* RT_USING_SPI1 */

#ifdef RT_USING_SPI2
	stm32_spi_bus_2.spix = SPI2;
	ret = stm32_spi_bus_register(stm32_spi_bus_2, "spi2");
	if(RT_EOK != ret){
		return ret;
	}
#endif /* RT_USING_SPI2 */

	return RT_EOK;
}

