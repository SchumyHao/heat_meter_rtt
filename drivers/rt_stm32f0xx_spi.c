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

/* SPI1 */
#define SPI1_GPIO_MISO
#define SPI1_GPIO_MISO_SOURCE
#define SPI1_GPIO_MOSI
#define SPI1_GPIO_MOSI_SOURCE
#define SPI1_GPIO_SCLK
#define SPI1_GPIO_SCLK_SOURCE
#define SPI1_GPIO_NSS_GROUP
#define SPI1_GPIO_NSS_PIN
#ifdef RT_USING_SPI1_TX_DMA
#define SPI1_DMA_TX 
#else
#define SPI1_DMA_TX RT_NULL
#endif /* RT_USING_SPI1_TX_DMA */
#ifdef RT_USING_SPI1_RX_DMA
#define SPI1_DMA_RX
#else
#define SPI1_DMA_RX RT_NULL
#endif /* RT_USING_SPI1_RX_DMA */

/* SPI2 */
#define SPI2_GPIO_MISO
#define SPI2_GPIO_MISO_SOURCE
#define SPI2_GPIO_MOSI
#define SPI2_GPIO_MOSI_SOURCE
#define SPI2_GPIO_SCLK
#define SPI2_GPIO_SCLK_SOURCE
#define SPI2_GPIO_NSS_GROUP
#define SPI2_GPIO_NSS_PIN
#ifdef RT_USING_SPI2_TX_DMA
#define SPI2_DMA_TX 
#else
#define SPI2_DMA_TX RT_NULL
#endif /* RT_USING_SPI2_TX_DMA */
#ifdef RT_USING_SPI2_RX_DMA
#define SPI2_DMA_RX
#else
#define SPI2_DMA_RX RT_NULL
#endif /* RT_USING_SPI2_RX_DMA */

struct stm32_spi_cs {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
};

struct stm32_spi_bus {
    SPI_TypeDef* spix;
	SPI_InitTypeDef init;
	struct stm32_spi_cs* cs;
	uint8_t *tx_buf_ptr;
	uint32_t tx_xfer_size;
	uint32_t tx_xfer_count;
	uint8_t *rx_buf_ptr;
	uint32_t rx_xfer_size;
	uint32_t rx_xfer_count;
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

rt_inline void get_spi_InitTypeDef_from_configuration(SPI_InitTypeDef* SPI_Init, struct rt_spi_configuration* cfg){
    SPI_Init->SPI_DataSize = get_spi_DataSize(cfg->data_width);
    SPI_Init->SPI_BaudRatePrescaler = get_spi_BaudRatePrescaler(cfg->max_hz);
    SPI_Init->SPI_CPOL = get_spi_CPOL(cfg->mode);
    SPI_Init->SPI_CPHA = get_spi_CPHA(cfg->mode);
    SPI_Init->SPI_FirstBit = get_spi_FirstBit(cfg->mode);
    SPI_Init->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_Init->SPI_Mode = get_spi_Mode(cfg->mode);
    SPI_Init->SPI_NSS  = SPI_NSS_Soft;
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
    SPI_Cmd(SPIx, ENABLE);

    return RT_EOK;
}

rt_inline void take_cs(struct stm32_spi_bus* bus){
	RT_ASSERT(bus->cs != RT_NULL);
	GPIO_ResetBits(bus->cs->GPIOx, bus->cs->GPIO_Pin);
}

static rt_uint32_t xfer(struct rt_spi_device* dev, struct rt_spi_message* msg)
{
    struct stm32_spi_bus* spi_bus = NULL;
	//struct rt_spi_configuration* cfg = NULL;
	SPI_TypeDef* SPIx = NULL;
	//rt_uint32_t size = 0;
	
    RT_ASSERT(dev != RT_NULL);
	RT_ASSERT(dev->bus != RT_NULL);
	RT_ASSERT(dev->bus->parent.user_data != RT_NULL);
    RT_ASSERT(msg != RT_NULL);

	spi_bus = (struct stm32_spi_bus*)dev->bus->parent.user_data;
    //cfg = &dev->config;
    SPIx = spi_bus->spix;
    //size = msg->length;

    if(msg->cs_take) {
		take_cs(spi_bus);
    }

	

#ifdef SPI_USE_DMA
    if(message->length > 32) {
        if(config->data_width <= 8) {
            DMA_Configuration(stm32f0xx_spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
            //Wait until the DMA xfer over
            while (DMA_GetFlagStatus(stm32f0xx_spi_bus->DMA_Channel_RX_FLAG_TC) == RESET
                   || DMA_GetFlagStatus(stm32f0xx_spi_bus->DMA_Channel_TX_FLAG_TC) == RESET);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
        }
//        rt_memcpy(buffer,_spi_flash_buffer,DMA_BUFFER_SIZE);
//        buffer += DMA_BUFFER_SIZE;
    }
    else
#endif
    {
        if(config->data_width <= 8) {
            const rt_uint8_t* send_ptr = message->send_buf;
            rt_uint8_t* recv_ptr = message->recv_buf;

            while(size--) {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL) {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_SendData8(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_ReceiveData8(SPI);

                if(recv_ptr != RT_NULL) {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16) {
            const rt_uint16_t* send_ptr = message->send_buf;
            rt_uint16_t* recv_ptr = message->recv_buf;

            while(size--) {
                rt_uint16_t data = 0xFFFF;

                if(send_ptr != RT_NULL) {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData16(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData16(SPI);

                if(recv_ptr != RT_NULL) {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if(message->cs_release) {
        GPIO_SetBits(stm32f0xx_spi_cs->GPIOx, stm32f0xx_spi_cs->GPIO_Pin);
    }

    return message->length;
};

static struct rt_spi_ops stm32f0xx_spi_ops = {
    configure,
    xfer
};

#ifdef USING_SPI1
static struct stm32f0xx_spi_bus stm32f0xx_spi_bus_1;
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct stm32f0xx_spi_bus stm32f0xx_spi_bus_2;
#endif /* #ifdef USING_SPI2 */

//------------------ DMA ------------------
#ifdef SPI_USE_DMA
static uint8_t dummy = 0xFF;
#endif

#ifdef SPI_USE_DMA
static void DMA_Configuration(struct stm32f0xx_spi_bus* stm32f0xx_spi_bus, const void* send_addr, void* recv_addr, rt_size_t size)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(stm32f0xx_spi_bus->DMA_Channel_RX_FLAG_TC
                  | stm32f0xx_spi_bus->DMA_Channel_RX_FLAG_TE
                  | stm32f0xx_spi_bus->DMA_Channel_TX_FLAG_TC
                  | stm32f0xx_spi_bus->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Cmd(stm32f0xx_spi_bus->DMA_Channel_RX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32f0xx_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(recv_addr != RT_NULL) {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) recv_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) (&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32f0xx_spi_bus->DMA_Channel_RX, &DMA_InitStructure);

    DMA_Cmd(stm32f0xx_spi_bus->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Cmd(stm32f0xx_spi_bus->DMA_Channel_TX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32f0xx_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(send_addr != RT_NULL) {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&dummy);;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32f0xx_spi_bus->DMA_Channel_TX, &DMA_InitStructure);

    DMA_Cmd(stm32f0xx_spi_bus->DMA_Channel_TX, ENABLE);
}
#endif







/** \brief init and register stm32f0xx spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2.
 * \param stm32f0xx_spi: stm32f0xx spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t stm32f0xx_spi_register(SPI_TypeDef* SPI,
                                struct stm32f0xx_spi_bus* stm32f0xx_spi,
                                const char* spi_bus_name)
{
    if(SPI == SPI1) {
        stm32f0xx_spi->SPI = SPI1;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        stm32f0xx_spi->DMA_Channel_RX = DMA1_Channel2;
        stm32f0xx_spi->DMA_Channel_TX = DMA1_Channel3;
        stm32f0xx_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC2;
        stm32f0xx_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_TE2;
        stm32f0xx_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC3;
        stm32f0xx_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_TE3;
#endif
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    }
    else if(SPI == SPI2) {
        stm32f0xx_spi->SPI = SPI2;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        stm32f0xx_spi->DMA_Channel_RX = DMA1_Channel4;
        stm32f0xx_spi->DMA_Channel_TX = DMA1_Channel5;
        stm32f0xx_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC4;
        stm32f0xx_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_TE4;
        stm32f0xx_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC5;
        stm32f0xx_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_TE5;
#endif
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    }
    else {
        return RT_ENOSYS;
    }

    return rt_spi_bus_register(&stm32f0xx_spi->parent, spi_bus_name, &stm32f0xx_spi_ops);
}

