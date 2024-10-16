#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "main.h"

/* 重定义下SPI SCK时钟，方便移植 */
#define SPI_BAUDRATEPRESCALER_42_5M			LL_SPI_BAUDRATEPRESCALER_DIV4
#define SPI_BAUDRATEPRESCALER_21_25M		LL_SPI_BAUDRATEPRESCALER_DIV8
#define SPI_BAUDRATEPRESCALER_10_625M		LL_SPI_BAUDRATEPRESCALER_DIV16
#define SPI_BAUDRATEPRESCALER_5_3125M		LL_SPI_BAUDRATEPRESCALER_DIV32
#define SPI_BAUDRATEPRESCALER_2_65625M		LL_SPI_BAUDRATEPRESCALER_DIV64
#define SPI_BAUDRATEPRESCALER_1_328125M		LL_SPI_BAUDRATEPRESCALER_DIV128
#define SPI_BAUDRATEPRESCALER_664_062K		LL_SPI_BAUDRATEPRESCALER_DIV256

#define	SPI_BUFFER_SIZE		(4 * 1024)				/*  */

  
extern uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];
extern uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];
extern uint32_t g_spiLen;

extern uint8_t g_spi_busy;

void bsp_InitSPIBus(void);
void bsp_InitSPIParam(uint32_t _BaudRatePrescaler, uint32_t _CLKPhase, uint32_t _CLKPolarity);
void LL_SPI_MspInit(LL_SPI_InitTypeDef *_hspi);

uint8_t sf_SendByte(uint8_t _ucValue);
void bsp_spiTransfer(void);

void bsp_SPI_GPIO_Enable(GPIO_TypeDef* GPIOx);
void bsp_InitSOFESPI(void);

void DMA_STREAM_Tx_IRQHandler(void);
void DMA_STREAM_Rx_IRQHandler(void);

#endif  /* __BSP_SPI_H */
