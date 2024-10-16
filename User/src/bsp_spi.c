#include "bsp_spi.h"
#include "bsp.h"


#define HARD_SPI
//#define SOFE_SPI


#ifdef HARD_SPI
/*
 * 选择DMA，中断或者查询方式
 */
//#define USE_SPI_DMA    /* DMA方式  */
//#define USE_SPI_INT    /* 中断方式 */
#define USE_SPI_POLL   /* 查询方式 */

/*
 * 时钟，引脚，DMA，中断等宏定义
 */
#define SPIx						SPI1

#define SPIx_CLK_ENABLE()			LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1)

#define DMAx_CLK_ENABLE() {	    \
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);	\
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1); \
	};

#define SPIx_FORCE_RESET()			LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SPI1)
#define SPIx_RELEASE_RESET()		LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SPI1)

#define SPIx_SCK_CLK_ENABLE()		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define SPIx_SCK_GPIO				GPIOB
#define SPIx_SCK_PIN				LL_GPIO_PIN_3
#define SPIx_SCK_AF					LL_GPIO_AF_5

#define SPIx_MISO_CLK_ENABLE()		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define SPIx_MISO_GPIO				GPIOB
#define SPIx_MISO_PIN 				LL_GPIO_PIN_4
#define SPIx_MISO_AF				LL_GPIO_AF_5

#define SPIx_MOSI_CLK_ENABLE()		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define SPIx_MOSI_GPIO				GPIOB
#define SPIx_MOSI_PIN 				LL_GPIO_PIN_5
#define SPIx_MOSI_AF				LL_GPIO_AF_5

#define DMAx						DMA1

#define SPIx_TX_DMA_CHANNEL         LL_DMA_CHANNEL_3
#define SPIx_RX_DMA_CHANNEL         LL_DMA_CHANNEL_1

#define SPIx_DMA_TX_IRQn            DMA1_Channel3_IRQn
#define SPIx_DMA_TX_IRQHandler      DMA1_Channel3_IRQHandler

#define SPIx_DMA_RX_IRQn            DMA1_Channel1_IRQn
#define SPIx_DMA_RX_IRQHandler      DMA1_Channel1_IRQHandler

#define SPIx_IRQn                   SPI1_IRQn
#define SPIx_IRQHandler             SPI1_IRQHandler

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

static LL_SPI_InitTypeDef  hspi = {0};
static LL_DMA_InitTypeDef hdma_tx;
static LL_DMA_InitTypeDef hdma_rx;

static uint32_t s_BaudRatePrescaler;
static uint32_t s_CLKPhase;
static uint32_t s_CLKPolarity;

uint8_t  g_spi_busy; /* SPI忙状态，0表示不忙，1表示忙 */
__IO uint32_t wTransferState = TRANSFER_WAIT;

uint32_t g_spiTxLen = 0, g_spiRxLen = 0;
uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];  
uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];


/**
 * @name   bsp_InitSPIBus
 * @brief  配置SPI总线
 * @param  无
 * @retval 无
 */
void bsp_InitSPIBus(void)
{	
	g_spi_busy = 0;
	
	bsp_InitSPIParam(LL_SPI_BAUDRATEPRESCALER_DIV4, LL_SPI_PHASE_1EDGE, LL_SPI_POLARITY_LOW);
}

/**
 * @name   bsp_InitSPIParam
 * @brief  配置SPI总线参数，时钟分频，时钟相位和时钟极性。
 * @param  _BaudRatePrescaler  SPI总线时钟分频设置，支持的参数如下：
 *                             LL_SPI_BAUDRATEPRESCALER_DIV2    2分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV4    4分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV8    8分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV16   16分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV32   32分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV64   64分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV128  128分频
 *                             LL_SPI_BAUDRATEPRESCALER_DIV256  256分频
 * @param  _CLKPhase           时钟相位，支持的参数如下：
 *							   LL_SPI_PHASE_1EDGE     SCK引脚的第1个边沿捕获传输的第1个数据
 *							   LL_SPI_PHASE_2EDGE     SCK引脚的第2个边沿捕获传输的第1个数据
 * @param  _CLKPolarity        时钟极性，支持的参数如下：
 *	                           LL_SPI_POLARITY_LOW    SCK引脚在空闲状态处于低电平
 *                             LL_SPI_POLARITY_HIGH   SCK引脚在空闲状态处于高电平
 * @retval 无
 */
void bsp_InitSPIParam(uint32_t _BaudRatePrescaler, uint32_t _CLKPhase, uint32_t _CLKPolarity)
{
	/* 提高执行效率，只有在SPI硬件参数发生变化时，才执行HAL_Init */
	if (s_BaudRatePrescaler == _BaudRatePrescaler && s_CLKPhase == _CLKPhase && s_CLKPolarity == _CLKPolarity)
	{		
		return;
	}

	s_BaudRatePrescaler = _BaudRatePrescaler;	
	s_CLKPhase = _CLKPhase;
	s_CLKPolarity = _CLKPolarity;
	
	LL_SPI_MspInit(&hspi);
	
	/* 设置SPI参数 */
	hspi.TransferDirection	= LL_SPI_FULL_DUPLEX;				/* 全双工 */
    hspi.Mode				= LL_SPI_MODE_MASTER;				/* SPI工作在主控模式 */
    hspi.DataWidth			= LL_SPI_DATAWIDTH_8BIT;			/* 设置数据宽度 */
    hspi.ClockPolarity		= _CLKPolarity;						/* 配置时钟极性 */
    hspi.ClockPhase			= _CLKPhase;						/* 配置时钟相位 */
    hspi.NSS				= LL_SPI_NSS_SOFT;					/* 使用软件方式管理片选引脚 */
    hspi.BaudRate			= _BaudRatePrescaler;				/* 设置波特率 */
    hspi.BitOrder			= LL_SPI_MSB_FIRST;					/* 数据传输先传高位 */
	hspi.CRCPoly			= 7;								/* 禁止CRC后，此位无效 */
	hspi.CRCCalculation		= LL_SPI_CRCCALCULATION_DISABLE;	/* 禁止CRC */

	/* 复位SPI */
	if(LL_SPI_DeInit(SPIx) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);     
	}

	/* 初始化SPI */
	if (LL_SPI_Init(SPIx, &hspi) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	LL_SPI_SetStandard(SPIx, LL_SPI_PROTOCOL_MOTOROLA);
	
	LL_SPI_EnableNSSPulseMgt(SPIx);
}

/**
 * @name   bsp_InitSPIParam
 * @brief  配置SPI总线时钟，GPIO，中断，DMA等
 * @param  SPI_HandleTypeDef 类型指针变量
 * @retval 无
 */
void LL_SPI_MspInit(LL_SPI_InitTypeDef *_hspi)
{
	/* 配置 SPI总线GPIO : SCK MOSI MISO */
	{
		LL_GPIO_InitTypeDef  GPIO_InitStruct;
			
		/* SPI和GPIP时钟 */
		SPIx_SCK_CLK_ENABLE();
		SPIx_MISO_CLK_ENABLE();
		SPIx_MOSI_CLK_ENABLE();
		SPIx_CLK_ENABLE();

		/* SPI SCK */
		GPIO_InitStruct.Pin			= SPIx_SCK_PIN;
		GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
		GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Alternate	= SPIx_SCK_AF;
		LL_GPIO_Init(SPIx_SCK_GPIO, &GPIO_InitStruct);	
	
		/* SPI MISO */
		GPIO_InitStruct.Pin = SPIx_MISO_PIN;
		GPIO_InitStruct.Alternate = SPIx_MISO_AF;
		LL_GPIO_Init(SPIx_MISO_GPIO, &GPIO_InitStruct);

		/* SPI MOSI */
		GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
		GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
		LL_GPIO_Init(SPIx_MOSI_GPIO, &GPIO_InitStruct);
	}

	/* 配置DMA和NVIC */
	#ifdef USE_SPI_DMA
	{
		/* 使能DMA时钟 */
		DMAx_CLK_ENABLE();      

		/* SPI DMA发送配置 */
		hdma_tx.Direction				= LL_DMA_DIRECTION_MEMORY_TO_PERIPH;	/* 传输方向是从存储器到外设 */  
		hdma_tx.MemoryOrM2MDstDataSize	= LL_DMA_MDATAALIGN_BYTE;				/* 存储器数据传输位宽选择字节，即8bit */    
		hdma_tx.MemoryOrM2MDstIncMode	= LL_DMA_MEMORY_INCREMENT;				/* 存储器地址自增使能 */  
		hdma_tx.Mode					= LL_DMA_MODE_NORMAL;					/* 正常模式 */
		hdma_tx.PeriphOrM2MSrcDataSize	= LL_DMA_PDATAALIGN_BYTE;				/* 外设数据传输位宽选择字节，即8bit */ 
		hdma_tx.PeriphOrM2MSrcIncMode	= LL_DMA_PERIPH_NOINCREMENT;			/* 外设地址自增禁止 */  
		hdma_tx.PeriphRequest			= LL_DMAMUX_REQ_SPI1_TX;
		hdma_tx.Priority				= LL_DMA_PRIORITY_LOW;					/* 优先级低 */

		/* 复位DMA */	
		if(LL_DMA_DeInit(DMAx,SPIx_TX_DMA_CHANNEL) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		 /* 初始化DMA */
		if(LL_DMA_Init(DMAx,SPIx_TX_DMA_CHANNEL, &hdma_tx) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		LL_DMA_SetMemoryAddress(DMAx, SPIx_TX_DMA_CHANNEL,(uint32_t)g_spiTxBuf);					/* 设置储存器地址 */
		LL_DMA_SetPeriphAddress(DMAx, SPIx_TX_DMA_CHANNEL,(uint32_t)LL_SPI_DMA_GetRegAddr(SPIx));	/* 设置外设地址 */
		
		LL_DMA_ConfigAddresses(DMAx, SPIx_TX_DMA_CHANNEL, (uint32_t)(&g_spiTxBuf[0]), (uint32_t)LL_SPI_DMA_GetRegAddr(SPIx), LL_DMA_GetDataTransferDirection(DMAx, SPIx_TX_DMA_CHANNEL));
		
		/* 使能DMA通道 */
		LL_DMA_EnableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
		/* 使能DMA SPI接受 */
		LL_SPI_EnableDMAReq_TX(SPIx);
		
		/* SPI DMA接收配置 */	
		hdma_rx.Direction				= LL_DMA_DIRECTION_PERIPH_TO_MEMORY;	/* 传输方向从外设到存储器 */  
		hdma_rx.MemoryOrM2MDstDataSize	= LL_DMA_MDATAALIGN_BYTE;				/* 存储器数据传输位宽选择字节，即8bit */    
		hdma_rx.MemoryOrM2MDstIncMode	= LL_DMA_MEMORY_INCREMENT;				/* 存储器地址自增使能 */  
		hdma_rx.Mode					= LL_DMA_MODE_NORMAL;					/* 正常模式 */
		hdma_rx.PeriphOrM2MSrcDataSize	= LL_DMA_PDATAALIGN_BYTE;				/* 外设数据传输位宽选择字节，即8bit */ 
		hdma_rx.PeriphOrM2MSrcIncMode	= LL_DMA_PERIPH_NOINCREMENT;			/* 外设地址自增禁止 */  
		hdma_rx.PeriphRequest			= LL_DMAMUX_REQ_SPI1_RX;
		hdma_rx.Priority				= LL_DMA_PRIORITY_HIGH;					/* 优先级高 */

		/* 复位DMA */
		if(LL_DMA_DeInit(DMAx,SPIx_RX_DMA_CHANNEL) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		 /* 初始化DMA */
		if(LL_DMA_Init(DMAx,SPIx_RX_DMA_CHANNEL, &hdma_rx) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		LL_DMA_SetMemoryAddress(DMAx, SPIx_RX_DMA_CHANNEL,(uint32_t)g_spiRxBuf);					/* 设置储存器地址 */
		LL_DMA_SetPeriphAddress(DMAx, SPIx_RX_DMA_CHANNEL, (uint32_t)LL_SPI_DMA_GetRegAddr(SPIx));	/* 设置外设地址 */
		
		LL_DMA_ConfigAddresses(DMAx, SPIx_RX_DMA_CHANNEL,(uint32_t)LL_SPI_DMA_GetRegAddr(SPIx), (uint32_t)(&g_spiRxBuf[0]), LL_DMA_GetDataTransferDirection(DMAx, SPIx_RX_DMA_CHANNEL));
		
		/* 使能DMA通道 */
		LL_DMA_EnableChannel(DMAx, SPIx_RX_DMA_CHANNEL);
		/* 使能DMA SPI接受 */
		LL_SPI_EnableDMAReq_RX(SPIx);
		
		/* 配置DMA发送中断 */
		NVIC_SetPriority(SPIx_DMA_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
		
		/* 配置DMA接收中断 */
		NVIC_SetPriority(SPIx_DMA_RX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);
		
		/* 配置SPI中断 */
		NVIC_SetPriority(SPIx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_IRQn);
	}
	#endif
	
	#ifdef USE_SPI_INT
		/* 配置SPI中断优先级并使能中断 */
		NVIC_SetPriority(SPIx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_IRQn);
	#endif
	
	LL_SPI_Enable(SPIx);
	LL_SPI_EnableIT_TXE(SPIx);
	LL_SPI_EnableIT_RXNE(SPIx);
}

/**
 * @name   sf_SendByte
 * @brief  启动数据传输
 * @param  无
 * @retval 无
 */
uint8_t sf_SendByte(uint8_t _ucValue)
{

	
	/* DMA方式传输 */
#ifdef USE_SPI_DMA
	/* wait dma transmit complete */
	wTransferState = TRANSFER_WAIT;

	while (wTransferState == TRANSFER_WAIT);
	
	return 0;
#endif

	/* 中断方式传输 */	
#ifdef USE_SPI_INT
	wTransferState = TRANSFER_WAIT;
	
	while (wTransferState == TRANSFER_WAIT);
	
	return 0;
#endif

	/* 查询方式传输 */	
#ifdef USE_SPI_POLL
	/* Check TXE flag */
	while(LL_SPI_IsActiveFlag_TXE(SPIx));
	LL_SPI_TransmitData8(SPIx, _ucValue);	

	/* Wait until RXNE flag is reset */
	while(LL_SPI_IsActiveFlag_RXNE(SPIx));
	return LL_SPI_ReceiveData8(SPIx);
#endif
}

/**
 * @name   bsp_spiTransfer
 * @brief  启动数据传输
 * @param  无
 * @retval 无
 */
void bsp_spiTransfer(void)
{
	if(g_spiRxLen > SPI_BUFFER_SIZE || g_spiTxLen > SPI_BUFFER_SIZE)
		return;
	/* DMA方式传输 */
#ifdef USE_SPI_DMA
	/* wait dma transmit complete */
	wTransferState = TRANSFER_WAIT;

	while (wTransferState == TRANSFER_WAIT);
	
	return;
#endif

	/* 中断方式传输 */	
#ifdef USE_SPI_INT
	wTransferState = TRANSFER_WAIT;
	
	while (wTransferState == TRANSFER_WAIT);
	
	return;
#endif

	/* 查询方式传输 */	
#ifdef USE_SPI_POLL
	/* Check TXE flag */
	while(LL_SPI_IsActiveFlag_TXE(SPIx));
	LL_SPI_TransmitData8(SPIx, g_spiTxBuf[g_spiTxLen++]);	

	/* Wait until RXNE flag is reset */
	while(LL_SPI_IsActiveFlag_RXNE(SPIx));
	g_spiRxBuf[g_spiRxLen++] = LL_SPI_ReceiveData8(SPIx);
#endif
}


/**
 * @name   SPIx_IRQHandler，SPIx_DMA_RX_IRQHandler，SPIx_DMA_TX_IRQHandler
 * @brief  中断服务程序
 * @param  无
 * @retval 无
 */
#ifdef USE_SPI_INT
	void SPIx_IRQHandler(void)
	{
		if(LL_SPI_IsActiveFlag_TXE(SPIx))
		{
			/* send data */
			while(LL_SPI_IsActiveFlag_TXE(SPIx));
			LL_SPI_TransmitData8(SPIx, g_spiTxBuf[g_spiTxLen++]);

			/* received data */
			if(LL_SPI_IsActiveFlag_RXNE(SPIx))
			{
				g_spiRxBuf[g_spiRxLen++] = LL_SPI_ReceiveData8(SPIx);
			}
		}
		wTransferState = TRANSFER_COMPLETE;
	}	
#endif

#ifdef USE_SPI_DMA
	void SPIx_DMA_RX_IRQHandler(void)
	{
		DMA_STREAM_Rx_IRQHandler();
	}

	void SPIx_DMA_TX_IRQHandler(void)
	{
		DMA_STREAM_Tx_IRQHandler();
	}

	void SPIx_IRQHandler(void)
	{
		if(LL_SPI_IsActiveFlag_TXE(SPIx))
		{
			/* send data */
			while(LL_SPI_IsActiveFlag_TXE(SPIx))
			{
			};
			LL_SPI_TransmitData8(SPIx, g_spiTxBuf[g_spiTxLen++]);

			/* received data */
			if(LL_SPI_IsActiveFlag_RXNE(SPIx))
			{
				g_spiRxBuf[g_spiRxLen++] = LL_SPI_ReceiveData8(SPIx);
			}
		}
		wTransferState = TRANSFER_COMPLETE;
	}
#endif

/**
  * 函数功能: DMA中断处理函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void DMA_STREAM_Tx_IRQHandler(void)
{
	/* 是否传输完成 */
	if(LL_DMA_IsActiveFlag_TC3(DMAx))
	{
		/* 清除传输完成中断标记 */
		LL_DMA_ClearFlag_TC3(DMAx);
		
		/* 使能DMA数据流 */
		LL_SPI_EnableDMAReq_RX(SPIx);
	}
	/* 是否传输异常 */
	if(LL_DMA_IsActiveFlag_TE3(DMAx))
	{
		/* 清除异常中断标记 */
		LL_DMA_ClearFlag_TE3(DMAx);
		/* 禁止传输异常中断 */
		LL_DMA_DisableIT_TE(DMAx, SPIx_TX_DMA_CHANNEL);
		/* 禁止DMA通道 */
		LL_DMA_DisableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
	}
}

/**
  * 函数功能: DMA中断处理函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void DMA_STREAM_Rx_IRQHandler(void)
{
	/* 是否传输完成 */
	if(LL_DMA_IsActiveFlag_TC1(DMAx))
	{
		/* 清除传输完成中断标记 */
		LL_DMA_ClearFlag_TC1(DMAx);
		
		/* 使能DMA数据流 */
		LL_SPI_EnableDMAReq_RX(SPIx);
	}
	/* 是否传输异常 */
	if(LL_DMA_IsActiveFlag_TE1(DMAx))
	{
		/* 清除异常中断标记 */
		LL_DMA_ClearFlag_TE1(DMAx);
		/* 禁止传输异常中断 */
		LL_DMA_DisableIT_TE(DMAx, SPIx_TX_DMA_CHANNEL);
		/* 禁止DMA通道 */
		LL_DMA_DisableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
	}
}
	
#endif  /* HARD_SPI */


#ifdef SOFE_SPI

/* 三线SPI 驱动ips屏幕 */
#define SPI_SCL_PIN LL_GPIO_PIN_15
#define SPI_SDA_PIN LL_GPIO_PIN_13
#define SPI_RST_PIN LL_GPIO_PIN_12
#define SPI_DC_PIN  LL_GPIO_PIN_14
#define SPI_CS_PIN  LL_GPIO_PIN_5
#define SPI_BLK_PIN LL_GPIO_PIN_1

#define SPI_SCL_GPIO GPIOB
#define SPI_SDA_GPIO GPIOB
#define SPI_RST_GPIO GPIOB
#define SPI_DC_GPIO  GPIOB
#define SPI_CS_GPIO  GPIOC
#define SPI_BLK_GPIO GPIOB

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/**
 * @name   bsp_SPI_GPIO_Enable
 * @brief  使能GPIO时钟
 * @param  GPIOx GPIOA - GPIOK
 * @retval 无
 */
void bsp_SPI_GPIO_Enable(GPIO_TypeDef* GPIOx)
{
	if (GPIOx == GPIOA)	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	else if (GPIOx == GPIOB) LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	else if (GPIOx == GPIOC) LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	else
	{
		Error_Handler(__FILE__, __LINE__);
	}	
}

/**
 * @brief  spi软件初始化
 * @param  无
 * @retval 无
 */
void bsp_InitSOFESPI(void)
{
	LL_GPIO_InitTypeDef gpio_init;
	
	/*开启LED相关的GPIO外设时钟*/
	bsp_SPI_GPIO_Enable(SPI_SCL_GPIO);
	bsp_SPI_GPIO_Enable(SPI_SDA_GPIO);
	bsp_SPI_GPIO_Enable(SPI_RST_GPIO);
	bsp_SPI_GPIO_Enable(SPI_DC_GPIO);
	bsp_SPI_GPIO_Enable(SPI_CS_GPIO);
	bsp_SPI_GPIO_Enable(SPI_BLK_GPIO);
	
	
	/* 配置SPI的 SCL引脚，普通IO即可 */
	
	gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	gpio_init.Pin = SPI_SCL_PIN;
	LL_GPIO_Init(SPI_SCL_GPIO, &gpio_init);
	
	/* 配置SPI的 SDA引脚，普通IO即可 */	
	gpio_init.Pin = SPI_SDA_PIN;
	LL_GPIO_Init(SPI_SDA_GPIO, &gpio_init);
	
	/* 配置SPI的 RST引脚，普通IO即可 */
	gpio_init.Pin = SPI_RST_PIN;
	LL_GPIO_Init(SPI_RST_GPIO, &gpio_init);
	
	/* 配置SPI的 DC引脚，普通IO即可 */
	gpio_init.Pin = SPI_DC_PIN;
	LL_GPIO_Init(SPI_DC_GPIO, &gpio_init);
	
	/* 配置SPI的 CS引脚，普通IO即可 */
	gpio_init.Pin = SPI_CS_PIN;
	LL_GPIO_Init(SPI_CS_GPIO, &gpio_init);
	
	/* 配置SPI的 BLK引脚，普通IO即可 */
	gpio_init.Pin = SPI_BLK_PIN;
	LL_GPIO_Init(SPI_BLK_GPIO, &gpio_init);
	
	/* 初始化GPIO */
//	LL_GPIO_WritePin(SPI_SCL_GPIO, SPI_SCL_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_SDA_GPIO, SPI_SDA_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_RST_GPIO, SPI_RST_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_BLK_PIN, GPIO_PIN_SET);
}

#endif  /* SOFE_SPI */
