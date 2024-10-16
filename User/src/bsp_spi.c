#include "bsp_spi.h"
#include "bsp.h"


#define HARD_SPI
//#define SOFE_SPI


#ifdef HARD_SPI
/*
 * ѡ��DMA���жϻ��߲�ѯ��ʽ
 */
//#define USE_SPI_DMA    /* DMA��ʽ  */
//#define USE_SPI_INT    /* �жϷ�ʽ */
#define USE_SPI_POLL   /* ��ѯ��ʽ */

/*
 * ʱ�ӣ����ţ�DMA���жϵȺ궨��
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

uint8_t  g_spi_busy; /* SPIæ״̬��0��ʾ��æ��1��ʾæ */
__IO uint32_t wTransferState = TRANSFER_WAIT;

uint32_t g_spiTxLen = 0, g_spiRxLen = 0;
uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];  
uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];


/**
 * @name   bsp_InitSPIBus
 * @brief  ����SPI����
 * @param  ��
 * @retval ��
 */
void bsp_InitSPIBus(void)
{	
	g_spi_busy = 0;
	
	bsp_InitSPIParam(LL_SPI_BAUDRATEPRESCALER_DIV4, LL_SPI_PHASE_1EDGE, LL_SPI_POLARITY_LOW);
}

/**
 * @name   bsp_InitSPIParam
 * @brief  ����SPI���߲�����ʱ�ӷ�Ƶ��ʱ����λ��ʱ�Ӽ��ԡ�
 * @param  _BaudRatePrescaler  SPI����ʱ�ӷ�Ƶ���ã�֧�ֵĲ������£�
 *                             LL_SPI_BAUDRATEPRESCALER_DIV2    2��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV4    4��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV8    8��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV16   16��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV32   32��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV64   64��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV128  128��Ƶ
 *                             LL_SPI_BAUDRATEPRESCALER_DIV256  256��Ƶ
 * @param  _CLKPhase           ʱ����λ��֧�ֵĲ������£�
 *							   LL_SPI_PHASE_1EDGE     SCK���ŵĵ�1�����ز�����ĵ�1������
 *							   LL_SPI_PHASE_2EDGE     SCK���ŵĵ�2�����ز�����ĵ�1������
 * @param  _CLKPolarity        ʱ�Ӽ��ԣ�֧�ֵĲ������£�
 *	                           LL_SPI_POLARITY_LOW    SCK�����ڿ���״̬���ڵ͵�ƽ
 *                             LL_SPI_POLARITY_HIGH   SCK�����ڿ���״̬���ڸߵ�ƽ
 * @retval ��
 */
void bsp_InitSPIParam(uint32_t _BaudRatePrescaler, uint32_t _CLKPhase, uint32_t _CLKPolarity)
{
	/* ���ִ��Ч�ʣ�ֻ����SPIӲ�����������仯ʱ����ִ��HAL_Init */
	if (s_BaudRatePrescaler == _BaudRatePrescaler && s_CLKPhase == _CLKPhase && s_CLKPolarity == _CLKPolarity)
	{		
		return;
	}

	s_BaudRatePrescaler = _BaudRatePrescaler;	
	s_CLKPhase = _CLKPhase;
	s_CLKPolarity = _CLKPolarity;
	
	LL_SPI_MspInit(&hspi);
	
	/* ����SPI���� */
	hspi.TransferDirection	= LL_SPI_FULL_DUPLEX;				/* ȫ˫�� */
    hspi.Mode				= LL_SPI_MODE_MASTER;				/* SPI����������ģʽ */
    hspi.DataWidth			= LL_SPI_DATAWIDTH_8BIT;			/* �������ݿ�� */
    hspi.ClockPolarity		= _CLKPolarity;						/* ����ʱ�Ӽ��� */
    hspi.ClockPhase			= _CLKPhase;						/* ����ʱ����λ */
    hspi.NSS				= LL_SPI_NSS_SOFT;					/* ʹ�������ʽ����Ƭѡ���� */
    hspi.BaudRate			= _BaudRatePrescaler;				/* ���ò����� */
    hspi.BitOrder			= LL_SPI_MSB_FIRST;					/* ���ݴ����ȴ���λ */
	hspi.CRCPoly			= 7;								/* ��ֹCRC�󣬴�λ��Ч */
	hspi.CRCCalculation		= LL_SPI_CRCCALCULATION_DISABLE;	/* ��ֹCRC */

	/* ��λSPI */
	if(LL_SPI_DeInit(SPIx) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);     
	}

	/* ��ʼ��SPI */
	if (LL_SPI_Init(SPIx, &hspi) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	LL_SPI_SetStandard(SPIx, LL_SPI_PROTOCOL_MOTOROLA);
	
	LL_SPI_EnableNSSPulseMgt(SPIx);
}

/**
 * @name   bsp_InitSPIParam
 * @brief  ����SPI����ʱ�ӣ�GPIO���жϣ�DMA��
 * @param  SPI_HandleTypeDef ����ָ�����
 * @retval ��
 */
void LL_SPI_MspInit(LL_SPI_InitTypeDef *_hspi)
{
	/* ���� SPI����GPIO : SCK MOSI MISO */
	{
		LL_GPIO_InitTypeDef  GPIO_InitStruct;
			
		/* SPI��GPIPʱ�� */
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

	/* ����DMA��NVIC */
	#ifdef USE_SPI_DMA
	{
		/* ʹ��DMAʱ�� */
		DMAx_CLK_ENABLE();      

		/* SPI DMA�������� */
		hdma_tx.Direction				= LL_DMA_DIRECTION_MEMORY_TO_PERIPH;	/* ���䷽���ǴӴ洢�������� */  
		hdma_tx.MemoryOrM2MDstDataSize	= LL_DMA_MDATAALIGN_BYTE;				/* �洢�����ݴ���λ��ѡ���ֽڣ���8bit */    
		hdma_tx.MemoryOrM2MDstIncMode	= LL_DMA_MEMORY_INCREMENT;				/* �洢����ַ����ʹ�� */  
		hdma_tx.Mode					= LL_DMA_MODE_NORMAL;					/* ����ģʽ */
		hdma_tx.PeriphOrM2MSrcDataSize	= LL_DMA_PDATAALIGN_BYTE;				/* �������ݴ���λ��ѡ���ֽڣ���8bit */ 
		hdma_tx.PeriphOrM2MSrcIncMode	= LL_DMA_PERIPH_NOINCREMENT;			/* �����ַ������ֹ */  
		hdma_tx.PeriphRequest			= LL_DMAMUX_REQ_SPI1_TX;
		hdma_tx.Priority				= LL_DMA_PRIORITY_LOW;					/* ���ȼ��� */

		/* ��λDMA */	
		if(LL_DMA_DeInit(DMAx,SPIx_TX_DMA_CHANNEL) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		 /* ��ʼ��DMA */
		if(LL_DMA_Init(DMAx,SPIx_TX_DMA_CHANNEL, &hdma_tx) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		LL_DMA_SetMemoryAddress(DMAx, SPIx_TX_DMA_CHANNEL,(uint32_t)g_spiTxBuf);					/* ���ô�������ַ */
		LL_DMA_SetPeriphAddress(DMAx, SPIx_TX_DMA_CHANNEL,(uint32_t)LL_SPI_DMA_GetRegAddr(SPIx));	/* ���������ַ */
		
		LL_DMA_ConfigAddresses(DMAx, SPIx_TX_DMA_CHANNEL, (uint32_t)(&g_spiTxBuf[0]), (uint32_t)LL_SPI_DMA_GetRegAddr(SPIx), LL_DMA_GetDataTransferDirection(DMAx, SPIx_TX_DMA_CHANNEL));
		
		/* ʹ��DMAͨ�� */
		LL_DMA_EnableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
		/* ʹ��DMA SPI���� */
		LL_SPI_EnableDMAReq_TX(SPIx);
		
		/* SPI DMA�������� */	
		hdma_rx.Direction				= LL_DMA_DIRECTION_PERIPH_TO_MEMORY;	/* ���䷽������赽�洢�� */  
		hdma_rx.MemoryOrM2MDstDataSize	= LL_DMA_MDATAALIGN_BYTE;				/* �洢�����ݴ���λ��ѡ���ֽڣ���8bit */    
		hdma_rx.MemoryOrM2MDstIncMode	= LL_DMA_MEMORY_INCREMENT;				/* �洢����ַ����ʹ�� */  
		hdma_rx.Mode					= LL_DMA_MODE_NORMAL;					/* ����ģʽ */
		hdma_rx.PeriphOrM2MSrcDataSize	= LL_DMA_PDATAALIGN_BYTE;				/* �������ݴ���λ��ѡ���ֽڣ���8bit */ 
		hdma_rx.PeriphOrM2MSrcIncMode	= LL_DMA_PERIPH_NOINCREMENT;			/* �����ַ������ֹ */  
		hdma_rx.PeriphRequest			= LL_DMAMUX_REQ_SPI1_RX;
		hdma_rx.Priority				= LL_DMA_PRIORITY_HIGH;					/* ���ȼ��� */

		/* ��λDMA */
		if(LL_DMA_DeInit(DMAx,SPIx_RX_DMA_CHANNEL) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		 /* ��ʼ��DMA */
		if(LL_DMA_Init(DMAx,SPIx_RX_DMA_CHANNEL, &hdma_rx) != SUCCESS)
		{
			Error_Handler(__FILE__, __LINE__);     
		}
		
		LL_DMA_SetMemoryAddress(DMAx, SPIx_RX_DMA_CHANNEL,(uint32_t)g_spiRxBuf);					/* ���ô�������ַ */
		LL_DMA_SetPeriphAddress(DMAx, SPIx_RX_DMA_CHANNEL, (uint32_t)LL_SPI_DMA_GetRegAddr(SPIx));	/* ���������ַ */
		
		LL_DMA_ConfigAddresses(DMAx, SPIx_RX_DMA_CHANNEL,(uint32_t)LL_SPI_DMA_GetRegAddr(SPIx), (uint32_t)(&g_spiRxBuf[0]), LL_DMA_GetDataTransferDirection(DMAx, SPIx_RX_DMA_CHANNEL));
		
		/* ʹ��DMAͨ�� */
		LL_DMA_EnableChannel(DMAx, SPIx_RX_DMA_CHANNEL);
		/* ʹ��DMA SPI���� */
		LL_SPI_EnableDMAReq_RX(SPIx);
		
		/* ����DMA�����ж� */
		NVIC_SetPriority(SPIx_DMA_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
		
		/* ����DMA�����ж� */
		NVIC_SetPriority(SPIx_DMA_RX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);
		
		/* ����SPI�ж� */
		NVIC_SetPriority(SPIx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_IRQn);
	}
	#endif
	
	#ifdef USE_SPI_INT
		/* ����SPI�ж����ȼ���ʹ���ж� */
		NVIC_SetPriority(SPIx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ(SPIx_IRQn);
	#endif
	
	LL_SPI_Enable(SPIx);
	LL_SPI_EnableIT_TXE(SPIx);
	LL_SPI_EnableIT_RXNE(SPIx);
}

/**
 * @name   sf_SendByte
 * @brief  �������ݴ���
 * @param  ��
 * @retval ��
 */
uint8_t sf_SendByte(uint8_t _ucValue)
{

	
	/* DMA��ʽ���� */
#ifdef USE_SPI_DMA
	/* wait dma transmit complete */
	wTransferState = TRANSFER_WAIT;

	while (wTransferState == TRANSFER_WAIT);
	
	return 0;
#endif

	/* �жϷ�ʽ���� */	
#ifdef USE_SPI_INT
	wTransferState = TRANSFER_WAIT;
	
	while (wTransferState == TRANSFER_WAIT);
	
	return 0;
#endif

	/* ��ѯ��ʽ���� */	
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
 * @brief  �������ݴ���
 * @param  ��
 * @retval ��
 */
void bsp_spiTransfer(void)
{
	if(g_spiRxLen > SPI_BUFFER_SIZE || g_spiTxLen > SPI_BUFFER_SIZE)
		return;
	/* DMA��ʽ���� */
#ifdef USE_SPI_DMA
	/* wait dma transmit complete */
	wTransferState = TRANSFER_WAIT;

	while (wTransferState == TRANSFER_WAIT);
	
	return;
#endif

	/* �жϷ�ʽ���� */	
#ifdef USE_SPI_INT
	wTransferState = TRANSFER_WAIT;
	
	while (wTransferState == TRANSFER_WAIT);
	
	return;
#endif

	/* ��ѯ��ʽ���� */	
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
 * @name   SPIx_IRQHandler��SPIx_DMA_RX_IRQHandler��SPIx_DMA_TX_IRQHandler
 * @brief  �жϷ������
 * @param  ��
 * @retval ��
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
  * ��������: DMA�жϴ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void DMA_STREAM_Tx_IRQHandler(void)
{
	/* �Ƿ������ */
	if(LL_DMA_IsActiveFlag_TC3(DMAx))
	{
		/* �����������жϱ�� */
		LL_DMA_ClearFlag_TC3(DMAx);
		
		/* ʹ��DMA������ */
		LL_SPI_EnableDMAReq_RX(SPIx);
	}
	/* �Ƿ����쳣 */
	if(LL_DMA_IsActiveFlag_TE3(DMAx))
	{
		/* ����쳣�жϱ�� */
		LL_DMA_ClearFlag_TE3(DMAx);
		/* ��ֹ�����쳣�ж� */
		LL_DMA_DisableIT_TE(DMAx, SPIx_TX_DMA_CHANNEL);
		/* ��ֹDMAͨ�� */
		LL_DMA_DisableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
	}
}

/**
  * ��������: DMA�жϴ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void DMA_STREAM_Rx_IRQHandler(void)
{
	/* �Ƿ������ */
	if(LL_DMA_IsActiveFlag_TC1(DMAx))
	{
		/* �����������жϱ�� */
		LL_DMA_ClearFlag_TC1(DMAx);
		
		/* ʹ��DMA������ */
		LL_SPI_EnableDMAReq_RX(SPIx);
	}
	/* �Ƿ����쳣 */
	if(LL_DMA_IsActiveFlag_TE1(DMAx))
	{
		/* ����쳣�жϱ�� */
		LL_DMA_ClearFlag_TE1(DMAx);
		/* ��ֹ�����쳣�ж� */
		LL_DMA_DisableIT_TE(DMAx, SPIx_TX_DMA_CHANNEL);
		/* ��ֹDMAͨ�� */
		LL_DMA_DisableChannel(DMAx, SPIx_TX_DMA_CHANNEL);
	}
}
	
#endif  /* HARD_SPI */


#ifdef SOFE_SPI

/* ����SPI ����ips��Ļ */
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

/*�ȴ���ʱʱ��*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/**
 * @name   bsp_SPI_GPIO_Enable
 * @brief  ʹ��GPIOʱ��
 * @param  GPIOx GPIOA - GPIOK
 * @retval ��
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
 * @brief  spi�����ʼ��
 * @param  ��
 * @retval ��
 */
void bsp_InitSOFESPI(void)
{
	LL_GPIO_InitTypeDef gpio_init;
	
	/*����LED��ص�GPIO����ʱ��*/
	bsp_SPI_GPIO_Enable(SPI_SCL_GPIO);
	bsp_SPI_GPIO_Enable(SPI_SDA_GPIO);
	bsp_SPI_GPIO_Enable(SPI_RST_GPIO);
	bsp_SPI_GPIO_Enable(SPI_DC_GPIO);
	bsp_SPI_GPIO_Enable(SPI_CS_GPIO);
	bsp_SPI_GPIO_Enable(SPI_BLK_GPIO);
	
	
	/* ����SPI�� SCL���ţ���ͨIO���� */
	
	gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	gpio_init.Pin = SPI_SCL_PIN;
	LL_GPIO_Init(SPI_SCL_GPIO, &gpio_init);
	
	/* ����SPI�� SDA���ţ���ͨIO���� */	
	gpio_init.Pin = SPI_SDA_PIN;
	LL_GPIO_Init(SPI_SDA_GPIO, &gpio_init);
	
	/* ����SPI�� RST���ţ���ͨIO���� */
	gpio_init.Pin = SPI_RST_PIN;
	LL_GPIO_Init(SPI_RST_GPIO, &gpio_init);
	
	/* ����SPI�� DC���ţ���ͨIO���� */
	gpio_init.Pin = SPI_DC_PIN;
	LL_GPIO_Init(SPI_DC_GPIO, &gpio_init);
	
	/* ����SPI�� CS���ţ���ͨIO���� */
	gpio_init.Pin = SPI_CS_PIN;
	LL_GPIO_Init(SPI_CS_GPIO, &gpio_init);
	
	/* ����SPI�� BLK���ţ���ͨIO���� */
	gpio_init.Pin = SPI_BLK_PIN;
	LL_GPIO_Init(SPI_BLK_GPIO, &gpio_init);
	
	/* ��ʼ��GPIO */
//	LL_GPIO_WritePin(SPI_SCL_GPIO, SPI_SCL_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_SDA_GPIO, SPI_SDA_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_RST_GPIO, SPI_RST_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_BLK_PIN, GPIO_PIN_SET);
}

#endif  /* SOFE_SPI */
