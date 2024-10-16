#include "bsp_uart.h"
#include "bsp.h"

/*-----------------------------------------------------------------------
		system_stm32g4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		APB1 ������ USART2, USART3, UART4
		APB2 ������ USART1

	----------------------------------------------------------------------- */
	
/* ����1��GPIO  PA9, PA10 */
#define USART1_CLK_ENABLE()              LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

#define USART1_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_PIN                    LL_GPIO_PIN_9
#define USART1_TX_AF                     LL_GPIO_AF_7

#define USART1_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_PIN                    LL_GPIO_PIN_10
#define USART1_RX_AF                     LL_GPIO_AF_7

/* ����2��GPIO  PA2 PA3 */
#define USART2_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

#define USART2_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_PIN                    LL_GPIO_PIN_2
#define USART2_TX_AF                     LL_GPIO_AF_7

#define USART2_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_PIN                    LL_GPIO_PIN_3
#define USART2_RX_AF                     LL_GPIO_AF_7

/* ����3��GPIO  PD8 PD9 */
#define USART3_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

#define USART3_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
#define USART3_TX_GPIO_PORT              GPIOD
#define USART3_TX_PIN                    LL_GPIO_PIN_8
#define USART3_TX_AF                     LL_GPIO_AF_7

#define USART3_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
#define USART3_RX_GPIO_PORT              GPIOD
#define USART3_RX_PIN                    LL_GPIO_PIN_9
#define USART3_RX_AF                     LL_GPIO_AF_7

/* ����4��GPIO  PC10 PC11 */
#define UART4_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);

#define UART4_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define UART4_TX_GPIO_PORT              GPIOC
#define UART4_TX_PIN                    LL_GPIO_PIN_10
#define UART4_TX_AF                     LL_GPIO_AF_8

#define UART4_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define UART4_RX_GPIO_PORT              GPIOC
#define UART4_RX_PIN                    LL_GPIO_PIN_11
#define UART4_RX_AF                     LL_GPIO_AF_8


/* ����ÿ�����ڽṹ����� */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART4_FIFO_EN == 1
	static UART_T g_tUart4;
	static uint8_t g_TxBuf4[UART4_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf4[UART4_RX_BUF_SIZE];		/* ���ջ����� */
#endif


static void UartVarInit(void);
static void InitHardUart(void);
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);


/**
 * @name   bsp_InitUart
 * @brief  ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
 * @param  ��
 * @retval ��
 */
void bsp_InitUart(void)
{
	
	UartVarInit();		/* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	InitHardUart();		/* ���ô��ڵ�Ӳ������(�����ʵ�) */
}

/**
 * @name   ComToUart
 * @brief  ��COM�˿ں�ת��ΪUARTָ��
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @retval uartָ��
 */
UART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return &g_tUart1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return &g_tUart2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return &g_tUart3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM4)
	{
		#if UART4_FIFO_EN == 1
			return &g_tUart4;
		#else
			return 0;
		#endif
	}
	else
	{
		Error_Handler(__FILE__, __LINE__);
		return 0;
	}
}

/**
 * @name   ComToUSARTx
 * @brief  ��COM�˿ں�ת��Ϊ USART_TypeDef* USARTx
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @retval USART_TypeDef*  USART1,USART2,USART3,UART4,UART5,USART6,UART7,UART8.
 */
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return USART1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return USART2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return USART3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM4)
	{
		#if UART4_FIFO_EN == 1
			return UART4;
		#else
			return 0;
		#endif
	}
	else
	{
		/* �����κδ��� */
		return 0;
	}
}

/**
 * @name   comSendBuf
 * @brief  �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @param  _ucaBuf: �����͵����ݻ�����
 * @param  _usLen : ���ݳ���
 * @retval ��
 */
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	if (pUart->SendBefor != 0)
	{
		pUart->SendBefor();		/* �����RS485ͨ�ţ���������������н�RS485����Ϊ����ģʽ */
	}

	UartSend(pUart, _ucaBuf, _usLen);
}

/**
 * @name   comSendChar
 * @brief  �򴮿ڷ���1���ֽڡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @param  _ucByte: �����͵�����
 * @retval ��
 */
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

/**
 * @name   comGetChar
 * @brief  �ӽ��ջ�������ȡ1�ֽڣ��������������������ݾ���������.
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @param  _pByte: ���յ������ݴ���������ַ
 * @retval 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
 */
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/**
 * @name   comClearTxFifo
 * @brief  ���㴮�ڷ��ͻ�����
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @retval ��
 */
void comClearTxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}

/**
 * @name   comClearRxFifo
 * @brief  ���㴮�ڽ��ջ�����
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @retval ��
 */
void comClearRxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
}

/**
 * @name   comSetBaud
 * @brief  ���ô��ڵĲ�����. �������̶�����Ϊ��У��,�շ���ʹ��ģʽ
 * @param  _ucPort: �˿ں�(COM1 - COM8)
 * @param  _BaudRate: ������  8��������  ������.0-12.5Mbps
							 16�������� ������.0-6.25Mbps
 * @retval ��
 */
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate)
{
	USART_TypeDef* USARTx;
	
	USARTx = ComToUSARTx(_ucPort);
	if (USARTx == 0)
	{
		return;
	}
	
	bsp_SetUartParam(USARTx,  _BaudRate, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);
}

/**
 * @name   UartVarInit
 * @brief  ��ʼ��������صı���
 * @param  ��
 * @retval ��
 */
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 �����豸 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* ���ͻ�����ָ�� */
	g_tUart1.pRxBuf = g_RxBuf1;					/* ���ջ�����ָ�� */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart1.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usTxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usRxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart1.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart1.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart1.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart1.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart1.Sending = 0;						/* ���ڷ����б�־ */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;						/* STM32 �����豸 */
	g_tUart2.pTxBuf = g_TxBuf2;					/* ���ͻ�����ָ�� */
	g_tUart2.pRxBuf = g_RxBuf2;					/* ���ջ�����ָ�� */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart2.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart2.usTxRead = 0;						/* ����FIFO������ */
	g_tUart2.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart2.usRxRead = 0;						/* ����FIFO������ */
	g_tUart2.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart2.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart2.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart2.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart2.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart2.Sending = 0;						/* ���ڷ����б�־ */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;						/* STM32 �����豸 */
	g_tUart3.pTxBuf = g_TxBuf3;					/* ���ͻ�����ָ�� */
	g_tUart3.pRxBuf = g_RxBuf3;					/* ���ջ�����ָ�� */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart3.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart3.usTxRead = 0;						/* ����FIFO������ */
	g_tUart3.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart3.usRxRead = 0;						/* ����FIFO������ */
	g_tUart3.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart3.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart3.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart3.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart3.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart3.Sending = 0;						/* ���ڷ����б�־ */
#endif

#if UART4_FIFO_EN == 1
	g_tUart4.uart = UART4;						/* STM32 �����豸 */
	g_tUart4.pTxBuf = g_TxBuf4;					/* ���ͻ�����ָ�� */
	g_tUart4.pRxBuf = g_RxBuf4;					/* ���ջ�����ָ�� */
	g_tUart4.usTxBufSize = UART4_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart4.usRxBufSize = UART4_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart4.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart4.usTxRead = 0;						/* ����FIFO������ */
	g_tUart4.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart4.usRxRead = 0;						/* ����FIFO������ */
	g_tUart4.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart4.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart4.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart4.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart4.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart4.Sending = 0;						/* ���ڷ����б�־ */
#endif

}

/**
 * @name   bsp_SetUartParam
 * @brief  ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32- H7������
 * @param  Instance   USART_TypeDef���ͽṹ��
 * @param  BaudRate   ������
 * @param  Parity     У�����ͣ���У�����żУ��
 * @param  Mode       ���ͺͽ���ģʽʹ��
 * @retval ��
 */
void bsp_SetUartParam(USART_TypeDef *Instance,  uint32_t BaudRate, uint32_t Parity, uint32_t Mode)
{
	LL_USART_InitTypeDef UartHandle;	
	
	/*##-1- ���ô���Ӳ������ ######################################*/
	/* �첽����ģʽ (UART Mode) */
	/* ��������:
	  - �ֳ�    = 8 λ
	  - ֹͣλ  = 1 ��ֹͣλ
	  - У��    = ����Parity
	  - ������  = ����BaudRate
	  - Ӳ�������ƹر� (RTS and CTS signals) */
	UartHandle.BaudRate				 = BaudRate;
	UartHandle.DataWidth			 = LL_USART_DATAWIDTH_8B;
	UartHandle.StopBits				 = LL_USART_STOPBITS_1;
	UartHandle.Parity				 = Parity;
	UartHandle.TransferDirection	 = Mode;
	UartHandle.HardwareFlowControl	 = LL_USART_HWCONTROL_NONE;
	UartHandle.OverSampling 		 = LL_USART_OVERSAMPLING_16;
	
	if (LL_USART_Init(Instance, &UartHandle) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	LL_USART_Enable(Instance);
}

/**
 * @name   InitHardUart
 * @brief  ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32- H7������
 * @param  ��
 * @retval ��
 */
static void InitHardUart(void)
{
	LL_GPIO_InitTypeDef  GPIO_InitStruct;


#if UART1_FIFO_EN == 1		/* ����1 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	USART1_TX_GPIO_CLK_ENABLE();
	USART1_RX_GPIO_CLK_ENABLE();
	
	/* ʹ�� USARTx ʱ�� */
	USART1_CLK_ENABLE();	

	/* ����TX���� */
	GPIO_InitStruct.Pin      	 = USART1_TX_PIN;
	GPIO_InitStruct.Mode     	 = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Pull     	 = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Speed    	 = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate	 = USART1_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	
	LL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����RX���� */
	GPIO_InitStruct.Pin = USART1_RX_PIN;
	GPIO_InitStruct.Alternate = USART1_RX_AF;
	LL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART1,  UART1_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ICR, USART_ICR_TCCF);   /* ���TC������ɱ�־ */
    CLEAR_BIT(USART1->RQR, USART_RQR_RXFRQ); /* ���RXNE���ձ�־ */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE);	/* ʹ��PE. RX�����ж� */
#endif

#if UART2_FIFO_EN == 1		/* ����2 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	USART2_TX_GPIO_CLK_ENABLE();
	USART2_RX_GPIO_CLK_ENABLE();
	
	/* ʹ�� USARTx ʱ�� */
	USART2_CLK_ENABLE();	

	/* ����TX���� */
	GPIO_InitStruct.Pin			 = USART2_TX_PIN;
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate	 = USART2_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����RX���� */
	GPIO_InitStruct.Pin = USART2_RX_PIN;
	GPIO_InitStruct.Alternate = USART2_RX_AF;
	LL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART2,  UART2_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);
	
	CLEAR_BIT(USART1->ICR, USART_ICR_TCCF);   /* ���TC������ɱ�־ */
    CLEAR_BIT(USART1->RQR, USART_RQR_RXFRQ); /* ���RXNE���ձ�־ */
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);	/* ʹ��PE. RX�����ж� */
#endif

#if UART3_FIFO_EN == 1			/* ����3 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	USART3_TX_GPIO_CLK_ENABLE();
	USART3_RX_GPIO_CLK_ENABLE();
	
	/* ʹ�� USARTx ʱ�� */
	USART3_CLK_ENABLE();	

	/* ����TX���� */
	GPIO_InitStruct.Pin			 = USART3_TX_PIN;
	GPIO_InitStruct.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		 = GPIO_PULLUP;
	GPIO_InitStruct.Speed		 = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	 = USART3_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����RX���� */
	GPIO_InitStruct.Pin = USART3_RX_PIN;
	GPIO_InitStruct.Alternate = USART3_RX_AF;
	HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART3,  UART3_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ISR, USART_ISR_TC);   /* ���TC������ɱ�־ */
    CLEAR_BIT(USART1->ISR, USART_ISR_RXNE); /* ���RXNE���ձ�־ */
	SET_BIT(USART3->CR1, USART_CR1_RXNEIE);	/* ʹ��PE. RX�����ж� */
#endif

#if UART4_FIFO_EN == 1
	/* ʹ�� GPIO TX/RX ʱ�� */
	UART4_TX_GPIO_CLK_ENABLE();
	UART4_RX_GPIO_CLK_ENABLE();
	
	/* ʹ�� USARTx ʱ�� */
	UART4_CLK_ENABLE();	

	/* ����TX���� */
	GPIO_InitStruct.Pin			 = UART4_TX_PIN;
	GPIO_InitStruct.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		 = GPIO_PULLUP;
	GPIO_InitStruct.Speed		 = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	 = UART4_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	HAL_GPIO_Init(UART4_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����RX���� */
	GPIO_InitStruct.Pin = UART4_RX_PIN;
	GPIO_InitStruct.Alternate = UART4_RX_AF;
	HAL_GPIO_Init(UART4_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(UART4,  UART4_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ISR, USART_ISR_TC);   /* ���TC������ɱ�־ */
    CLEAR_BIT(USART1->ISR, USART_ISR_RXNE); /* ���RXNE���ձ�־ */
	SET_BIT(UART4->CR1, USART_CR1_RXNEIE);	/* ʹ��RX�����ж� */
#endif

}

/**
 * @name   UartSend
 * @brief  ��д���ݵ�UART���ͻ�����,�����������жϡ��жϴ�����������Ϻ��Զ��رշ����ж�
 * @param  ��
 * @retval ��
 */
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* ������ͻ������Ѿ����ˣ���ȴ��������� */
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _pUart->usTxCount;
			ENABLE_INT();

			if (usCount < _pUart->usTxBufSize)
			{
				break;
			}
			else if(usCount == _pUart->usTxBufSize)/* ���������������� */
			{
				if((_pUart->uart->CR1 & USART_CR1_TXEIE) == 0)
				{
					SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);
				}  
			}
		}

		/* �����������뷢�ͻ����� */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);	/* ʹ�ܷ����жϣ��������գ� */
}

/**
 * @name   UartGetChar
 * @brief  �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
 * @param  _pUart : �����豸
 * @param  _pByte : ��Ŷ�ȡ���ݵ�ָ��
 * @retval ��
 */
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� */
	DISABLE_INT();
	usCount = _pUart->usRxCount;
	ENABLE_INT();

	/* �������д������ͬ���򷵻�0 */
	//if (_pUart->usRxRead == usRxWrite)
	if (usCount == 0)	/* �Ѿ�û������ */
	{
		return 0;
	}
	else
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead];		/* �Ӵ��ڽ���FIFOȡ1������ */

		/* ��дFIFO������ */
		DISABLE_INT();
		if (++_pUart->usRxRead >= _pUart->usRxBufSize)
		{
			_pUart->usRxRead = 0;
		}
		_pUart->usRxCount--;
		ENABLE_INT();
		return 1;
	}
}

/**
 * @name   UartTxEmpty
 * @brief  �жϷ��ͻ������Ƿ�Ϊ��.
 * @param  _pUart : �����豸
 * @retval 1Ϊ��,0Ϊ����.
 */
uint8_t UartTxEmpty(COM_PORT_E _ucPort)
{
   UART_T *pUart;
   uint8_t Sending;
   
   pUart = ComToUart(_ucPort);
   if (pUart == 0)
   {
      return 0;
   }

   Sending = pUart->Sending;

   if (Sending != 0)
   {
      return 0;
   }
   return 1;
}

/**
 * @name   UartIRQ
 * @brief  ���жϷ��������ã�ͨ�ô����жϴ�����.
 * @param  _pUart : �����豸
 * @retval ��
 */
static void UartIRQ(UART_T *_pUart)
{
	uint32_t isrflags   = READ_REG(_pUart->uart->ISR);
	uint32_t cr1its     = READ_REG(_pUart->uart->CR1);
	uint32_t cr3its     = READ_REG(_pUart->uart->CR3);
	
	/* ��������ж�  */
	if ((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
	{
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
		uint8_t ch;

		ch = READ_REG(_pUart->uart->RDR);
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
		{
			_pUart->usRxWrite = 0;
		}
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
		}

		/* �ص�����,֪ͨӦ�ó����յ�������,һ���Ƿ���1����Ϣ��������һ����� */
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch); /* ���磬����MODBUS����������ֽ��� */
			}
		}
	}

	/* �����ͻ��������ж� */
	if ( ((isrflags & USART_ISR_TXE_TXFNF) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�*/
			//USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

			/* ʹ�����ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			_pUart->Sending = 1;
			
			/* �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}

	}
	/* ����bitλȫ��������ϵ��ж� */
	if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

			/* �ص�����, һ����������RS485ͨ�ţ���RS485оƬ����Ϊ����ģʽ��������ռ���� */
			if (_pUart->SendOver)
			{
				_pUart->SendOver();
			}
			
			_pUart->Sending = 0;
		}
		else
		{
			/* ��������£��������˷�֧ */

			/* �������FIFO�����ݻ�δ��ϣ���ӷ���FIFOȡ1������д�뷢�����ݼĴ��� */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
}

/**
 * @name   USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler��
 * @brief  USART�жϷ������
 * @param  ��
 * @retval ��
 */
#if UART1_FIFO_EN == 1
void USART1_IRQHandler(void)
{
	UartIRQ(&g_tUart1);
}
#endif

#if UART2_FIFO_EN == 1
void USART2_IRQHandler(void)
{
	UartIRQ(&g_tUart2);
}
#endif

#if UART3_FIFO_EN == 1
void USART3_IRQHandler(void)
{
	UartIRQ(&g_tUart3);
}
#endif

#if UART4_FIFO_EN == 1
void UART4_IRQHandler(void)
{
	UartIRQ(&g_tUart4);
}
#endif

/**
 * @name   fputc
 * @brief  �ض���putc��������������ʹ��printf�����Ӵ���1��ӡ���
 * @param  ��
 * @retval ��
 */
int fputc(int ch, FILE *f)
{
#if 1	/* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
	comSendChar(COM2, ch);
	
	return ch;
#else	/* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
	/* дһ���ֽڵ�USART1 */
	USART1->DR = ch;
	
	/* �ȴ����ͽ��� */
	while((USART2->ISR & USART_ISR_TC) == 0)
	{}
	
	return ch;
#endif
}

/**
 * @name   fgetc
 * @brief  �ض���getc��������������ʹ��getchar�����Ӵ���1��������
 * @param  ��
 * @retval ��
 */
int fgetc(FILE *f)
{

#if 1	/* �Ӵ��ڽ���FIFO��ȡ1������, ֻ��ȡ�����ݲŷ��� */
	uint8_t ucData;

	while(comGetChar(COM2, &ucData) == 0);

	return ucData;
#else
	/* �ȴ����յ����� */
	while((USART2->ISR & USART_ISR_RXNE) == 0)
	{}

	return (int)USART2->RDR;
#endif
}
