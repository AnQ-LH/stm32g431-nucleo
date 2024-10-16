#include "bsp_uart.h"
#include "bsp.h"

/*-----------------------------------------------------------------------
		system_stm32g4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		APB1 串口有 USART2, USART3, UART4
		APB2 串口有 USART1

	----------------------------------------------------------------------- */
	
/* 串口1的GPIO  PA9, PA10 */
#define USART1_CLK_ENABLE()              LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

#define USART1_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_PIN                    LL_GPIO_PIN_9
#define USART1_TX_AF                     LL_GPIO_AF_7

#define USART1_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_PIN                    LL_GPIO_PIN_10
#define USART1_RX_AF                     LL_GPIO_AF_7

/* 串口2的GPIO  PA2 PA3 */
#define USART2_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

#define USART2_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_PIN                    LL_GPIO_PIN_2
#define USART2_TX_AF                     LL_GPIO_AF_7

#define USART2_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_PIN                    LL_GPIO_PIN_3
#define USART2_RX_AF                     LL_GPIO_AF_7

/* 串口3的GPIO  PD8 PD9 */
#define USART3_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

#define USART3_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
#define USART3_TX_GPIO_PORT              GPIOD
#define USART3_TX_PIN                    LL_GPIO_PIN_8
#define USART3_TX_AF                     LL_GPIO_AF_7

#define USART3_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
#define USART3_RX_GPIO_PORT              GPIOD
#define USART3_RX_PIN                    LL_GPIO_PIN_9
#define USART3_RX_AF                     LL_GPIO_AF_7

/* 串口4的GPIO  PC10 PC11 */
#define UART4_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);

#define UART4_TX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define UART4_TX_GPIO_PORT              GPIOC
#define UART4_TX_PIN                    LL_GPIO_PIN_10
#define UART4_TX_AF                     LL_GPIO_AF_8

#define UART4_RX_GPIO_CLK_ENABLE()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define UART4_RX_GPIO_PORT              GPIOC
#define UART4_RX_PIN                    LL_GPIO_PIN_11
#define UART4_RX_AF                     LL_GPIO_AF_8


/* 定义每个串口结构体变量 */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART4_FIFO_EN == 1
	static UART_T g_tUart4;
	static uint8_t g_TxBuf4[UART4_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf4[UART4_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif


static void UartVarInit(void);
static void InitHardUart(void);
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);


/**
 * @name   bsp_InitUart
 * @brief  初始化串口硬件，并对全局变量赋初值.
 * @param  无
 * @retval 无
 */
void bsp_InitUart(void)
{
	
	UartVarInit();		/* 必须先初始化全局变量,再配置硬件 */

	InitHardUart();		/* 配置串口的硬件参数(波特率等) */
}

/**
 * @name   ComToUart
 * @brief  将COM端口号转换为UART指针
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @retval uart指针
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
 * @brief  将COM端口号转换为 USART_TypeDef* USARTx
 * @param  _ucPort: 端口号(COM1 - COM8)
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
		/* 不做任何处理 */
		return 0;
	}
}

/**
 * @name   comSendBuf
 * @brief  向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @param  _ucaBuf: 待发送的数据缓冲区
 * @param  _usLen : 数据长度
 * @retval 无
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
		pUart->SendBefor();		/* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */
	}

	UartSend(pUart, _ucaBuf, _usLen);
}

/**
 * @name   comSendChar
 * @brief  向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @param  _ucByte: 待发送的数据
 * @retval 无
 */
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

/**
 * @name   comGetChar
 * @brief  从接收缓冲区读取1字节，非阻塞。无论有无数据均立即返回.
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @param  _pByte: 接收到的数据存放在这个地址
 * @retval 0 表示无数据, 1 表示读取到有效字节
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
 * @brief  清零串口发送缓冲区
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @retval 无
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
 * @brief  清零串口接收缓冲区
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @retval 无
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
 * @brief  设置串口的波特率. 本函数固定设置为无校验,收发都使能模式
 * @param  _ucPort: 端口号(COM1 - COM8)
 * @param  _BaudRate: 波特率  8倍过采样  波特率.0-12.5Mbps
							 16倍过采样 波特率.0-6.25Mbps
 * @retval 无
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
 * @brief  初始化串口相关的变量
 * @param  无
 * @retval 无
 */
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 串口设备 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* 发送缓冲区指针 */
	g_tUart1.pRxBuf = g_RxBuf1;					/* 接收缓冲区指针 */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart1.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart1.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart1.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart1.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart1.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart1.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart1.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart1.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart1.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
	g_tUart1.Sending = 0;						/* 正在发送中标志 */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;						/* STM32 串口设备 */
	g_tUart2.pTxBuf = g_TxBuf2;					/* 发送缓冲区指针 */
	g_tUart2.pRxBuf = g_RxBuf2;					/* 接收缓冲区指针 */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart2.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart2.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart2.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart2.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart2.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart2.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart2.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart2.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart2.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
	g_tUart2.Sending = 0;						/* 正在发送中标志 */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;						/* STM32 串口设备 */
	g_tUart3.pTxBuf = g_TxBuf3;					/* 发送缓冲区指针 */
	g_tUart3.pRxBuf = g_RxBuf3;					/* 接收缓冲区指针 */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart3.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart3.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart3.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart3.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart3.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart3.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart3.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart3.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart3.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
	g_tUart3.Sending = 0;						/* 正在发送中标志 */
#endif

#if UART4_FIFO_EN == 1
	g_tUart4.uart = UART4;						/* STM32 串口设备 */
	g_tUart4.pTxBuf = g_TxBuf4;					/* 发送缓冲区指针 */
	g_tUart4.pRxBuf = g_RxBuf4;					/* 接收缓冲区指针 */
	g_tUart4.usTxBufSize = UART4_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart4.usRxBufSize = UART4_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart4.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart4.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart4.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart4.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart4.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart4.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart4.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart4.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart4.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
	g_tUart4.Sending = 0;						/* 正在发送中标志 */
#endif

}

/**
 * @name   bsp_SetUartParam
 * @brief  配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32- H7开发板
 * @param  Instance   USART_TypeDef类型结构体
 * @param  BaudRate   波特率
 * @param  Parity     校验类型，奇校验或者偶校验
 * @param  Mode       发送和接收模式使能
 * @retval 无
 */
void bsp_SetUartParam(USART_TypeDef *Instance,  uint32_t BaudRate, uint32_t Parity, uint32_t Mode)
{
	LL_USART_InitTypeDef UartHandle;	
	
	/*##-1- 配置串口硬件参数 ######################################*/
	/* 异步串口模式 (UART Mode) */
	/* 配置如下:
	  - 字长    = 8 位
	  - 停止位  = 1 个停止位
	  - 校验    = 参数Parity
	  - 波特率  = 参数BaudRate
	  - 硬件流控制关闭 (RTS and CTS signals) */
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
 * @brief  配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32- H7开发板
 * @param  无
 * @retval 无
 */
static void InitHardUart(void)
{
	LL_GPIO_InitTypeDef  GPIO_InitStruct;


#if UART1_FIFO_EN == 1		/* 串口1 */
	/* 使能 GPIO TX/RX 时钟 */
	USART1_TX_GPIO_CLK_ENABLE();
	USART1_RX_GPIO_CLK_ENABLE();
	
	/* 使能 USARTx 时钟 */
	USART1_CLK_ENABLE();	

	/* 配置TX引脚 */
	GPIO_InitStruct.Pin      	 = USART1_TX_PIN;
	GPIO_InitStruct.Mode     	 = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Pull     	 = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Speed    	 = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate	 = USART1_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	
	LL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* 配置RX引脚 */
	GPIO_InitStruct.Pin = USART1_RX_PIN;
	GPIO_InitStruct.Alternate = USART1_RX_AF;
	LL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

	/* 配置NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(USART1,  UART1_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ICR, USART_ICR_TCCF);   /* 清除TC发送完成标志 */
    CLEAR_BIT(USART1->RQR, USART_RQR_RXFRQ); /* 清除RXNE接收标志 */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE);	/* 使能PE. RX接受中断 */
#endif

#if UART2_FIFO_EN == 1		/* 串口2 */
	/* 使能 GPIO TX/RX 时钟 */
	USART2_TX_GPIO_CLK_ENABLE();
	USART2_RX_GPIO_CLK_ENABLE();
	
	/* 使能 USARTx 时钟 */
	USART2_CLK_ENABLE();	

	/* 配置TX引脚 */
	GPIO_InitStruct.Pin			 = USART2_TX_PIN;
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate	 = USART2_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* 配置RX引脚 */
	GPIO_InitStruct.Pin = USART2_RX_PIN;
	GPIO_InitStruct.Alternate = USART2_RX_AF;
	LL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

	/* 配置NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(USART2,  UART2_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);
	
	CLEAR_BIT(USART1->ICR, USART_ICR_TCCF);   /* 清除TC发送完成标志 */
    CLEAR_BIT(USART1->RQR, USART_RQR_RXFRQ); /* 清除RXNE接收标志 */
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);	/* 使能PE. RX接受中断 */
#endif

#if UART3_FIFO_EN == 1			/* 串口3 */
	/* 使能 GPIO TX/RX 时钟 */
	USART3_TX_GPIO_CLK_ENABLE();
	USART3_RX_GPIO_CLK_ENABLE();
	
	/* 使能 USARTx 时钟 */
	USART3_CLK_ENABLE();	

	/* 配置TX引脚 */
	GPIO_InitStruct.Pin			 = USART3_TX_PIN;
	GPIO_InitStruct.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		 = GPIO_PULLUP;
	GPIO_InitStruct.Speed		 = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	 = USART3_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* 配置RX引脚 */
	GPIO_InitStruct.Pin = USART3_RX_PIN;
	GPIO_InitStruct.Alternate = USART3_RX_AF;
	HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);

	/* 配置NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(USART3,  UART3_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ISR, USART_ISR_TC);   /* 清除TC发送完成标志 */
    CLEAR_BIT(USART1->ISR, USART_ISR_RXNE); /* 清除RXNE接收标志 */
	SET_BIT(USART3->CR1, USART_CR1_RXNEIE);	/* 使能PE. RX接受中断 */
#endif

#if UART4_FIFO_EN == 1
	/* 使能 GPIO TX/RX 时钟 */
	UART4_TX_GPIO_CLK_ENABLE();
	UART4_RX_GPIO_CLK_ENABLE();
	
	/* 使能 USARTx 时钟 */
	UART4_CLK_ENABLE();	

	/* 配置TX引脚 */
	GPIO_InitStruct.Pin			 = UART4_TX_PIN;
	GPIO_InitStruct.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		 = GPIO_PULLUP;
	GPIO_InitStruct.Speed		 = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	 = UART4_TX_AF;
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;
	HAL_GPIO_Init(UART4_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* 配置RX引脚 */
	GPIO_InitStruct.Pin = UART4_RX_PIN;
	GPIO_InitStruct.Alternate = UART4_RX_AF;
	HAL_GPIO_Init(UART4_RX_GPIO_PORT, &GPIO_InitStruct);

	/* 配置NVIC the NVIC for UART */   
	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
	NVIC_EnableIRQ(USART2_IRQn);
  
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(UART4,  UART4_BAUD, LL_USART_PARITY_NONE, LL_USART_DIRECTION_TX_RX);

	CLEAR_BIT(USART1->ISR, USART_ISR_TC);   /* 清除TC发送完成标志 */
    CLEAR_BIT(USART1->ISR, USART_ISR_RXNE); /* 清除RXNE接收标志 */
	SET_BIT(UART4->CR1, USART_CR1_RXNEIE);	/* 使能RX接受中断 */
#endif

}

/**
 * @name   UartSend
 * @brief  填写数据到UART发送缓冲区,并启动发送中断。中断处理函数发送完毕后，自动关闭发送中断
 * @param  无
 * @retval 无
 */
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* 如果发送缓冲区已经满了，则等待缓冲区空 */
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
			else if(usCount == _pUart->usTxBufSize)/* 数据已填满缓冲区 */
			{
				if((_pUart->uart->CR1 & USART_CR1_TXEIE) == 0)
				{
					SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);
				}  
			}
		}

		/* 将新数据填入发送缓冲区 */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);	/* 使能发送中断（缓冲区空） */
}

/**
 * @name   UartGetChar
 * @brief  从串口接收缓冲区读取1字节数据 （用于主程序调用）
 * @param  _pUart : 串口设备
 * @param  _pByte : 存放读取数据的指针
 * @retval 无
 */
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
	DISABLE_INT();
	usCount = _pUart->usRxCount;
	ENABLE_INT();

	/* 如果读和写索引相同，则返回0 */
	//if (_pUart->usRxRead == usRxWrite)
	if (usCount == 0)	/* 已经没有数据 */
	{
		return 0;
	}
	else
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead];		/* 从串口接收FIFO取1个数据 */

		/* 改写FIFO读索引 */
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
 * @brief  判断发送缓冲区是否为空.
 * @param  _pUart : 串口设备
 * @retval 1为空,0为不空.
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
 * @brief  供中断服务程序调用，通用串口中断处理函数.
 * @param  _pUart : 串口设备
 * @retval 无
 */
static void UartIRQ(UART_T *_pUart)
{
	uint32_t isrflags   = READ_REG(_pUart->uart->ISR);
	uint32_t cr1its     = READ_REG(_pUart->uart->CR1);
	uint32_t cr3its     = READ_REG(_pUart->uart->CR3);
	
	/* 处理接收中断  */
	if ((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
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

		/* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch); /* 比如，交给MODBUS解码程序处理字节流 */
			}
		}
	}

	/* 处理发送缓冲区空中断 */
	if ( ((isrflags & USART_ISR_TXE_TXFNF) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
			//USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

			/* 使能数据发送完毕中断 */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			_pUart->Sending = 1;
			
			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}

	}
	/* 数据bit位全部发送完毕的中断 */
	if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

			/* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
			if (_pUart->SendOver)
			{
				_pUart->SendOver();
			}
			
			_pUart->Sending = 0;
		}
		else
		{
			/* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
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
 * @name   USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler等
 * @brief  USART中断服务程序
 * @param  无
 * @retval 无
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
 * @brief  重定义putc函数，这样可以使用printf函数从串口1打印输出
 * @param  无
 * @retval 无
 */
int fputc(int ch, FILE *f)
{
#if 1	/* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	comSendChar(COM2, ch);
	
	return ch;
#else	/* 采用阻塞方式发送每个字符,等待数据发送完毕 */
	/* 写一个字节到USART1 */
	USART1->DR = ch;
	
	/* 等待发送结束 */
	while((USART2->ISR & USART_ISR_TC) == 0)
	{}
	
	return ch;
#endif
}

/**
 * @name   fgetc
 * @brief  重定义getc函数，这样可以使用getchar函数从串口1输入数据
 * @param  无
 * @retval 无
 */
int fgetc(FILE *f)
{

#if 1	/* 从串口接收FIFO中取1个数据, 只有取到数据才返回 */
	uint8_t ucData;

	while(comGetChar(COM2, &ucData) == 0);

	return ucData;
#else
	/* 等待接收到数据 */
	while((USART2->ISR & USART_ISR_RXNE) == 0)
	{}

	return (int)USART2->RDR;
#endif
}
