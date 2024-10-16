#include "bsp_i2c.h"
#include "bsp.h"

//#define SOFE_I2C
#define HARD_I2C

#ifdef SOFE_I2C

#define I2C_SCL_GPIO	GPIOB			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_GPIO	GPIOB			/* 连接到SDA数据线的GPIO */

#define I2C_SCL_PIN		LL_GPIO_PIN_6			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		LL_GPIO_PIN_9			/* 连接到SDA数据线的GPIO */

/* 使能GPIO时钟 */
#define ALL_I2C_GPIO_CLK_ENABLE() {	    \
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);	\
	};

/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  I2C_SCL_GPIO->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)				/* SCL = 0 */

#define I2C_SDA_1()  I2C_SDA_GPIO->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)				/* SDA = 0 */

#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) != 0)	/* 读SCL口线状态 */

/**
 * @name   bsp_InitI2C
 * @brief  配置I2C总线的GPIO，采用模拟IO的方式实现
 * @param  无
 * @retval 无
 */
void bsp_InitI2C(void)
{
	LL_GPIO_InitTypeDef gpio_init;

	/* 第1步：打开GPIO时钟 */
	ALL_I2C_GPIO_CLK_ENABLE();
	
	/* 第2步：配置所有的按键GPIO为推挽输出模式 */
	gpio_init.Mode		 = LL_GPIO_MODE_OUTPUT;				/* 设置输出 */
	gpio_init.Pull		 = LL_GPIO_PULL_NO;					/* 设置IO无上下拉 */
	gpio_init.Speed		 = LL_GPIO_SPEED_FREQ_LOW;			/* GPIO速度等级 */
	gpio_init.OutputType	 = LL_GPIO_OUTPUT_OPENDRAIN;	/* 开漏输出 */
	
	gpio_init.Pin = I2C_SCL_PIN;	
	LL_GPIO_Init(I2C_SCL_GPIO, &gpio_init);	
	
	gpio_init.Pin = I2C_SDA_PIN;		
	LL_GPIO_Init(I2C_SDA_GPIO, &gpio_init);	

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}

/**
 * @name   i2c_Delay
 * @brief  I2C总线位延迟，最快400KHz
 * @param  无
 * @retval 无
 */
static void i2c_Delay(void)
{
	/*　
		CPU主频168MHz时，在内部Flash运行, MDK工程不优化。用台式示波器观测波形。
		循环次数为5时，SCL频率 = 1.78MHz (读耗时: 92ms, 读写正常，但是用示波器探头碰上就读写失败。时序接近临界)
		循环次数为10时，SCL频率 = 1.1MHz (读耗时: 138ms, 读速度: 118724B/s)
		循环次数为30时，SCL频率 = 440KHz， SCL高电平时间1.0us，SCL低电平时间1.2us

		上拉电阻选择2.2K欧时，SCL上升沿时间约0.5us，如果选4.7K欧，则上升沿约1us

		实际应用选择400KHz左右的速率即可
	*/
	//for (i = 0; i < 30; i++);
	//for (i = 0; i < 60; i++);
	//bsp_DelayUS(2); 229.57KHz时钟
	bsp_DelayUS(2);
}

/**
 * @name   i2c_Start
 * @brief  CPU发起I2C总线启动信号
 * @param  无
 * @retval 无
 */
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	
	I2C_SCL_0();
	i2c_Delay();
}

/**
 * @name   i2c_Stop
 * @brief  CPU发起I2C总线停止信号
 * @param  无
 * @retval 无
 */
void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/**
 * @name   i2c_SendByte
 * @brief  CPU向I2C总线设备发送8bit数据
 * @param  _ucByte ： 等待发送的字节
 * @retval 无
 */
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		I2C_SCL_0();	/* 2019-03-14 针对GT811电容触摸，添加一行，相当于延迟几十ns */
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */	
	}
}

/**
 * @name   i2c_ReadByte
 * @brief  CPU从I2C总线设备读取8bit数据
 * @param  无
 * @retval 读到的数据
 */
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/**
 * @name   i2c_WaitAck
 * @brief  CPU产生一个时钟，并读取器件的ACK应答信号
 * @param  无
 * @retval 返回0表示正确应答，1表示无器件响应
 */
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/**
 * @name   i2c_Ack
 * @brief  CPU产生一个ACK信号
 * @param  无
 * @retval 无
 */
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
	
	i2c_Delay();
}

/**
 * @name   i2c_NAck
 * @brief  CPU产生1个NACK信号
 * @param  无
 * @retval 无
 */
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/**
 * @name   i2c_CheckDevice
 * @brief  检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 * @param  _Address：设备的I2C总线地址
 * @retval 返回值 0 表示正确， 返回1表示未探测到
 */
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* 发送启动信号 */

		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

		i2c_Stop();			/* 发送停止信号 */

		return ucAck;
	}
	return 1;	/* I2C总线异常 */
}

#endif  /* SOFE_IIC */

#ifdef HARD_I2C

#define I2Cx                             I2C1
#define I2Cx_CLK_ENABLE()                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1)
#define I2Cx_SDA_GPIO_CLK_ENABLE()       LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define I2Cx_SCL_GPIO_CLK_ENABLE()       LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)

#define I2Cx_FORCE_RESET()               LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1)
#define I2Cx_RELEASE_RESET()             LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1)

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    LL_GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     LL_GPIO_AF_4
#define I2Cx_SDA_PIN                    LL_GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     LL_GPIO_AF_4

/* 这个地址只要与STM32外挂的I2C器件地址不一样即可 */
#define I2C_OWN_ADDRESS7      0X0A  

LL_I2C_InitTypeDef  I2C_Handle; 

/**
 * @brief  I2C MSP Initialization 
 *         This function configures the hardware resources used in this example: 
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration  
 *           - DMA configuration for transmission request by peripheral 
 *           - NVIC configuration for DMA interrupt request enable
 * @param  hi2c: I2C handle pointer
 * @retval None
 */
void LL_I2C_MspInit(LL_I2C_InitTypeDef *hi2c)
{  
	LL_GPIO_InitTypeDef  GPIO_InitStruct;
  
	/* Enable peripherals and GPIO Clocks */
	/* Enable GPIO TX/RX clock */
	I2Cx_SCL_GPIO_CLK_ENABLE();
	I2Cx_SDA_GPIO_CLK_ENABLE();
	/* Enable I2C1 clock */
	I2Cx_CLK_ENABLE(); 
	  
	/* Configure peripheral GPIO */  
	/* I2C TX GPIO pin configuration  */
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_ALTERNATE;				/* 设置输出 */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;					/* 设置IO无上下拉 */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_LOW;			/* GPIO速度等级 */
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_OPENDRAIN;		/* 开漏输出 */
	GPIO_InitStruct.Alternate = I2Cx_SCL_AF;
	
	GPIO_InitStruct.Pin = I2Cx_SCL_PIN;	
	LL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
	
	
	
	/* I2C RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
	GPIO_InitStruct.Alternate = I2Cx_SDA_AF;
	LL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
	
  	/* Force the I2C peripheral clock reset */  
	I2Cx_FORCE_RESET() ; 

	/* Release the I2C peripheral clock reset */  
	I2Cx_RELEASE_RESET(); 
}

/**
 * @brief  I2C 工作模式配置
 * @param  无
 * @retval 无
 */
static void I2C_Mode_Config(void)
{
	I2C_Handle.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_Handle.Timing = 0x30A0A7FB;
	I2C_Handle.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_Handle.DigitalFilter = 0;
	I2C_Handle.OwnAddress1 = 0;
	I2C_Handle.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	I2C_Handle.TypeAcknowledge = LL_I2C_ACK;
	/* Init the I2C */
	LL_I2C_Init(I2Cx, &I2C_Handle);	

	
	LL_I2C_EnableAutoEndMode(I2Cx);
	LL_I2C_SetOwnAddress2(I2Cx, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2Cx);
	LL_I2C_DisableGeneralCall(I2Cx);
	LL_I2C_EnableClockStretching(I2Cx);
	
	/* I2C Fast mode Plus enable */
	LL_SYSCFG_EnableFastModePlus(LL_SYSCFG_I2C_FASTMODEPLUS_I2C1);  
}

/**
  * @brief  I2C 外设初始化
  * @param  无
  * @retval 无
  */
void bsp_InitI2C(void)
{
	LL_I2C_MspInit(&I2C_Handle);
	I2C_Mode_Config();
}

#endif  /* HARD_IIC */

