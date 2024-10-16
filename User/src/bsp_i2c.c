#include "bsp_i2c.h"
#include "bsp.h"

//#define SOFE_I2C
#define HARD_I2C

#ifdef SOFE_I2C

#define I2C_SCL_GPIO	GPIOB			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_GPIO	GPIOB			/* ���ӵ�SDA�����ߵ�GPIO */

#define I2C_SCL_PIN		LL_GPIO_PIN_6			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		LL_GPIO_PIN_9			/* ���ӵ�SDA�����ߵ�GPIO */

/* ʹ��GPIOʱ�� */
#define ALL_I2C_GPIO_CLK_ENABLE() {	    \
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);	\
	};

/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  I2C_SCL_GPIO->BSRR = I2C_SCL_PIN				/* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)				/* SCL = 0 */

#define I2C_SDA_1()  I2C_SDA_GPIO->BSRR = I2C_SDA_PIN				/* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)				/* SDA = 0 */

#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) != 0)	/* ��SCL����״̬ */

/**
 * @name   bsp_InitI2C
 * @brief  ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
 * @param  ��
 * @retval ��
 */
void bsp_InitI2C(void)
{
	LL_GPIO_InitTypeDef gpio_init;

	/* ��1������GPIOʱ�� */
	ALL_I2C_GPIO_CLK_ENABLE();
	
	/* ��2�����������еİ���GPIOΪ�������ģʽ */
	gpio_init.Mode		 = LL_GPIO_MODE_OUTPUT;				/* ������� */
	gpio_init.Pull		 = LL_GPIO_PULL_NO;					/* ����IO�������� */
	gpio_init.Speed		 = LL_GPIO_SPEED_FREQ_LOW;			/* GPIO�ٶȵȼ� */
	gpio_init.OutputType	 = LL_GPIO_OUTPUT_OPENDRAIN;	/* ��©��� */
	
	gpio_init.Pin = I2C_SCL_PIN;	
	LL_GPIO_Init(I2C_SCL_GPIO, &gpio_init);	
	
	gpio_init.Pin = I2C_SDA_PIN;		
	LL_GPIO_Init(I2C_SDA_GPIO, &gpio_init);	

	/* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
	i2c_Stop();
}

/**
 * @name   i2c_Delay
 * @brief  I2C����λ�ӳ٣����400KHz
 * @param  ��
 * @retval ��
 */
static void i2c_Delay(void)
{
	/*��
		CPU��Ƶ168MHzʱ�����ڲ�Flash����, MDK���̲��Ż�����̨ʽʾ�����۲Ⲩ�Ρ�
		ѭ������Ϊ5ʱ��SCLƵ�� = 1.78MHz (����ʱ: 92ms, ��д������������ʾ����̽ͷ���ϾͶ�дʧ�ܡ�ʱ��ӽ��ٽ�)
		ѭ������Ϊ10ʱ��SCLƵ�� = 1.1MHz (����ʱ: 138ms, ���ٶ�: 118724B/s)
		ѭ������Ϊ30ʱ��SCLƵ�� = 440KHz�� SCL�ߵ�ƽʱ��1.0us��SCL�͵�ƽʱ��1.2us

		��������ѡ��2.2Kŷʱ��SCL������ʱ��Լ0.5us�����ѡ4.7Kŷ����������Լ1us

		ʵ��Ӧ��ѡ��400KHz���ҵ����ʼ���
	*/
	//for (i = 0; i < 30; i++);
	//for (i = 0; i < 60; i++);
	//bsp_DelayUS(2); 229.57KHzʱ��
	bsp_DelayUS(2);
}

/**
 * @name   i2c_Start
 * @brief  CPU����I2C���������ź�
 * @param  ��
 * @retval ��
 */
void i2c_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
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
 * @brief  CPU����I2C����ֹͣ�ź�
 * @param  ��
 * @retval ��
 */
void i2c_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/**
 * @name   i2c_SendByte
 * @brief  CPU��I2C�����豸����8bit����
 * @param  _ucByte �� �ȴ����͵��ֽ�
 * @retval ��
 */
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
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
		I2C_SCL_0();	/* 2019-03-14 ���GT811���ݴ��������һ�У��൱���ӳټ�ʮns */
		if (i == 7)
		{
			 I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */	
	}
}

/**
 * @name   i2c_ReadByte
 * @brief  CPU��I2C�����豸��ȡ8bit����
 * @param  ��
 * @retval ����������
 */
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
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
 * @brief  CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
 * @param  ��
 * @retval ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
 */
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
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
 * @brief  CPU����һ��ACK�ź�
 * @param  ��
 * @retval ��
 */
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	
	i2c_Delay();
}

/**
 * @name   i2c_NAck
 * @brief  CPU����1��NACK�ź�
 * @param  ��
 * @retval ��
 */
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/**
 * @name   i2c_CheckDevice
 * @brief  ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
 * @param  _Address���豸��I2C���ߵ�ַ
 * @retval ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
 */
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* ���������ź� */

		/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* ����豸��ACKӦ�� */

		i2c_Stop();			/* ����ֹͣ�ź� */

		return ucAck;
	}
	return 1;	/* I2C�����쳣 */
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

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
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
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_ALTERNATE;				/* ������� */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;					/* ����IO�������� */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_LOW;			/* GPIO�ٶȵȼ� */
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_OPENDRAIN;		/* ��©��� */
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
 * @brief  I2C ����ģʽ����
 * @param  ��
 * @retval ��
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
  * @brief  I2C �����ʼ��
  * @param  ��
  * @retval ��
  */
void bsp_InitI2C(void)
{
	LL_I2C_MspInit(&I2C_Handle);
	I2C_Mode_Config();
}

#endif  /* HARD_IIC */

