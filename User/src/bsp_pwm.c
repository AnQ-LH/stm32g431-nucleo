#include "bsp_pwm.h"
#include "bsp.h"

/*
 	可以输出到GPIO的TIM通道:
	Advanced Motor Control 16-bit
	
	AF6
	
	TIM1_CH1,  PA8,
	TIM1_CH1N, PA7, PA11, PB13
	TIM1_CH2,  PA9,
	TIM1_CH2N, PB0, PA12, PB14
	TIM1_CH3,  PA10,
	TIM1_CH3N, PB1
	TIM1_CH4,  PA11(AF11)
	TIM1_CH4N, PC5
	
	AF4
	
	TIM8_CH1,  PC6
	TIM8_CH1N, PC10
	TIM8_CH2,  PC7
	TIM8_CH2N, PC11	
	TIM8_CH3,  PC8
	TIM8_CH3N, PC12
	TIM8_CH4,  PC9,	
	TIM8_CH4N, PD1
	
	General-purpose 32-bit
	AF1
	TIM2_CH1, PA0, PA5, PA15
	TIM2_CH2, PA1, PB3
	TIM2_CH3, PA2, PB10
	TIM2_CH4, PA3, PB11
	
	General-purpose 16-bit 
	AF2
	TIM3_CH1, PA6,  PC6, PB4
	TIM3_CH2, PA7,	PC7, PB5, PA4 
	TIM3_CH3, PB0,  PC8
	TIM3_CH4, PB1,  PC8
	
	AF2
	TIM4_CH1, PB6,	PA11(AF10)
	TIM4_CH2, PB7,	PA12(AF10)
	TIM4_CH3, PB8,	PA13(AF10)
	TIM4_CH4, PB9


	APB1 定时器有 TIM2, TIM3 ,TIM4, TIM6, TIM7
	APB2 定时器有 TIM1, TIM8 ,TIM15, TIM16, TIM17
	

	APB1 定时器的输入时钟 TIMxCLK = SystemCoreClock;	170M
	APB2 定时器的输入时钟 TIMxCLK = SystemCoreClock;	170M
*/


/**
 * @name   bsp_RCC_GPIO_Enable
 * @brief  使能GPIO时钟
 * @param  GPIOx GPIOA - GPIOK
 * @retval 无
 */
void bsp_RCC_GPIO_Enable(GPIO_TypeDef* GPIOx)
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
 * @name   bsp_RCC_TIM_Enable
 * @brief  使能TIM RCC 时钟
 * @param  TIMx TIM1 - TIM14
 * @retval 无
 */
void bsp_RCC_TIM_Enable(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	else if (TIMx == TIM2) LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	else if (TIMx == TIM3) LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	else if (TIMx == TIM4) LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	else if (TIMx == TIM8) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

	else
	{
		Error_Handler(__FILE__, __LINE__);
	}	
}

/**
 * @name   bsp_RCC_TIM_Disable
 * @brief  关闭TIM RCC 时钟
 * @param  TIMx TIM1 - TIM14
 * @retval TIM外设时钟名
 */
void bsp_RCC_TIM_Disable(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM1);
	else if (TIMx == TIM2) LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
	else if (TIMx == TIM3) LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
	else if (TIMx == TIM4) LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM4);
	else if (TIMx == TIM8) LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM8);
	else
	{
		Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * @name   bsp_GetAFofTIM
 * @brief  根据TIM 得到AF寄存器配置
 * @param  TIMx TIM1 - TIM14
 * @retval AF寄存器配置
 */
uint8_t bsp_GetAFofTIM(TIM_TypeDef* TIMx)
{
	uint8_t ret = 0;

	if (TIMx == TIM1) ret = LL_GPIO_AF_6;
	else if (TIMx == TIM2) ret = LL_GPIO_AF_1;
	else if (TIMx == TIM3) ret = LL_GPIO_AF_2;
	else if (TIMx == TIM4) ret = LL_GPIO_AF_2;
	else if (TIMx == TIM8) ret = LL_GPIO_AF_4;
	else
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	return ret;
}

/**
 * @name   bsp_ConfigTimGpio
 * @brief  配置GPIO和TIM时钟， GPIO连接到TIM输出通道
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @param  TIMx : TIM1 - TIM14
 * @retval AF寄存器配置
 */
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX, TIM_TypeDef* TIMx)
{
	LL_GPIO_InitTypeDef   GPIO_InitStruct;

	/* 使能GPIO时钟 */
	bsp_RCC_GPIO_Enable(GPIOx);

  	/* 使能TIM时钟 */
	bsp_RCC_TIM_Enable(TIMx);
	
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = bsp_GetAFofTIM(TIMx);
	GPIO_InitStruct.Pin = GPIO_PinX;
	LL_GPIO_Init(GPIOx, &GPIO_InitStruct);	
}

/**
 * @name   bsp_ConfigGpioOut
 * @brief  配置GPIO为推挽输出,主要用于PWM输出,占空比为0和100的情况。
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @retval 无
 */
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX)
{
	LL_GPIO_InitTypeDef   GPIO_InitStruct;

	bsp_RCC_GPIO_Enable(GPIOx);		/* 使能GPIO时钟 */

	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PinX;
	LL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * @name   bsp_SetTIMOutPWM
 * @brief  设置引脚输出的PWM信号的频率和占空比.当频率为0,并且占空为0时,关闭定时器,GPIO输出0;
		   当频率为0，占空比为100%时，GPIO输出1.
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @param  GPIO_Pin : GPIO_PIN_0 - GPIO__PIN_15
 * @param  TIMx : TIM1 - TIM17
 * @param  _ucChannel：使用的定时器通道，范围1 - 4
 * @param  _ulFreq : PWM信号频率，单位Hz (实际测试，可以输出100MHz），0 表示禁止输出
 * @param  _ulDutyCycle : PWM信号占空比,单位: 万分之一.如5000,表示50.00%的占空比
 * @retval 无
 */
void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx, uint8_t _ucChannel, uint32_t _ulFreq, uint32_t _ulDutyCycle)
{
	LL_TIM_InitTypeDef   TimHandle = {0};
	LL_TIM_OC_InitTypeDef  sConfig = {0};	
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t pulse;
	uint32_t uiTIMxCLK;
	const uint16_t TimChannel[6+1] = {0, LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4};

	if (_ucChannel > 6)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	if (_ulDutyCycle == 0)
	{		
		//bsp_RCC_TIM_Disable(TIMx);		/* 关闭TIM时钟, 可能影响其他通道 */		
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* 配置GPIO为推挽输出 */				
		LL_GPIO_ResetOutputPin(GPIOx, GPIO_Pin);	/* PWM = 0 */		
		return;
	}
	else if (_ulDutyCycle == 10000)
	{
		//bsp_RCC_TIM_Disable(TIMx);		/* 关闭TIM时钟, 可能影响其他通道 */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* 配置GPIO为推挽输出 */		
		LL_GPIO_SetOutputPin(GPIOx, GPIO_Pin);	/* PWM = 1 */			
		return;
	}
	
	/* 下面是PWM输出 */
	
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, TIMx);	/* 使能GPIO和TIM时钟，并连接TIM通道到GPIO */
	
	/*-----------------------------------------------------------------------

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		因为APB1 prescaler == 1, 所以 APB1上的TIMxCLK = PCLK1 = SystemCoreClock;
		因为APB2 prescaler == 1, 所以 APB2上的TIMxCLK = PCLK2 = SystemCoreClock;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 定时器有 TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8) )
	{
		/* APB2 定时器时钟 = 170M */
		uiTIMxCLK = SystemCoreClock;
	}
	else	
	{
		/* APB1 定时器 = 170M */
		uiTIMxCLK = SystemCoreClock;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* 分频比 = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* 分频比 = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else	/* 大于4K的频率，无需分频 */
	{
		usPrescaler = 0;					/* 分频比 = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* 自动重装的值 */
	}
	pulse = (_ulDutyCycle * usPeriod) / 10000;

	
	LL_TIM_DeInit(TIMx);
    
	/*  PWM频率 = TIMxCLK / usPrescaler + 1）/usPeriod + 1）*/
	TimHandle.Prescaler         = usPrescaler;
	TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;
	TimHandle.RepetitionCounter = 0;
	TimHandle.Autoreload  = usPeriod;
	if (LL_TIM_Init(TIMx, &TimHandle) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* 配置定时器PWM输出通道 */
	sConfig.OCMode       = LL_TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
	sConfig.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	sConfig.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;

	/* 占空比 */
//	LL_TIM_OC_SetCompareCH2(TIM2, pulse);
	sConfig.CompareValue  = pulse;
	if (LL_TIM_OC_Init(TIMx, TimChannel[_ucChannel], &sConfig) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	/* 启动PWM输出 */
	LL_TIM_EnableCounter(TIMx);
    LL_TIM_EnableAllOutputs(TIMx);
}

/**
 * @name   bsp_SetTIMforInt
 * @brief  配置TIM和NVIC，用于简单的定时中断，开启定时中断。另外注意中断服务程序需要由用户应用程序实现。
 * @param  TIMx : 定时器
 * @param  _ulFreq : 定时频率 （Hz）。 0 表示关闭。 
 * @param  _PreemptionPriority : 抢占优先级
 * @param  _SubPriority : 子优先级
 * @retval 无
 */
/*	
TIM定时中断服务程序范例，必须清中断标志
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_FLAG_UPDATE) != RESET)
	{
		TIM6->SR = ~ TIM_FLAG_UPDATE;
		//添加用户代码
	}
}
*/
void bsp_SetTIMforInt(TIM_TypeDef* TIMx, uint32_t _ulFreq, uint8_t _PreemptionPriority, uint8_t _SubPriority)
{
	LL_TIM_InitTypeDef   TimHandle = {0};
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;
	
	/* 使能TIM时钟 */
	bsp_RCC_TIM_Enable(TIMx);
	
	/*-----------------------------------------------------------------------

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		因为APB1 prescaler == 1, 所以 APB1上的TIMxCLK = PCLK1 = SystemCoreClock;
		因为APB2 prescaler == 1, 所以 APB2上的TIMxCLK = PCLK2 = SystemCoreClock;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 定时器有 TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8))
	{
		/* APB2 定时器时钟 = 170M */
		uiTIMxCLK = SystemCoreClock;
	}
	else	
	{
		/* APB1 定时器 = 84M */
		uiTIMxCLK = SystemCoreClock;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* 分频比 = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* 分频比 = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else	/* 大于4K的频率，无需分频 */
	{
		usPrescaler = 0;					/* 分频比 = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* 自动重装的值 */
	}

	/* 
       定时器中断更新周期 = TIMxCLK / usPrescaler + 1）/usPeriod + 1）
	*/
	
	/*  PWM频率 = TIMxCLK / usPrescaler + 1）/usPeriod + 1）*/
	TimHandle.Prescaler         = usPrescaler;
	TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;
	TimHandle.RepetitionCounter = 0;
	TimHandle.Autoreload  = usPeriod;
	if (LL_TIM_Init(TIMx, &TimHandle) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* 使能定时器中断  */
	LL_TIM_EnableIT_UPDATE(TIMx);

	/* 配置TIM定时更新中断 (Update) */
	{
        uint8_t irq = 0;	/* 中断号, 定义在 stm32g431xx.h */

        if (TIMx == TIM1) irq = TIM1_UP_TIM16_IRQn;
        else if (TIMx == TIM2) irq = TIM2_IRQn;
        else if (TIMx == TIM3) irq = TIM3_IRQn;
        else if (TIMx == TIM4) irq = TIM4_IRQn;
        else if (TIMx == TIM8) irq = TIM8_UP_IRQn;

        else
        {
            Error_Handler(__FILE__, __LINE__);
        }	
		
		NVIC_SetPriority((IRQn_Type)irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
		NVIC_EnableIRQ((IRQn_Type)irq);
	}
	
	LL_TIM_EnableCounter(TIMx);
}
