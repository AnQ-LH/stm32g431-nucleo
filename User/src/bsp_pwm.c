#include "bsp_pwm.h"
#include "bsp.h"

/*
 	���������GPIO��TIMͨ��:
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


	APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM6, TIM7
	APB2 ��ʱ���� TIM1, TIM8 ,TIM15, TIM16, TIM17
	

	APB1 ��ʱ��������ʱ�� TIMxCLK = SystemCoreClock;	170M
	APB2 ��ʱ��������ʱ�� TIMxCLK = SystemCoreClock;	170M
*/


/**
 * @name   bsp_RCC_GPIO_Enable
 * @brief  ʹ��GPIOʱ��
 * @param  GPIOx GPIOA - GPIOK
 * @retval ��
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
 * @brief  ʹ��TIM RCC ʱ��
 * @param  TIMx TIM1 - TIM14
 * @retval ��
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
 * @brief  �ر�TIM RCC ʱ��
 * @param  TIMx TIM1 - TIM14
 * @retval TIM����ʱ����
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
 * @brief  ����TIM �õ�AF�Ĵ�������
 * @param  TIMx TIM1 - TIM14
 * @retval AF�Ĵ�������
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
 * @brief  ����GPIO��TIMʱ�ӣ� GPIO���ӵ�TIM���ͨ��
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @param  TIMx : TIM1 - TIM14
 * @retval AF�Ĵ�������
 */
void bsp_ConfigTimGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX, TIM_TypeDef* TIMx)
{
	LL_GPIO_InitTypeDef   GPIO_InitStruct;

	/* ʹ��GPIOʱ�� */
	bsp_RCC_GPIO_Enable(GPIOx);

  	/* ʹ��TIMʱ�� */
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
 * @brief  ����GPIOΪ�������,��Ҫ����PWM���,ռ�ձ�Ϊ0��100�������
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @retval ��
 */
void bsp_ConfigGpioOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX)
{
	LL_GPIO_InitTypeDef   GPIO_InitStruct;

	bsp_RCC_GPIO_Enable(GPIOx);		/* ʹ��GPIOʱ�� */

	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PinX;
	LL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * @name   bsp_SetTIMOutPWM
 * @brief  �������������PWM�źŵ�Ƶ�ʺ�ռ�ձ�.��Ƶ��Ϊ0,����ռ��Ϊ0ʱ,�رն�ʱ��,GPIO���0;
		   ��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1.
 * @param  GPIOx : GPIOA - GPIOK
 * @param  GPIO_PinX : GPIO_PIN_0 - GPIO__PIN_15
 * @param  GPIO_Pin : GPIO_PIN_0 - GPIO__PIN_15
 * @param  TIMx : TIM1 - TIM17
 * @param  _ucChannel��ʹ�õĶ�ʱ��ͨ������Χ1 - 4
 * @param  _ulFreq : PWM�ź�Ƶ�ʣ���λHz (ʵ�ʲ��ԣ��������100MHz����0 ��ʾ��ֹ���
 * @param  _ulDutyCycle : PWM�ź�ռ�ձ�,��λ: ���֮һ.��5000,��ʾ50.00%��ռ�ձ�
 * @retval ��
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
		//bsp_RCC_TIM_Disable(TIMx);		/* �ر�TIMʱ��, ����Ӱ������ͨ�� */		
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* ����GPIOΪ������� */				
		LL_GPIO_ResetOutputPin(GPIOx, GPIO_Pin);	/* PWM = 0 */		
		return;
	}
	else if (_ulDutyCycle == 10000)
	{
		//bsp_RCC_TIM_Disable(TIMx);		/* �ر�TIMʱ��, ����Ӱ������ͨ�� */
		bsp_ConfigGpioOut(GPIOx, GPIO_Pin);	/* ����GPIOΪ������� */		
		LL_GPIO_SetOutputPin(GPIOx, GPIO_Pin);	/* PWM = 1 */			
		return;
	}
	
	/* ������PWM��� */
	
	bsp_ConfigTimGpio(GPIOx, GPIO_Pin, TIMx);	/* ʹ��GPIO��TIMʱ�ӣ�������TIMͨ����GPIO */
	
	/*-----------------------------------------------------------------------

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		��ΪAPB1 prescaler == 1, ���� APB1�ϵ�TIMxCLK = PCLK1 = SystemCoreClock;
		��ΪAPB2 prescaler == 1, ���� APB2�ϵ�TIMxCLK = PCLK2 = SystemCoreClock;

		APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 ��ʱ���� TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8) )
	{
		/* APB2 ��ʱ��ʱ�� = 170M */
		uiTIMxCLK = SystemCoreClock;
	}
	else	
	{
		/* APB1 ��ʱ�� = 170M */
		uiTIMxCLK = SystemCoreClock;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* ��Ƶ�� = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* ��Ƶ�� = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else	/* ����4K��Ƶ�ʣ������Ƶ */
	{
		usPrescaler = 0;					/* ��Ƶ�� = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* �Զ���װ��ֵ */
	}
	pulse = (_ulDutyCycle * usPeriod) / 10000;

	
	LL_TIM_DeInit(TIMx);
    
	/*  PWMƵ�� = TIMxCLK / usPrescaler + 1��/usPeriod + 1��*/
	TimHandle.Prescaler         = usPrescaler;
	TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;
	TimHandle.RepetitionCounter = 0;
	TimHandle.Autoreload  = usPeriod;
	if (LL_TIM_Init(TIMx, &TimHandle) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* ���ö�ʱ��PWM���ͨ�� */
	sConfig.OCMode       = LL_TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
	sConfig.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	sConfig.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;

	/* ռ�ձ� */
//	LL_TIM_OC_SetCompareCH2(TIM2, pulse);
	sConfig.CompareValue  = pulse;
	if (LL_TIM_OC_Init(TIMx, TimChannel[_ucChannel], &sConfig) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}
	
	/* ����PWM��� */
	LL_TIM_EnableCounter(TIMx);
    LL_TIM_EnableAllOutputs(TIMx);
}

/**
 * @name   bsp_SetTIMforInt
 * @brief  ����TIM��NVIC�����ڼ򵥵Ķ�ʱ�жϣ�������ʱ�жϡ�����ע���жϷ��������Ҫ���û�Ӧ�ó���ʵ�֡�
 * @param  TIMx : ��ʱ��
 * @param  _ulFreq : ��ʱƵ�� ��Hz���� 0 ��ʾ�رա� 
 * @param  _PreemptionPriority : ��ռ���ȼ�
 * @param  _SubPriority : �����ȼ�
 * @retval ��
 */
/*	
TIM��ʱ�жϷ�����������������жϱ�־
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_FLAG_UPDATE) != RESET)
	{
		TIM6->SR = ~ TIM_FLAG_UPDATE;
		//����û�����
	}
}
*/
void bsp_SetTIMforInt(TIM_TypeDef* TIMx, uint32_t _ulFreq, uint8_t _PreemptionPriority, uint8_t _SubPriority)
{
	LL_TIM_InitTypeDef   TimHandle = {0};
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;
	
	/* ʹ��TIMʱ�� */
	bsp_RCC_TIM_Enable(TIMx);
	
	/*-----------------------------------------------------------------------

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		��ΪAPB1 prescaler == 1, ���� APB1�ϵ�TIMxCLK = PCLK1 = SystemCoreClock;
		��ΪAPB2 prescaler == 1, ���� APB2�ϵ�TIMxCLK = PCLK2 = SystemCoreClock;

		APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 ��ʱ���� TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	if ((TIMx == TIM1) || (TIMx == TIM8))
	{
		/* APB2 ��ʱ��ʱ�� = 170M */
		uiTIMxCLK = SystemCoreClock;
	}
	else	
	{
		/* APB1 ��ʱ�� = 84M */
		uiTIMxCLK = SystemCoreClock;
	}

	if (_ulFreq < 100)
	{
		usPrescaler = 10000 - 1;					/* ��Ƶ�� = 10000 */
		usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else if (_ulFreq < 3000)
	{
		usPrescaler = 100 - 1;					/* ��Ƶ�� = 100 */
		usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
	}
	else	/* ����4K��Ƶ�ʣ������Ƶ */
	{
		usPrescaler = 0;					/* ��Ƶ�� = 1 */
		usPeriod = uiTIMxCLK / _ulFreq - 1;	/* �Զ���װ��ֵ */
	}

	/* 
       ��ʱ���жϸ������� = TIMxCLK / usPrescaler + 1��/usPeriod + 1��
	*/
	
	/*  PWMƵ�� = TIMxCLK / usPrescaler + 1��/usPeriod + 1��*/
	TimHandle.Prescaler         = usPrescaler;
	TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;
	TimHandle.RepetitionCounter = 0;
	TimHandle.Autoreload  = usPeriod;
	if (LL_TIM_Init(TIMx, &TimHandle) != SUCCESS)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* ʹ�ܶ�ʱ���ж�  */
	LL_TIM_EnableIT_UPDATE(TIMx);

	/* ����TIM��ʱ�����ж� (Update) */
	{
        uint8_t irq = 0;	/* �жϺ�, ������ stm32g431xx.h */

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
