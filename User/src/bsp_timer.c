#include "bsp_timer.h"
#include "bsp.h"
/*
 * ��������Ӳ����ʱ����TIM�� ����ʹ TIM2 - TIM5
 */
#define USE_TIM2
//#define USE_TIM3
//#define USE_TIM4

#ifdef USE_TIM2
	#define TIM_HARD					TIM2
	#define	RCC_TIM_HARD_CLK_ENABLE()	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	#define TIM_HARD_IRQn				TIM2_IRQn
	#define TIM_HARD_IRQHandler			TIM2_IRQHandler
#endif

#ifdef USE_TIM3
	#define TIM_HARD					TIM3
	#define	RCC_TIM_HARD_CLK_ENABLE()	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	#define TIM_HARD_IRQn				TIM3_IRQn
	#define TIM_HARD_IRQHandler			TIM3_IRQHandler
#endif

#ifdef USE_TIM4
	#define TIM_HARD					TIM4
	#define	RCC_TIM_HARD_CLK_ENABLE()	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	#define TIM_HARD_IRQn				TIM4_IRQn
	#define TIM_HARD_IRQHandler			TIM4_IRQHandler
#endif


/* ���� TIM��ʱ�жϵ���ִ�еĻص�����ָ�� */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);

#ifdef TIM_HARD

/**
 * @name   bsp_InitHardTimer
 * @brief  ���� TIMx������us����Ӳ����ʱ��TIMx���������У�����ֹͣ.
		   TIMx������TIM2 - TIM4 ֮���TIM, ��ЩTIM��3��ͨ��, ���� APB1 �ϣ�����ʱ��=SystemCoreClock / 1
 * @param  ��
 * @retval ��
 */
void bsp_InitHardTimer(void)
{
    LL_TIM_InitTypeDef  TimHandle = {0};
	uint32_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;
	TIM_TypeDef* TIMx = TIM_HARD;
	
	RCC_TIM_HARD_CLK_ENABLE();		/* ʹ��TIMʱ�� */
	
    /*-----------------------------------------------------------------------
		system_stm32g4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		��ΪAPB1 prescaler = 1, ���� APB1�ϵ�TIMxCLK = SystemCoreClock;
		��ΪAPB2 prescaler = 1, ���� APB2�ϵ�TIMxCLK = SystemCoreClock;

		APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 ��ʱ���� TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	uiTIMxCLK = SystemCoreClock;

	usPrescaler = uiTIMxCLK / 1000000 - 1;	/* ��Ƶ�� = 1 */
	
	if (TIMx == TIM2)
	{
		usPeriod = 0xFFFFFFFF;
	}
	else
	{
		usPeriod = 0xFFFF;
	}

	/* 
     * ���÷�ƵΪusPrescaler����ô��ʱ����������1�ξ���1us
     * ������usPeriod��ֵ�Ǿ�������������
     * usPeriod = 0xFFFF ��ʾ���0xFFFF΢�롣
     * usPeriod = 0xFFFFFFFF ��ʾ���0xFFFFFFFF΢�롣
     */
	TimHandle.Prescaler				 = usPrescaler;
	TimHandle.CounterMode			 = LL_TIM_COUNTERMODE_UP;
	TimHandle.ClockDivision			 = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.Autoreload 			 = usPeriod; 
	
	LL_TIM_Init(TIM_HARD, &TimHandle);

	/* ���ö�ʱ���жϣ���CC����Ƚ��ж�ʹ�� */
	NVIC_SetPriority(TIM_HARD_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
	NVIC_EnableIRQ(TIM_HARD_IRQn);
    
    /* ������ʱ�� */
	LL_TIM_EnableCounter(TIM_HARD);
}

/**
 * @name   bsp_StartHardTimer
 * @brief  ʹ��TIM2-4�����ζ�ʱ��ʹ��, ��ʱʱ�䵽��ִ�лص�����������ͬʱ����4����ʱ��ͨ�����������š�
		   ��ʱ��������1us ����Ҫ�ķ��ڵ��ñ�������ִ��ʱ�䣩
		   TIM2		  ��32λ��ʱ������ʱ��Χ�ܴ�
		   TIM3��TIM4 ��16λ��ʱ����
 * @param  _CC : ����Ƚ�ͨ������1��2��3, 4
 * @param  _uiTimeOut : ��ʱʱ��, ��λ 1us. ����16λ��ʱ������� 65.5ms; ����32λ��ʱ������� 4294��
 * @param  _pCallBack : ��ʱʱ�䵽�󣬱�ִ�еĺ���
 * @retval ��
 */
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
    uint32_t cnt_now;
    uint32_t cnt_tar;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    /* ���貹���ӳ٣�ʵ�⾫������1us */
    
    cnt_now = TIMx->CNT; 
    cnt_tar = cnt_now + _uiTimeOut;			/* ���㲶��ļ�����ֵ */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

		TIMx->CCR1 = cnt_tar; 			    /* ���ò���Ƚϼ�����CC1 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC1IE;   /* ���CC1�жϱ�־ */
		TIMx->DIER |= LL_TIM_DIER_CC1IE;			/* ʹ��CC1�ж� */
	}
    else if (_CC == 2)
    {
		s_TIM_CallBack2 = (void (*)(void))_pCallBack;

		TIMx->CCR2 = cnt_tar;				/* ���ò���Ƚϼ�����CC2 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC2IE;	/* ���CC2�жϱ�־ */
		TIMx->DIER |= LL_TIM_DIER_CC2IE;			/* ʹ��CC2�ж� */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

		TIMx->CCR3 = cnt_tar;				/* ���ò���Ƚϼ�����CC3 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC3IE;	/* ���CC3�жϱ�־ */
		TIMx->DIER |= LL_TIM_DIER_CC3IE;			/* ʹ��CC3�ж� */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

		TIMx->CCR4 = cnt_tar;				/* ���ò���Ƚϼ�����CC4 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC4IE;	/* ���CC4�жϱ�־ */
		TIMx->DIER |= LL_TIM_DIER_CC4IE;			/* ʹ��CC4�ж� */
    }
	else
    {
        return;
    }
}

/**
 * @name   TIMx_IRQHandler
 * @brief  TIM �жϷ������
 * @param  ��
 * @retval ��
 */
void TIM_HARD_IRQHandler(void)
{
	uint16_t itstatus = 0x0, itenable = 0x0;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    
  	itstatus = TIMx->SR & LL_TIM_DIER_CC1IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC1IE;
    
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC1IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC1IE;		/* ����CC1�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack1();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC2IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC2IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC2IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC2IE;		/* ����CC2�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack2();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC3IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC3IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC3IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC3IE;		/* ����CC2�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack3();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC4IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC4IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC4IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC4IE;		/* ����CC4�ж� */	

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack4();
    }	
}

#endif
