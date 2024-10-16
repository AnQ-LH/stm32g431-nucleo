#include "bsp_timer.h"
#include "bsp.h"
/*
 * 定义用于硬件定时器的TIM， 可以使 TIM2 - TIM5
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


/* 保存 TIM定时中断到后执行的回调函数指针 */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);

#ifdef TIM_HARD

/**
 * @name   bsp_InitHardTimer
 * @brief  配置 TIMx，用于us级别硬件定时。TIMx将自由运行，永不停止.
		   TIMx可以用TIM2 - TIM4 之间的TIM, 这些TIM有3个通道, 挂在 APB1 上，输入时钟=SystemCoreClock / 1
 * @param  无
 * @retval 无
 */
void bsp_InitHardTimer(void)
{
    LL_TIM_InitTypeDef  TimHandle = {0};
	uint32_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;
	TIM_TypeDef* TIMx = TIM_HARD;
	
	RCC_TIM_HARD_CLK_ENABLE();		/* 使能TIM时钟 */
	
    /*-----------------------------------------------------------------------
		system_stm32g4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 1      (APB2Periph)
		PCLK1 = HCLK / 1      (APB1Periph)

		因为APB1 prescaler = 1, 所以 APB1上的TIMxCLK = SystemCoreClock;
		因为APB2 prescaler = 1, 所以 APB2上的TIMxCLK = SystemCoreClock;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM6, TIM7
		APB2 定时器有 TIM1, TIM8 ,TIM15, TIM16, TIM17

	----------------------------------------------------------------------- */
	uiTIMxCLK = SystemCoreClock;

	usPrescaler = uiTIMxCLK / 1000000 - 1;	/* 分频比 = 1 */
	
	if (TIMx == TIM2)
	{
		usPeriod = 0xFFFFFFFF;
	}
	else
	{
		usPeriod = 0xFFFF;
	}

	/* 
     * 设置分频为usPrescaler后，那么定时器计数器计1次就是1us
     * 而参数usPeriod的值是决定了最大计数：
     * usPeriod = 0xFFFF 表示最大0xFFFF微秒。
     * usPeriod = 0xFFFFFFFF 表示最大0xFFFFFFFF微秒。
     */
	TimHandle.Prescaler				 = usPrescaler;
	TimHandle.CounterMode			 = LL_TIM_COUNTERMODE_UP;
	TimHandle.ClockDivision			 = LL_TIM_CLOCKDIVISION_DIV1;
	TimHandle.Autoreload 			 = usPeriod; 
	
	LL_TIM_Init(TIM_HARD, &TimHandle);

	/* 配置定时器中断，给CC捕获比较中断使用 */
	NVIC_SetPriority(TIM_HARD_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
	NVIC_EnableIRQ(TIM_HARD_IRQn);
    
    /* 启动定时器 */
	LL_TIM_EnableCounter(TIM_HARD);
}

/**
 * @name   bsp_StartHardTimer
 * @brief  使用TIM2-4做单次定时器使用, 定时时间到后执行回调函数。可以同时启动4个定时器通道，互不干扰。
		   定时精度正负1us （主要耗费在调用本函数的执行时间）
		   TIM2		  是32位定时器。定时范围很大
		   TIM3和TIM4 是16位定时器。
 * @param  _CC : 捕获比较通道几，1，2，3, 4
 * @param  _uiTimeOut : 超时时间, 单位 1us. 对于16位定时器，最大 65.5ms; 对于32位定时器，最大 4294秒
 * @param  _pCallBack : 定时时间到后，被执行的函数
 * @retval 无
 */
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
    uint32_t cnt_now;
    uint32_t cnt_tar;
	TIM_TypeDef* TIMx = TIM_HARD;
	
    /* 无需补偿延迟，实测精度正负1us */
    
    cnt_now = TIMx->CNT; 
    cnt_tar = cnt_now + _uiTimeOut;			/* 计算捕获的计数器值 */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

		TIMx->CCR1 = cnt_tar; 			    /* 设置捕获比较计数器CC1 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC1IE;   /* 清除CC1中断标志 */
		TIMx->DIER |= LL_TIM_DIER_CC1IE;			/* 使能CC1中断 */
	}
    else if (_CC == 2)
    {
		s_TIM_CallBack2 = (void (*)(void))_pCallBack;

		TIMx->CCR2 = cnt_tar;				/* 设置捕获比较计数器CC2 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC2IE;	/* 清除CC2中断标志 */
		TIMx->DIER |= LL_TIM_DIER_CC2IE;			/* 使能CC2中断 */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

		TIMx->CCR3 = cnt_tar;				/* 设置捕获比较计数器CC3 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC3IE;	/* 清除CC3中断标志 */
		TIMx->DIER |= LL_TIM_DIER_CC3IE;			/* 使能CC3中断 */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

		TIMx->CCR4 = cnt_tar;				/* 设置捕获比较计数器CC4 */
        TIMx->SR = (uint16_t)~LL_TIM_DIER_CC4IE;	/* 清除CC4中断标志 */
		TIMx->DIER |= LL_TIM_DIER_CC4IE;			/* 使能CC4中断 */
    }
	else
    {
        return;
    }
}

/**
 * @name   TIMx_IRQHandler
 * @brief  TIM 中断服务程序
 * @param  无
 * @retval 无
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
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC1IE;		/* 禁能CC1中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack1();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC2IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC2IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC2IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC2IE;		/* 禁能CC2中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack2();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC3IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC3IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC3IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC3IE;		/* 禁能CC2中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack3();
    }

	itstatus = TIMx->SR & LL_TIM_DIER_CC4IE;
	itenable = TIMx->DIER & LL_TIM_DIER_CC4IE;
	if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
	{
		TIMx->SR = (uint16_t)~LL_TIM_DIER_CC4IE;
		TIMx->DIER &= (uint16_t)~LL_TIM_DIER_CC4IE;		/* 禁能CC4中断 */	

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack4();
    }	
}

#endif
