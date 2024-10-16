#include "bsp.h"


static void SystemClock_Config(void);


/**
 * @name   bsp_Init
 * @brief  初始化所有的硬件设备,该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量,只需要调用一次
 * @param  无
 * @retval 无
 */
void bsp_Init(void)
{
	/*
	 * 配置系统时钟到170MHz
     * - 切换使用HSE。
     * - 此函数会更新全局变量SystemCoreClock，并重新配置HAL_InitTick。
	 */
	SystemClock_Config();

	/* 
	 * Event Recorder：
	 * - 可用于代码执行时间测量，MDK5.25及其以上版本才支持，IAR不支持。
	 * - 默认不开启，如果要使能此选项，务必看V5开发板用户手册第8章
	 */	
#if Enable_EventRecorder == 1  
	/* 初始化EventRecorder并开启 */
	EventRecorderInitialize(EventRecordAll, 1U);
	EventRecorderStart();
#endif
	
	bsp_InitKey();    	/* 按键初始化，要放在滴答定时器之前，因为按钮检测是通过滴答定时器扫描 */
	bsp_InitSystick();  /* 初始化滴答定时器 */
	bsp_InitUart();		/* 初始化串口 */
	bsp_InitLed();    	/* 初始化LED */	
//	bsp_InitI2C();      /* 初始化IIC */
	bsp_InitSPIBus();   /* 初始化硬件SPI */

}

/**
 * @name   SystemClock_Config
 * @brief  初始化系统时钟
 *		   System Clock source            = PLL (HSE)
 *		   SYSCLK(Hz)                     = 170000000	(CPU Clock)
 *		   HCLK = SYSCLK / 1              = 170000000	(AHB1Periph)
 *		   PCLK2 = HCLK / 1               = 170000000	(APB2Periph)
 *		   PCLK1 = HCLK / 1               = 170000000	(APB1Periph)
 *		   MCO Frequency(Hz)              = 8000000
 *		   PLL_M                          = 2
 *		   PLL_N                          = 85
 *		   PLL_R                          = 2
 *		   VDD(V)                         = 3.3
 *		   Flash Latency(WS)              = 4
 * @param  无
 * @retval 无
 */
static void SystemClock_Config(void)
{
	/* Enable voltage range 1 boost mode for frequency above 150 Mhz */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableRange1BoostMode();
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
	/* Set Flash Latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

	/* HSI already enabled at reset */
	/* HSE  enable */
	LL_RCC_HSE_Enable();
	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	while(LL_RCC_PLL_IsReady() != 1)
	{
	}

	/* PLL system Output activation */
	LL_RCC_PLL_EnableDomain_SYS();

	/* Sysclk activation on the main PLL */
	/* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	};

	/* Insure 1μs transition state at intermediate medium speed clock based on DWT */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;
	while(DWT->CYCCNT < 100);

	/* AHB prescaler 1 */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

	/* Set APB1 & APB2 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	/* Set systick to 1ms in using frequency set to 170MHz */
	/* This frequency can be calculated through LL RCC macro */
	/* ex: __LL_RCC_CALC_PLLCLK_FREQ(HSI_VALUE,
								  LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2) */
	LL_Init1msTick(170000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(170000000);
}

/**
 * @name   Error_Handler
 * @param  file : 源代码文件名称,关键字 __FILE__ 表示源代码文件名.
 * @param  line :代码行号,关键字 __LINE__ 表示源代码行号.
 * @retval 无
 */
void Error_Handler(char *file, uint32_t line)
{
	/* 
	 * 用户可以添加自己的代码报告源代码文件名和代码行号，比如将错误文件和行号打印到串口
	 * printf("Wrong parameters value: file %s on line %d\r\n", file, line) 
	 */
	
	/* 这是一个死循环，断言失败时程序会在此处死机，以便于用户查错 */
	if (line == 0)
	{
		return;
	}
	
	while(1)
	{
		
	}
}

/**
 * @name   bsp_RunPer10ms
 * @brief  该函数每隔10ms被Systick中断调用1次。详见 bsp_timer.c的定时中断服务程序。一些处理时间要求不严格的
		   任务可以放在此函数。比如：按键扫描、蜂鸣器鸣叫控制等。
 * @param  无
 * @retval 无
 */
void bsp_RunPer10ms(void)
{
//	bsp_KeyScan10ms();
}

/**
 * @name   bsp_RunPer1ms
 * @brief  该函数每隔1ms被Systick中断调用1次.详见 bsp_timer.c的定时中断服务程序.一些需要周期性处理的事务
		   可以放在此函数.比如：触摸坐标扫描.
 * @param  无
 * @retval 无
 */
void bsp_RunPer1ms(void)
{
	
}

/**
 * @name   bsp_Idle
 * @brief  空闲时执行的函数.一般主程序在for和while循环程序体中需要插入 CPU_IDLE() 宏来调用本函数.
		   可以放在此函数.比如：触摸坐标扫描.
 * @param  无
 * @retval 无
 */
void bsp_Idle(void)
{
	/* --- 喂狗 */

	/* --- 让CPU进入休眠，由Systick定时中断唤醒或者其他中断唤醒 */

	/* 例如 emWin 图形库，可以插入图形库需要的轮询函数 */
	//GUI_Exec();

	/* 例如 uIP 协议，可以插入uip轮询函数 */
	//TOUCH_CapScan();
}

/**
 * @name   HAL_Delay
 * @brief  重定向毫秒延迟函数。替换HAL中的函数。因为HAL中的缺省函数依赖于Systick中断，如果在USB、SD卡
		   中断中有延迟函数，则会锁死.也可以通过函数HAL_NVIC_SetPriority提升Systick中断
 * @param  无
 * @retval 无
 */
/* 当前例子使用stm32f4xx_hal.c默认方式实现，未使用下面重定向的函数 */
#if 0
void HAL_Delay(uint32_t Delay)
{
	bsp_DelayUS(Delay * 1000);
}
#endif
