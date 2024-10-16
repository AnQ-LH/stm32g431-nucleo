#include "bsp.h"


static void SystemClock_Config(void);


/**
 * @name   bsp_Init
 * @brief  ��ʼ�����е�Ӳ���豸,�ú�������CPU�Ĵ���������ļĴ�������ʼ��һЩȫ�ֱ���,ֻ��Ҫ����һ��
 * @param  ��
 * @retval ��
 */
void bsp_Init(void)
{
	/*
	 * ����ϵͳʱ�ӵ�170MHz
     * - �л�ʹ��HSE��
     * - �˺��������ȫ�ֱ���SystemCoreClock������������HAL_InitTick��
	 */
	SystemClock_Config();

	/* 
	 * Event Recorder��
	 * - �����ڴ���ִ��ʱ�������MDK5.25�������ϰ汾��֧�֣�IAR��֧�֡�
	 * - Ĭ�ϲ����������Ҫʹ�ܴ�ѡ���ؿ�V5�������û��ֲ��8��
	 */	
#if Enable_EventRecorder == 1  
	/* ��ʼ��EventRecorder������ */
	EventRecorderInitialize(EventRecordAll, 1U);
	EventRecorderStart();
#endif
	
	bsp_InitKey();    	/* ������ʼ����Ҫ���ڵδ�ʱ��֮ǰ����Ϊ��ť�����ͨ���δ�ʱ��ɨ�� */
	bsp_InitSystick();  /* ��ʼ���δ�ʱ�� */
	bsp_InitUart();		/* ��ʼ������ */
	bsp_InitLed();    	/* ��ʼ��LED */	
//	bsp_InitI2C();      /* ��ʼ��IIC */
	bsp_InitSPIBus();   /* ��ʼ��Ӳ��SPI */

}

/**
 * @name   SystemClock_Config
 * @brief  ��ʼ��ϵͳʱ��
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
 * @param  ��
 * @retval ��
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

	/* Insure 1��s transition state at intermediate medium speed clock based on DWT */
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
 * @param  file : Դ�����ļ�����,�ؼ��� __FILE__ ��ʾԴ�����ļ���.
 * @param  line :�����к�,�ؼ��� __LINE__ ��ʾԴ�����к�.
 * @retval ��
 */
void Error_Handler(char *file, uint32_t line)
{
	/* 
	 * �û���������Լ��Ĵ��뱨��Դ�����ļ����ʹ����кţ����罫�����ļ����кŴ�ӡ������
	 * printf("Wrong parameters value: file %s on line %d\r\n", file, line) 
	 */
	
	/* ����һ����ѭ��������ʧ��ʱ������ڴ˴��������Ա����û���� */
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
 * @brief  �ú���ÿ��10ms��Systick�жϵ���1�Ρ���� bsp_timer.c�Ķ�ʱ�жϷ������һЩ����ʱ��Ҫ���ϸ��
		   ������Է��ڴ˺��������磺����ɨ�衢���������п��Ƶȡ�
 * @param  ��
 * @retval ��
 */
void bsp_RunPer10ms(void)
{
//	bsp_KeyScan10ms();
}

/**
 * @name   bsp_RunPer1ms
 * @brief  �ú���ÿ��1ms��Systick�жϵ���1��.��� bsp_timer.c�Ķ�ʱ�жϷ������.һЩ��Ҫ�����Դ��������
		   ���Է��ڴ˺���.���磺��������ɨ��.
 * @param  ��
 * @retval ��
 */
void bsp_RunPer1ms(void)
{
	
}

/**
 * @name   bsp_Idle
 * @brief  ����ʱִ�еĺ���.һ����������for��whileѭ������������Ҫ���� CPU_IDLE() �������ñ�����.
		   ���Է��ڴ˺���.���磺��������ɨ��.
 * @param  ��
 * @retval ��
 */
void bsp_Idle(void)
{
	/* --- ι�� */

	/* --- ��CPU�������ߣ���Systick��ʱ�жϻ��ѻ��������жϻ��� */

	/* ���� emWin ͼ�ο⣬���Բ���ͼ�ο���Ҫ����ѯ���� */
	//GUI_Exec();

	/* ���� uIP Э�飬���Բ���uip��ѯ���� */
	//TOUCH_CapScan();
}

/**
 * @name   HAL_Delay
 * @brief  �ض�������ӳٺ������滻HAL�еĺ�������ΪHAL�е�ȱʡ����������Systick�жϣ������USB��SD��
		   �ж������ӳٺ������������.Ҳ����ͨ������HAL_NVIC_SetPriority����Systick�ж�
 * @param  ��
 * @retval ��
 */
/* ��ǰ����ʹ��stm32f4xx_hal.cĬ�Ϸ�ʽʵ�֣�δʹ�������ض���ĺ��� */
#if 0
void HAL_Delay(uint32_t Delay)
{
	bsp_DelayUS(Delay * 1000);
}
#endif
