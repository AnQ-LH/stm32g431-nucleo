/**
 * @notice Use the MCO as the HES_VALUE (8Mhz)
*/
#include "main.h"

static void PrintfHelp(void);
static void PrintfLogo(void);

/**
 * @name   main
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{	
	bsp_Init();

	bsp_StartAutoTimer(0, 1000);	/* 启动1个100ms的自动重装的定时器 */
	
	PrintfLogo();
	
	/* 进入主程序循环体 */
	while(1)
	{
		bsp_Idle();		/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */

		/* 判断定时器超时时间 */
		if (bsp_CheckTimer(0))	
		{
			/* 每隔50ms 进来一次 */  
			bsp_LedToggle(yellowLed);
		}
		
	}
	return 0;
}



/**
 * @name   PrintfHelp
 * @brief  打印操作提示
 * @param  无
 * @retval 无
 */
static void PrintfHelp(void)
{

}

/**
 * @name   PrintfLogo
 * @brief  打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
 * @param  无
 * @retval 无
 */
static void PrintfLogo(void)
{
	printf("\n\r*************************************************************\r\n");
	printf("CPU: STM32G431KBT6, LQFP48\r\n");
	printf("Board: STM32G431NUCLEO\r\n");
	printf("Frequency:%dMHz, RAM:32KB,	FLASH:128KB\r\n", (int)SystemCoreClock / 1000000);
	printf("\n\r*************************************************************\r\n");
}