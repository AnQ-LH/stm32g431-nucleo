#include "system.h"

static bool isInit = false;

bool systemTest(void);
static void peripheralDeviceInit(void);
/**
 * @brief 系统初始化
 */
void systemInit(void)
{
	uint8_t cnt = 0;
	bsp_Init();
	peripheralDeviceInit();
	if(systemTest() == true){}
}

bool systemTest(void)
{
	return isInit;
}

/**
 * @brief 硬件初始化
 */
static void peripheralDeviceInit(void)
{
	if(isInit) return;
	
	
	isInit = true;
}
