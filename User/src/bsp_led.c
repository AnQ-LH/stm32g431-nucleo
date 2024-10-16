#include "bsp_led.h" 

#define HARD_LED_NUM   	 	1	/* 3个LED */

/* 使能GPIO时钟 */
#define ALL_LED_GPIO_CLK_ENABLE() {	    \
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);	\
	};

/* 依次定义GPIO */
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	uint8_t ActiveLevel;	/* 激活电平 */
}X_GPIO_T;


/* GPIO和PIN定义 */
static const X_GPIO_T s_gpio_list[HARD_LED_NUM] = {
	{GPIOB, LL_GPIO_PIN_8, 1},
};	


/**
 * @brief  bsp_InitLed
 * @param  无
 * @retval 无
 */
void bsp_InitLed(void)
{
	uint8_t i;
	LL_GPIO_InitTypeDef  GPIO_InitStruct;
	
	/* 第1步：打开GPIO时钟 */
	ALL_LED_GPIO_CLK_ENABLE();
	
	/* 第2步：配置所有的按键GPIO为推挽输出模式 */
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_OUTPUT;			/* 设置输出 */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;				/* 设置IO无上下拉 */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;		/* GPIO速度等级 */
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;		/* 推挽输出 */

	for (i = 0; i < HARD_LED_NUM; i++)
	{
		GPIO_InitStruct.Pin		 = s_gpio_list[i].pin;
		LL_GPIO_Init(s_gpio_list[i].gpio, &GPIO_InitStruct);
		bsp_LedOff(i);
	}
	
}

/**
 * @brief  bsp_LedOn
 * @param  _no : 指示灯序号，范围 0 - 2
 * @retval 无
 */
void bsp_LedOn(uint8_t _no)
{
	LL_GPIO_SetOutputPin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}

/**
 * @brief  bsp_LedOff
 * @param  _no : 指示灯序号，范围 0 - 2
 * @retval 无
 */
void bsp_LedOff(uint8_t _no)
{
	LL_GPIO_ResetOutputPin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}

/**
 * @brief  bsp_LedToggle
 * @param  _no : 指示灯序号，范围 1 - 3
 * @retval 无
 */
void bsp_LedToggle(uint8_t _no)
{
	LL_GPIO_TogglePin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}


