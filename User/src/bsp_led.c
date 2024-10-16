#include "bsp_led.h" 

#define HARD_LED_NUM   	 	1	/* 3��LED */

/* ʹ��GPIOʱ�� */
#define ALL_LED_GPIO_CLK_ENABLE() {	    \
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);	\
	};

/* ���ζ���GPIO */
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	uint8_t ActiveLevel;	/* �����ƽ */
}X_GPIO_T;


/* GPIO��PIN���� */
static const X_GPIO_T s_gpio_list[HARD_LED_NUM] = {
	{GPIOB, LL_GPIO_PIN_8, 1},
};	


/**
 * @brief  bsp_InitLed
 * @param  ��
 * @retval ��
 */
void bsp_InitLed(void)
{
	uint8_t i;
	LL_GPIO_InitTypeDef  GPIO_InitStruct;
	
	/* ��1������GPIOʱ�� */
	ALL_LED_GPIO_CLK_ENABLE();
	
	/* ��2�����������еİ���GPIOΪ�������ģʽ */
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_OUTPUT;			/* ������� */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;				/* ����IO�������� */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;		/* GPIO�ٶȵȼ� */
	GPIO_InitStruct.OutputType	 = LL_GPIO_OUTPUT_PUSHPULL;		/* ������� */

	for (i = 0; i < HARD_LED_NUM; i++)
	{
		GPIO_InitStruct.Pin		 = s_gpio_list[i].pin;
		LL_GPIO_Init(s_gpio_list[i].gpio, &GPIO_InitStruct);
		bsp_LedOff(i);
	}
	
}

/**
 * @brief  bsp_LedOn
 * @param  _no : ָʾ����ţ���Χ 0 - 2
 * @retval ��
 */
void bsp_LedOn(uint8_t _no)
{
	LL_GPIO_SetOutputPin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}

/**
 * @brief  bsp_LedOff
 * @param  _no : ָʾ����ţ���Χ 0 - 2
 * @retval ��
 */
void bsp_LedOff(uint8_t _no)
{
	LL_GPIO_ResetOutputPin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}

/**
 * @brief  bsp_LedToggle
 * @param  _no : ָʾ����ţ���Χ 1 - 3
 * @retval ��
 */
void bsp_LedToggle(uint8_t _no)
{
	LL_GPIO_TogglePin(s_gpio_list[_no].gpio, s_gpio_list[_no].pin);
}


