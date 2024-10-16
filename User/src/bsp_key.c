#include "bsp_key.h"


/**
 * Key 键	: PC13   (高电平表示按下)
 */

#define HARD_KEY_NUM	    1	   						/* 实体按键个数 */
#define KEY_COUNT   	 	( HARD_KEY_NUM )	/* 1个独立建 + 0个组合按键 */

/* 使能GPIO时钟 */
#define ALL_KEY_GPIO_CLK_ENABLE() {	    \
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
static const X_GPIO_T s_gpio_list[HARD_KEY_NUM] = {
	{GPIOC, LL_GPIO_PIN_8, 1},		/* Key */
};	

/**  
 * 定义一个宏函数简化后续代码 
 * 判断GPIO引脚是否有效按下
 */
static KEY_T s_tBtn[KEY_COUNT] = { 0 };
static KEY_FIFO_T s_tKey;		/* 按键FIFO变量,结构体 */

static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);

#define KEY_PIN_ACTIVE(id)	

/**
 * @name   KeyPinActive
 * @brief  判断按键是否按下
 * @param  无
 * @retval 返回值1 表示按下(导通），0表示未按下（释放）
 */
static uint8_t KeyPinActive(uint8_t _id)
{
	uint8_t level;
	
	if ( ( s_gpio_list[_id].gpio->IDR & s_gpio_list[_id].pin ) == 0 )
	{
		level = 0;
	}
	else
	{
		level = 1;
	}

	if (level == s_gpio_list[_id].ActiveLevel)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @name   IsKeyDownFunc
 * @brief  判断按键是否按下。单键和组合键区分。单键事件不允许有其他键按下。
 * @param  无
 * @retval 返回值1 表示按下(导通），0表示未按下（释放）
 */
static uint8_t IsKeyDownFunc(uint8_t _id)
{
	/* 实体单键 */
	if (_id < HARD_KEY_NUM)
	{
		uint8_t i;
		uint8_t count = 0;
		uint8_t save = 255;
		
		/* 判断有几个键按下 */
		for (i = 0; i < HARD_KEY_NUM; i++)
		{
			if (KeyPinActive(i)) 
			{
				count++;
				save = i;
			}
		}
		
		if (count == 1 && save == _id)
		{
			return 1;	/* 只有1个键按下时才有效 */
		}		

		return 0;
	}

	return 0;
}

/**
 * @name   bsp_InitKey
 * @brief  初始化按键. 该函数被 bsp_Init() 调用.
 * @param  无
 * @retval 无
 */
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* 初始化按键变量 */
	bsp_InitKeyHard();		/* 初始化按键硬件 */
}

/**
 * @name   bsp_InitKeyHard
 * @brief  配置按键对应的GPIO
 * @param  无
 * @retval 无
 */
static void bsp_InitKeyHard(void)
{	
	uint8_t i;
	LL_GPIO_InitTypeDef  GPIO_InitStruct;
	
	/* 第1步：打开GPIO时钟 */
	ALL_KEY_GPIO_CLK_ENABLE();

	/* 第2步：配置所有的按键GPIO为推挽输出模式 */
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_INPUT;			/* 设置输出 */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;				/* 设置IO无上下拉 */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;		/* GPIO速度等级 */
	
	/* 第2步：配置所有的按键GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	for (i = 0; i < HARD_KEY_NUM; i++)
	{
		GPIO_InitStruct.Pin		 = s_gpio_list[i].pin;
		LL_GPIO_Init(s_gpio_list[i].gpio, &GPIO_InitStruct);
	}
}

/**
 * @name   bsp_InitKeyVar
 * @brief  初始化按键变量
 * @param  无
 * @retval 无
 */
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* 对按键FIFO读写指针清零 */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* 给每个按键结构体成员变量赋一组缺省值 */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;				/* 长按时间 0 表示不检测长按键事件 */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;			/* 计数器设置为滤波时间的一半 */
		s_tBtn[i].State = 0;							/* 按键缺省状态，0为未按下 */
		s_tBtn[i].RepeatSpeed = 0;						/* 按键连发的速度，0表示不支持连发 */
		s_tBtn[i].RepeatCount = 0;						/* 连发计数器 */
	}

	/* 如果需要单独更改某个按键的参数，可以在此单独重新赋值 */
	
//	bsp_SetKeyParam(KID_JOY_U, 100, 6);
}

/**
 * @name   bsp_PutKey
 * @brief  将1个键值压入按键FIFO缓冲区,可用于模拟一个按键.
 * @param  KeyCode : 按键代码
 * @retval 无
 */
void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

/**
 * @name   bsp_GetKey
 * @brief  从按键FIFO缓冲区读取一个键值.
 * @param  无
 * @retval 按键代码
 */
uint8_t bsp_GetKey(void)
{
	uint8_t ret;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/**
 * @name   bsp_GetKey2
 * @brief  从按键FIFO缓冲区读取一个键值,独立的读指针.
 * @param  无
 * @retval 按键代码
 */
uint8_t bsp_GetKey2(void)
{
	uint8_t ret;

	if (s_tKey.Read2 == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read2];

		if (++s_tKey.Read2 >= KEY_FIFO_SIZE)
		{
			s_tKey.Read2 = 0;
		}
		return ret;
	}
}

/**
 * @name   bsp_GetKeyState
 * @brief  读取按键的状态
 * @param  _ucKeyID : 按键ID，从0开始
 * @retval 1 表示按下， 0 表示未按下
 */
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/**
 * @name   bsp_SetKeyParam
 * @brief  设置按键参数
 * @param  _ucKeyID 		按键ID，从0开始
 * @param  _LongTime 		长按事件时间
 * @param  _RepeatSpeed	连发速度
 * @retval 1 表示按下， 0 表示未按下
 */
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* 长按时间 0 表示不检测长按键事件 */
	s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;	/* 按键连发的速度，0表示不支持连发 */
	s_tBtn[_ucKeyID].RepeatCount = 0;				/* 连发计数器 */
}

/**
 * @name   bsp_ClearKey
 * @brief  清空按键FIFO缓冲区
 * @param  无
 * @retval 按键代码
 */
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/**
 * @name   bsp_DetectKey
 * @brief  检测一个按键,非阻塞状态,必须被周期性的调用.
 * @param  IO的id， 从0开始编码
 * @retval 无
 */
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	pBtn = &s_tBtn[i];
	if (IsKeyDownFunc(i))
	{
		if (pBtn->Count < KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count < 2 * KEY_FILTER_TIME)
		{
			pBtn->Count++;
		}
		else
		{
			if (pBtn->State == 0)
			{
				pBtn->State = 1;

				/* 发送按钮按下的消息 */
				bsp_PutKey((uint8_t)(3 * i + 1));
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* 发送按钮持续按下的消息 */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* 键值放入按键FIFO */
						bsp_PutKey((uint8_t)(3 * i + 3));
					}
				}
				else
				{
					if (pBtn->RepeatSpeed > 0)
					{
						if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
						{
							pBtn->RepeatCount = 0;
							/* 常按键后，每隔10ms发送1个按键 */
							bsp_PutKey((uint8_t)(3 * i + 1));
						}
					}
				}
			}
		}
	}
	else
	{
		if(pBtn->Count > KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count != 0)
		{
			pBtn->Count--;
		}
		else
		{
			if (pBtn->State == 1)
			{
				pBtn->State = 0;

				/* 发送按钮弹起的消息 */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/**
 * @name   bsp_DetectFastIO
 * @brief  检测高速的输入IO,1ms刷新一次.
 * @param  IO的id， 从0开始编码
 * @retval 无
 */
static void bsp_DetectFastIO(uint8_t i)
{
	KEY_T *pBtn;

	pBtn = &s_tBtn[i];
	if (IsKeyDownFunc(i))
	{
		if (pBtn->State == 0)
		{
			pBtn->State = 1;

			/* 发送按钮按下的消息 */
			bsp_PutKey((uint8_t)(3 * i + 1));
		}

		if (pBtn->LongTime > 0)
		{
			if (pBtn->LongCount < pBtn->LongTime)
			{
				/* 发送按钮持续按下的消息 */
				if (++pBtn->LongCount == pBtn->LongTime)
				{
					/* 键值放入按键FIFO */
					bsp_PutKey((uint8_t)(3 * i + 3));
				}
			}
			else
			{
				if (pBtn->RepeatSpeed > 0)
				{
					if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
					{
						pBtn->RepeatCount = 0;
						/* 常按键后，每隔10ms发送1个按键 */
						bsp_PutKey((uint8_t)(3 * i + 1));
					}
				}
			}
		}
	}
	else
	{
		if (pBtn->State == 1)
		{
			pBtn->State = 0;

			/* 发送按钮弹起的消息 */
			bsp_PutKey((uint8_t)(3 * i + 2));
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/**
 * @name   bsp_KeyScan10ms
 * @brief  扫描所有按键。非阻塞,被systick中断周期性的调用,10ms一次
 * @param  无
 * @retval 无
 */
void bsp_KeyScan10ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectKey(i);
	}
}

/**
 * @name   bsp_KeyScan1ms
 * @brief  扫描所有按键。非阻塞，被systick中断周期性的调用，1ms一次.
 * @param  无
 * @retval 无
 */
void bsp_KeyScan1ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectFastIO(i);
	}
}
