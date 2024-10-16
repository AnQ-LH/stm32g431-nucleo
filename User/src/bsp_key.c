#include "bsp_key.h"


/**
 * Key ��	: PC13   (�ߵ�ƽ��ʾ����)
 */

#define HARD_KEY_NUM	    1	   						/* ʵ�尴������ */
#define KEY_COUNT   	 	( HARD_KEY_NUM )	/* 1�������� + 0����ϰ��� */

/* ʹ��GPIOʱ�� */
#define ALL_KEY_GPIO_CLK_ENABLE() {	    \
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
static const X_GPIO_T s_gpio_list[HARD_KEY_NUM] = {
	{GPIOC, LL_GPIO_PIN_8, 1},		/* Key */
};	

/**  
 * ����һ���꺯���򻯺������� 
 * �ж�GPIO�����Ƿ���Ч����
 */
static KEY_T s_tBtn[KEY_COUNT] = { 0 };
static KEY_FIFO_T s_tKey;		/* ����FIFO����,�ṹ�� */

static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);

#define KEY_PIN_ACTIVE(id)	

/**
 * @name   KeyPinActive
 * @brief  �жϰ����Ƿ���
 * @param  ��
 * @retval ����ֵ1 ��ʾ����(��ͨ����0��ʾδ���£��ͷţ�
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
 * @brief  �жϰ����Ƿ��¡���������ϼ����֡������¼������������������¡�
 * @param  ��
 * @retval ����ֵ1 ��ʾ����(��ͨ����0��ʾδ���£��ͷţ�
 */
static uint8_t IsKeyDownFunc(uint8_t _id)
{
	/* ʵ�嵥�� */
	if (_id < HARD_KEY_NUM)
	{
		uint8_t i;
		uint8_t count = 0;
		uint8_t save = 255;
		
		/* �ж��м��������� */
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
			return 1;	/* ֻ��1��������ʱ����Ч */
		}		

		return 0;
	}

	return 0;
}

/**
 * @name   bsp_InitKey
 * @brief  ��ʼ������. �ú����� bsp_Init() ����.
 * @param  ��
 * @retval ��
 */
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* ��ʼ���������� */
	bsp_InitKeyHard();		/* ��ʼ������Ӳ�� */
}

/**
 * @name   bsp_InitKeyHard
 * @brief  ���ð�����Ӧ��GPIO
 * @param  ��
 * @retval ��
 */
static void bsp_InitKeyHard(void)
{	
	uint8_t i;
	LL_GPIO_InitTypeDef  GPIO_InitStruct;
	
	/* ��1������GPIOʱ�� */
	ALL_KEY_GPIO_CLK_ENABLE();

	/* ��2�����������еİ���GPIOΪ�������ģʽ */
	GPIO_InitStruct.Mode		 = LL_GPIO_MODE_INPUT;			/* ������� */
	GPIO_InitStruct.Pull		 = LL_GPIO_PULL_NO;				/* ����IO�������� */
	GPIO_InitStruct.Speed		 = LL_GPIO_SPEED_FREQ_HIGH;		/* GPIO�ٶȵȼ� */
	
	/* ��2�����������еİ���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	for (i = 0; i < HARD_KEY_NUM; i++)
	{
		GPIO_InitStruct.Pin		 = s_gpio_list[i].pin;
		LL_GPIO_Init(s_gpio_list[i].gpio, &GPIO_InitStruct);
	}
}

/**
 * @name   bsp_InitKeyVar
 * @brief  ��ʼ����������
 * @param  ��
 * @retval ��
 */
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;				/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;			/* ����������Ϊ�˲�ʱ���һ�� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
		s_tBtn[i].RepeatSpeed = 0;						/* �����������ٶȣ�0��ʾ��֧������ */
		s_tBtn[i].RepeatCount = 0;						/* ���������� */
	}

	/* �����Ҫ��������ĳ�������Ĳ����������ڴ˵������¸�ֵ */
	
//	bsp_SetKeyParam(KID_JOY_U, 100, 6);
}

/**
 * @name   bsp_PutKey
 * @brief  ��1����ֵѹ�밴��FIFO������,������ģ��һ������.
 * @param  KeyCode : ��������
 * @retval ��
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
 * @brief  �Ӱ���FIFO��������ȡһ����ֵ.
 * @param  ��
 * @retval ��������
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
 * @brief  �Ӱ���FIFO��������ȡһ����ֵ,�����Ķ�ָ��.
 * @param  ��
 * @retval ��������
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
 * @brief  ��ȡ������״̬
 * @param  _ucKeyID : ����ID����0��ʼ
 * @retval 1 ��ʾ���£� 0 ��ʾδ����
 */
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/**
 * @name   bsp_SetKeyParam
 * @brief  ���ð�������
 * @param  _ucKeyID 		����ID����0��ʼ
 * @param  _LongTime 		�����¼�ʱ��
 * @param  _RepeatSpeed	�����ٶ�
 * @retval 1 ��ʾ���£� 0 ��ʾδ����
 */
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
	s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;	/* �����������ٶȣ�0��ʾ��֧������ */
	s_tBtn[_ucKeyID].RepeatCount = 0;				/* ���������� */
}

/**
 * @name   bsp_ClearKey
 * @brief  ��հ���FIFO������
 * @param  ��
 * @retval ��������
 */
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/**
 * @name   bsp_DetectKey
 * @brief  ���һ������,������״̬,���뱻�����Եĵ���.
 * @param  IO��id�� ��0��ʼ����
 * @retval ��
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

				/* ���Ͱ�ť���µ���Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 1));
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* ���Ͱ�ť�������µ���Ϣ */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* ��ֵ���밴��FIFO */
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
							/* ��������ÿ��10ms����1������ */
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

				/* ���Ͱ�ť�������Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/**
 * @name   bsp_DetectFastIO
 * @brief  �����ٵ�����IO,1msˢ��һ��.
 * @param  IO��id�� ��0��ʼ����
 * @retval ��
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

			/* ���Ͱ�ť���µ���Ϣ */
			bsp_PutKey((uint8_t)(3 * i + 1));
		}

		if (pBtn->LongTime > 0)
		{
			if (pBtn->LongCount < pBtn->LongTime)
			{
				/* ���Ͱ�ť�������µ���Ϣ */
				if (++pBtn->LongCount == pBtn->LongTime)
				{
					/* ��ֵ���밴��FIFO */
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
						/* ��������ÿ��10ms����1������ */
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

			/* ���Ͱ�ť�������Ϣ */
			bsp_PutKey((uint8_t)(3 * i + 2));
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/**
 * @name   bsp_KeyScan10ms
 * @brief  ɨ�����а�����������,��systick�ж������Եĵ���,10msһ��
 * @param  ��
 * @retval ��
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
 * @brief  ɨ�����а���������������systick�ж������Եĵ��ã�1msһ��.
 * @param  ��
 * @retval ��
 */
void bsp_KeyScan1ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectFastIO(i);
	}
}
