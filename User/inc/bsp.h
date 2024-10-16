/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��(For STM32F429)
*	�ļ����� : bsp.h
*	��    �� : V1.0
*	˵    �� : ����Ӳ���ײ�������������ļ���ÿ��c�ļ����� #include "bsp.h" ���������е���������ģ�顣
*			   bsp = Borad surport packet �弶֧�ְ�
*	�޸ļ�¼ :
*		�汾��  ����         ����       ˵��
*		V1.0    2018-07-29  Eric2013   ��ʽ����
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#ifndef __BSP_H
#define __BSP_H

#define STM32G431NUCLEO    


/* ����Ƿ����˿������ͺ� */
#if !defined (STM32G431NUCLEO)
	#error "Please define the board model : STM32G431NUCLEO"
#endif

/* ���� BSP �汾�� */
#define __STM32G431_BSP_VERSION		"1.0"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf

#define ERROR_HANDLER()		Error_Handler(__FILE__, __LINE__);

/* Ĭ���ǹر�״̬ */
#define  Enable_EventRecorder  0

#if Enable_EventRecorder == 1
	#include "EventRecorder.h"
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* �������ȼ����� */
#define NVIC_PREEMPT_PRIORITY	4

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_SysTick.h"
#include "bsp_timer.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_i2c.h"

/* �ṩ������C�ļ����õĺ��� */
void bsp_Init(void);
void bsp_Idle(void);
void bsp_GetCpuID(uint32_t *_id);
void Error_Handler(char *file, uint32_t line);

#endif  /* __BSP_H */
