/*
*********************************************************************************************************
*
*	模块名称 : BSP模块(For STM32F429)
*	文件名称 : bsp.h
*	版    本 : V1.0
*	说    明 : 这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
*			   bsp = Borad surport packet 板级支持包
*	修改记录 :
*		版本号  日期         作者       说明
*		V1.0    2018-07-29  Eric2013   正式发布
*
*	Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#ifndef __BSP_H
#define __BSP_H

#define STM32G431NUCLEO    


/* 检查是否定义了开发板型号 */
#if !defined (STM32G431NUCLEO)
	#error "Please define the board model : STM32G431NUCLEO"
#endif

/* 定义 BSP 版本号 */
#define __STM32G431_BSP_VERSION		"1.0"

/* CPU空闲时执行的函数 */
//#define CPU_IDLE()		bsp_Idle()

/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

/* 这个宏仅用于调试阶段排错 */
#define BSP_Printf		printf

#define ERROR_HANDLER()		Error_Handler(__FILE__, __LINE__);

/* 默认是关闭状态 */
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

/* 定义优先级分组 */
#define NVIC_PREEMPT_PRIORITY	4

/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_SysTick.h"
#include "bsp_timer.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_i2c.h"

/* 提供给其他C文件调用的函数 */
void bsp_Init(void);
void bsp_Idle(void);
void bsp_GetCpuID(uint32_t *_id);
void Error_Handler(char *file, uint32_t line);

#endif  /* __BSP_H */
