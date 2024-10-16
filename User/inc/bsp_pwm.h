#ifndef __BSP_TIM_PWM_H
#define __BSP_TIM_PWM_H

#include "main.h"

/* 提供给其他C文件调用的函数 */
void bsp_SetTIMOutPWM(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx, uint8_t _ucChannel, uint32_t _ulFreq, uint32_t _ulDutyCycle);
void bsp_SetTIMOutPWM_N(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_TypeDef* TIMx, uint8_t _ucChannel, uint32_t _ulFreq, uint32_t _ulDutyCycle);	 
void bsp_SetTIMforInt(TIM_TypeDef* TIMx, uint32_t _ulFreq, uint8_t _PreemptionPriority, uint8_t _SubPriority);

#endif  /* __BSP_TIM_PWM_H */
