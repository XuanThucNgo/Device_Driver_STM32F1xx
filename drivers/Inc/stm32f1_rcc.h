/*
 * stm32f1_rcc.h
 *
 *  Created on: Aug 16, 2024
 *      Author: LENOVO
 */

#ifndef INC_STM32F1_RCC_H_
#define INC_STM32F1_RCC_H_

#include <stm32f1.h>

//This return APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This return APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F1_RCC_H_ */
