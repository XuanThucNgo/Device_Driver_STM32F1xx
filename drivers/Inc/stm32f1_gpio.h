/*
 * stm32f1_gpio.h
 *
 *  Created on: Aug 2, 2024
 *      Author: ADMIN
 */
#include <stdint.h>
#include <stm32f1.h>
#ifndef INC_STM32F1_GPIO_H_
#define INC_STM32F1_GPIO_H_

typedef struct
{
    uint8_t GPIO_PinNumber;             //0 -> 15
    uint8_t GPIO_PinMode;               //INPUT, OUTPUT hoac ANALOG
    // uint8_t GPIO_PinSpeed;           //2MHz, 10MHz hoac 50MHz
    uint8_t GPIO_PinPuPdControl;     //Pull up, Pull down hoac No PuPd
    // uint8_t GPIO_PinOPType;          //Push-pull hoac Open-drain
    uint8_t GPIO_PinAltFuncMode;     //Alternate Function Input hoac Output
} GPIO_PinConfig_t;


typedef struct
{
    GPIO_RegDef_t* pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15

typedef enum{
    GPIO_MODE_INPUT_ANALOG          = (0 << 2) | 0,
    GPIO_MODE_INPUT_FLOAT           = (1 << 2) | 0,
    GPIO_MODE_INPUT_PULL            = (2 << 2) | 0,

    GPIO_MODE_OUTPUT_PP_10MHz       = (0 << 2) | 1,
    GPIO_MODE_OUTPUT_OD_10MHz       = (1 << 2) | 1,
    GPIO_MODE_AF_OUTPUT_PP_10MHz    = (2 << 2) | 1,
    GPIO_MODE_AF_OUTPUT_OD_10MHz    = (3 << 2) | 1, 
    
    GPIO_MODE_OUTPUT_PP_2MHz        = (0 << 2) | 2,
    GPIO_MODE_OUTPUT_OD_2MHz        = (1 << 2) | 2,
    GPIO_MODE_AF_OUTPUT_PP_2MHz     = (2 << 2) | 2,
    GPIO_MODE_AF_OUTPUT_OD_2MHz     = (3 << 2) | 2, 

    GPIO_MODE_OUTPUT_PP_50MHz       = (0 << 2) | 3,
    GPIO_MODE_OUTPUT_OD_50MHz       = (1 << 2) | 3,
    GPIO_MODE_AF_OUTPUT_PP_50MHz    = (2 << 2) | 3,
    GPIO_MODE_AF_OUTPUT_OD_50MHz    = (3 << 2) | 3, 

    GPIO_MODE_IT_FT                 = 16,
    GPIO_MODE_IT_RT                 = 17,
    GPIO_MODE_IT_RFT                = 18

} GPIO_MODE;

typedef enum{
    GPIO_PIN_NO_PUPD = 0x00,
    GPIO_PIN_PU      = 0x01,
    GPIO_PIN_PD      = 0x02,
} GPIO_PUPDConfig;



void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOx);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_ConfigMode(GPIO_Handle_t *pGPIOHandle);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);


#endif /* INC_STM32F1_GPIO_H_ */
