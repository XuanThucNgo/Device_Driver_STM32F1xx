/*
 * stm32f1_gpio.c
 *
 *  Created on: Aug 2, 2024
 *      Author: ADMIN
 */
#include <stm32f1_gpio.h>

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
    }

    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
    }
}

void GPIO_ConfigMode(GPIO_Handle_t *pGPIOHandle) {
//    uint8_t temp1, temp2;
//    temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
//    temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
//
//    pGPIOHandle->pGPIOx->CR[temp1] &= ~(0x4 << (4 * temp2));                                     // reset
//    pGPIOHandle->pGPIOx->CR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * temp2)); // set
    
    uint32_t reset = 0, set = 0;
    uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    uint32_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    if (pinNumber <= 7) {
        // Configure CRL (Pins 0-7)
        uint8_t position = pinNumber * 4;
        reset = 0xF << position;
        set = mode << position;
        pGPIOHandle->pGPIOx->CR[0] &= ~reset;
        pGPIOHandle->pGPIOx->CR[0] |= set;
    } else if (7 < pinNumber && pinNumber <= 15) {
        // Configure CRH (Pins 8-15)
        uint8_t position = (pinNumber - 8) * 4;
        reset = 0xF << position;
        set = mode << position;
        pGPIOHandle->pGPIOx->CR[1] &= ~reset;
        pGPIOHandle->pGPIOx->CR[1] |= set;
    }
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    //enable peripheral clock

    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    //1. configure mode of pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode < GPIO_MODE_IT_FT )
    {
        GPIO_ConfigMode(pGPIOHandle);
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode)
        {
            if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode ==  GPIO_MODE_IT_FT)
            {
                EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            }
            else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode == GPIO_MODE_IT_RT)
            {
                EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            }
            else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode == GPIO_MODE_IT_RFT)
            {
                EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            }

            uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
            uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
            uint8_t code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

            AFIO_PCLK_EN();
            AFIO->EXTICR[temp1] = code << (temp2 * 4);

            EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
        }
    }
    
    
    //2. configure speed
    //No need because in mode we have already configured the speed

    //3. configure pupd setting
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl != GPIO_PIN_NO_PUPD)
    {
        // if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
        // {
        //     temp = pGPIOHandle->pGPIOx->CRL.REGS & ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4);
        //     temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4);
        //     pGPIOHandle->pGPIOx->CRL.REGS |= temp;
        // }
        // else
        // {
        //     temp = pGPIOHandle->pGPIOx->CRH.REGS & ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) * 4);
        //     temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) * 4);
        //     pGPIOHandle->pGPIOx->CRH.REGS |= temp;
        // }

        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PIN_PU)
        {
            pGPIOHandle->pGPIOx->ODR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PIN_PD)
        {
            pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        
    }
    
    
    //4. configure output type
    //No need because in mode we have already configured the output type

    //5. configure alternate function
    //No need because in mode we have already configured the alternate function
    

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET(); 
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)(pGPIOx->IDR);
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31) // 0 - 31
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (31 < IRQNumber && IRQNumber < 64) // 32 - 63
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (64 <= IRQNumber && IRQNumber < 96) // 64 - 95
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31) // 0 - 31
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (31 < IRQNumber && IRQNumber < 64) // 32 - 63
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (64 <= IRQNumber && IRQNumber < 96) // 64 - 95
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
    
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4;

    *(NVIC_PR_BASEADDR + ipr) &= ~(0xF << (8 * irq + 4));

    *(NVIC_PR_BASEADDR + ipr) |= (IRQPriority << (8 * irq + 4));

}
void GPIO_IRQHandler(uint8_t PinNumber)
{
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }
}

