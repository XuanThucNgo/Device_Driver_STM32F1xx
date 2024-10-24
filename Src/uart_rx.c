/*
 * uart_rx.c
 *
 *  Created on: Aug 17, 2024
 *      Author: LENOVO
 */

#include <stdio.h>
#include <string.h>
#include "stm32f1.h"
#include "stm32f1_gpio.h"
#include "stm32f1_uart.h"

char msg[10];
int a = 0, b = 0;

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_RX;
    usart2_handle.USART_Config.USART_NoOfStopBit = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BIT;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
    GPIO_Handle_t UARTPins;

    UARTPins.pGPIOx = GPIOA;
    //UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_OUTPUT_OD_50MHz;
    UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_PULL;
    UARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    //USART2 TX
    UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&UARTPins);

    //USART2 RX
    UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&UARTPins);
}

int main(void)
{
    USART2_GPIOInit();
    USART2_Init();
    USART_PeripheralControl(USART2, ENABLE);

    while(1)
    {

        USART_ReceiveData(&usart2_handle, (uint8_t *)msg, 6);


        //while(1);
    }
    return 0;

}


