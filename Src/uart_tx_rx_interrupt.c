/*
 * uart_tx_rx_interrupt.c
 *
 *  Created on: Aug 18, 2024
 *      Author: LENOVO
 */

#include <stdio.h>
#include <string.h>
#include "stm32f1.h"
#include "stm32f1_gpio.h"
#include "stm32f1_uart.h"

char rx_buf[7];

int a = 0, b = 0;

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart2_handle.USART_Config.USART_NoOfStopBit = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BIT;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
    GPIO_Handle_t UARTPins;

    UARTPins.pGPIOx = GPIOA;
    

    //USART2 TX
    UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_OUTPUT_PP_50MHz;
    UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&UARTPins);

    //USART2 RX
    UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_PULL;
    UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    UARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_Init(&UARTPins);
}

int main()
{
    USART2_GPIOInit();
    USART2_Init();
    USART_IRQInteruptConfig(IRQ_NO_USART2, ENABLE);
    USART_PeripheralControl(USART2, ENABLE);

    while(1)
    {
        while( USART_ReceiveDataIT(&usart2_handle, (uint8_t *)rx_buf, 6) != USART_READY);
        USART_SendDataIT(&usart2_handle, (uint8_t *)rx_buf, 6);
    }    
}

void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
    if(AppEv == USART_EVENT_RX_CMPLT)
    {
        a = 1;
    }
    else if(AppEv == USART_EVENT_TX_CMPLT)
    {
        b = 1;
    }
}
