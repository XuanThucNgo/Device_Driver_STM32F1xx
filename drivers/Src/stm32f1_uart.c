/*
 * stm32f1_uart.c
 *
 *  Created on: Aug 16, 2024
 *      Author: LENOVO
 */

#include <stm32f1_uart.h>
#include <stm32f1_rcc.h>


/*
 * Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_EN(); 
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_EN();
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_EN();
        }
    }
    else
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_DI();
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_DI();
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_DI(); 
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_DI();
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_DI();
        }        
    }
}

/*
 * Init and Deinit
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    uint32_t temp = 0;

    //enable CLK
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);  

    //BEGIN CR1   
    //Mode
    if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        temp |= (1 << USART_CR1_TE);
    }
    else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        temp |= (1 << USART_CR1_RE);
    }
    else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        temp |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
    }

    //Word length
    temp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    //Parity
    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        //Implement the code to enable the parity control
        temp |= (1 << USART_CR1_PCE);
        //Implement the code to enable the EVEN parity 
        //temp |= (1 << USART_CR1_PS);
    }
    else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        //Implement the code to enable the parity control
        temp |= (1 << USART_CR1_PCE);
        //Implement the code to enable the EVEN parity 
        temp |= (1 << USART_CR1_PS);
    }

    pUSARTHandle->pUSARTx->CR1 = temp;

    //END CR1

    //BEGIN CR2
    temp = 0;

    //Stop bit
    temp |= pUSARTHandle->USART_Config.USART_NoOfStopBit << USART_CR2_STOP;

    pUSARTHandle->pUSARTx->CR2 = temp;
    //END CR2

    //BEGIN CR3
    temp = 0;
    if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        temp |= (1 << USART_CR3_CTSE);
    }
    else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        temp |= (1 << USART_CR3_RTSE);
    }
    else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        temp |= (1 << USART_CR3_RTSE);
        temp |= (1 << USART_CR3_CTSE);
    }

    pUSARTHandle->pUSARTx->CR3 = temp;

    USART_SetBaudrate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

    //END CR3


}
void USART_DeInit(USART_Handle_t *pUSARTHandle)
{
    //return 0;
}

void USART_SetBaudrate(USART_RegDef_t *pUSARTx, uint32_t Baudrate)
{
    uint32_t PCLKx;
    uint32_t usartdiv;
    uint32_t M_part, F_part;

    uint32_t tempreg = 0;

    //Get Clock
    if(pUSARTx == USART1)
    {
        PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
        PCLKx = RCC_GetPCLK1Value();
    }

    //UARTDIV
    usartdiv = ((25 * PCLKx) / (4 * Baudrate));

    //Mantissan: M_Part
    M_part = usartdiv / 100;
    tempreg |= M_part << 4;

    //Fraction:  F_Part
    F_part = (usartdiv - (M_part * 100));
    F_part = (((F_part * 16) + 50) / 100 & (uint8_t) 0x0F);
    tempreg |= F_part;

    pUSARTx->BRR = tempreg;
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        //Set bit UE in CR1 register
        pUSARTx->CR1 |= (1 << 13);
    }
    else
    {
        //Reset bit UE in CR1 register
        pUSARTx->CR1 &= ~(1 << 13);
    }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusflagName)
{
    uint8_t status = RESET;
    if(pUSARTx->SR & StatusflagName)
    {
        status = SET;
    }
    return status;
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
    uint16_t *pdata;

    for (uint32_t i = 0; i < len; i++)
    {
        /* Wait until TXE flag is set in SR */
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));
        
        //Check the USART WorkLength item for 9 BIT or 8 BIT in a frame
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BIT)
        {
            //Load to DR register 2 byte
            pdata = (uint16_t*) pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x1FF);

            //Check for USART_parityControl
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //Bits of user data will be sent 9 bit (2 byte)
                pTxBuffer++;
                pTxBuffer++;
            }
            else //USART_PARITY_ENABLE = 8 bit
            {
                //Increment the buffer address
                pTxBuffer++;
            }
        }
        else //USART_WordLength 8 bits
        {
            //This is 8bits data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
            //Increment the buffer address
            pTxBuffer++;
        }
    }
    
    //wait till TC flag is st in SR
    while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        //Wait untill RXNE set, khi buff chưa đủ data thì đợi
        while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

        //Check WorkLength 8 bit or 9 bit
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BIT)
        {
            //Check parity
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

                pRxBuffer++;
                pRxBuffer++;
            }
            else    //parity enable   9 bit = 8 bit + 1 parity
            {
                *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
                pRxBuffer++;
            }
        }
        else    //8 bit
        {
            //Check parity
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
            }
            else    //parity enable   8 bit = 7 bit + 1 parity
            {
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
            }

            pRxBuffer++;
        }
    }
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusflagName)
{
    pUSARTx->SR &= ~(StatusflagName);
}

void USART_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4; 

    //*(NVIC_PR_BASEADDR + ipr) &= ~(0xF << (8 * irq + 4));

    *(NVIC_PR_BASEADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
    uint8_t txstate = pUSARTHandle->TxBusyState;

    if(txstate != USART_BUSY_IN_TX)
    {
        pUSARTHandle->TxLen = len;
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        //Enable interrupt for TXE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        //Enable interrupt for TC
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    return txstate;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
    uint8_t rxstate = pUSARTHandle->RxBusyState;
    
    if (rxstate != USART_BUSY_IN_RX)
    {
        pUSARTHandle->RxLen = len;
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        (void)pUSARTHandle->pUSARTx->DR;

        //Enable interrupt for RXNE
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }
    
    return rxstate;
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
    uint32_t temp1, temp2;
    uint16_t *pdata;

    //TC flag
    //Check the state of TC bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
    //Check the state of TCEIE bit in register CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

    if(temp1 && temp2)
    {
        //Interrupt is because of TC
        //If Txlen is zero close transmit and call application
        if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            if (! pUSARTHandle->TxLen)
            {
                //Clear the TC flag
                pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
                //Clear the TCIE control bit
                pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

                pUSARTHandle->TxBusyState = USART_READY;
                pUSARTHandle->pTxBuffer = NULL;
                pUSARTHandle->TxLen = 0;

                //Call application
                USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
            }
            
        }
    }

    //TXE Flag
    //Check the state of TXE bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
    //Check the state of TXEIE bit in register CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

    if(temp1 && temp2)
    {
        //Interrupt is because of TXE
        if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            if(pUSARTHandle->TxLen > 0)
            {
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BIT)
                {
                    pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
                    pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        pUSARTHandle->pTxBuffer++;
                        pUSARTHandle->pTxBuffer++;
                        pUSARTHandle->TxLen -= 2;
                    }
                    else
                    {
                        pUSARTHandle->pTxBuffer++;
                        pUSARTHandle->TxLen -= 1;
                    }
                }
                else
                {
                    pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
                    pUSARTHandle->pTxBuffer++;
                    pUSARTHandle->TxLen -= 1;

                }
            }
            if(pUSARTHandle->TxLen == 0)
            {
                //clear the TXRIE
                pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
            }
        }

    }

    //RXNE Flag
    //Check the state of RXEIE bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
    //Check the state of RXNEIE bit in register CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

    if(temp1 && temp2)
    {
        if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
        {
            if(pUSARTHandle->RxLen > 0)
            {
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BIT)
                {
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        *((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
                        
                        pUSARTHandle->pRxBuffer++;
                        pUSARTHandle->pRxBuffer++;
                        pUSARTHandle->RxLen -= 2;
                    }
                    else
                    {
                        *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
                        pUSARTHandle->pRxBuffer++;
                        pUSARTHandle->RxLen -= 1;
                    }
                }
                else
                {
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
                    }
                    else
                    {
                        *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
                    }

                    pUSARTHandle->pRxBuffer++;
                    pUSARTHandle->RxLen -= 1;

                }
            }
            if(!pUSARTHandle->RxLen)
            {
                //Disable the rxne
                pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
                pUSARTHandle->RxBusyState = USART_READY;
                USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
            }
        }
    }

    //CTS Flag
    // Not use for UART4 and UART5
    //Check the state of CTS bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
    //Check the state of CTSE bit in register CR3
    temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

    if(temp1 && temp2)
    {
        //Imlement the code to clear the CTS flag in SR
        pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

        //This interrupt because of CTS
        USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
    }

    //IDLE dectection flag
    //Check the state of IDLE bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
    //Check the state of IDLEIE bit in register CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

    if(temp1 && temp2)
    {
        //Imlement the code to clear the IDLE flag in SR
        pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

        //This interrupt because of IDLE
        USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
    }

    //Overrun detection flag
    //Check the state of ORE bit in register SR
    temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
    //Check the state of RXNEIE bit in register CR1
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

    if(temp1 && temp2)
    {
        //This interrupt because of Overrun Error
        USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
    }

    //Error flag
    temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

    if(temp2)
    {
        temp1 = pUSARTHandle->pUSARTx->SR;
        if(temp1 & (1 << USART_SR_FE))
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
        }

        if(temp1 & (1 << USART_SR_NE))
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
        }
        if(temp1 & (1 << USART_SR_ORE))
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE); 
        }
    }

}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{

}


