/*
 * button.c
 *
 *  Created on: Aug 7, 2024
 *      Author: ADMIN
 */
#include <stm32f1_spi.h>
#include "stddef.h"

static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_EER_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }

    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
    
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t temp = 0;
    //enable CLK

    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    //1. configure device mode
    temp |= pSPIHandle->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

    //2. configure bus config
    if (pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        //BIDIMODE should be cleared
        temp &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        //BIDIMODE should be set
        temp |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        //BIDIMODE should be cleared
        temp &= ~(1 << SPI_CR1_BIDIMODE);
        //RXONLY should be set
        temp |= (1 << SPI_CR1_RXONLY);
    } 

    //3. configure spi serial clock speed (baud rate)
    temp |= pSPIHandle->SPI_config.SPI_SclkSpeed << SPI_CR1_BR;

    //4. configure DFF
    temp |= pSPIHandle->SPI_config.SPI_DFF << SPI_CR1_DFF;

    //5. configure CPOL
    temp |= pSPIHandle->SPI_config.SPI_CPOL << SPI_CR1_CPOL;

    //6. configure CPHA
    temp |= pSPIHandle->SPI_config.SPI_CPHA << SPI_CR1_CPHA;

    temp |= pSPIHandle->SPI_config.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
    if (pSPIx->SR & flagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
    while (len > 0)
    {
        //1. wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        //2. check the DFF in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            //16-bits
            //1. load data to DR
            pSPIx->DR = *((uint16_t*) pTxBuffer);
            len--;
            len--;
            (uint16_t*)pTxBuffer++;
        }
        else
        {
            //8-bits
            pSPIx->DR = *pTxBuffer;
            len--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
    while (len > 0)
    {
        //1. wait until RXNE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

        //2. check the DFF in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            //16-bits
            //1. load data from DR to RxBuffer address
            *((uint16_t*) pRxBuffer) = pSPIx->DR;
            len--;
            len--;
            (uint16_t*)pRxBuffer++;
        }
        else
        {
            //8-bits
            *pRxBuffer = pSPIx->DR ;
            len--;
            pRxBuffer++;
        }
    }
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
    
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }   
}

void SPI_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4;

    *(NVIC_PR_BASEADDR + ipr) &= ~(0xF << (8 * irq + 4));

    *(NVIC_PR_BASEADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
    uint8_t state = pSPIHandle->TxState;
    if(state != SPI_BUSY_IN_TX)
    {
        //1. Save the TX buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = len;
        //2. Mark the SPI state as busy in transmistion so that
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        //3. Enable the TXEIE control bit to get interrupt whenever TXE is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
        //When, transmit will be handled ai ISR code
    }

    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
    uint8_t state = pSPIHandle->RxState;
    if(state != SPI_BUSY_IN_RX)
    {
        //1. Save the TX buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = len;
        //2. Mark the SPI state as busy in transmisstion so that
        pSPIHandle->RxState = SPI_BUSY_IN_RX;
        //3. Enable the RXNEIE control bit to get interrupt wwhenever TXE is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
        //When, transmit will be handled ai ISR code
    }

    return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    //TO DO
    uint8_t temp1, temp2;

    /*******************  TXE  *******************/
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(temp1 && temp2)
    {
        //handled TXE
        SPI_TXE_Interrupt_Handle(pSPIHandle);
    }

    /*******************  RXNE  *******************/
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2)
    {
        //handled RXNE
        SPI_RXNE_Interrupt_Handle(pSPIHandle);
    }

    /*******************  OVR  *******************/
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(temp1 && temp2)
    {
        //OVR Error
        SPI_OVR_EER_Interrupt_Handle(pSPIHandle);
    }
}

static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    //2. check the DFF in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        //16-bits
        //1. load data to DR to RxBuffer address
        pSPIHandle->pSPIx->DR = *(uint16_t*)pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen --;
        pSPIHandle->TxLen --;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else
    {
        //8-bits
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen --;
        pSPIHandle->pTxBuffer++;
    }

    if(!pSPIHandle->TxLen)  
    {
        //TxLen is zero so close the transmisstion and inform the application that
        //TX is over

        SPI_CloseTransmition(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    } 
}

void SPI_CloseTransmition(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    //2. check the DFF in CR1
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        //16-bits
        //1. load data from DR to RxBuffer address
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;

        pSPIHandle->pRxBuffer++;
        pSPIHandle->pRxBuffer++;
    }
    else
    {
        //8-bits
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen --;
        pSPIHandle->pRxBuffer++;
    }

    if(!pSPIHandle->RxLen)  
    {
        //Reception is complete

        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    } 
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

static void SPI_OVR_EER_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;

    //1. clear the ovr flag
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    //2. inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

}
