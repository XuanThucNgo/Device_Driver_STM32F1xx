/*
 * stm32f1_i2c.c
 *
 *  Created on: Aug 20, 2024
 *      Author: LENOVO
 */


#include "stm32f1.h"
#include "stm32f1_i2c.h"
#include "stm32f1_rcc.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/w bit = 0
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;
    //Check for Device Mode
    if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RxSize == 1)
            {
                //First disable the ACK
                I2C_ManageAcKing(pI2CHandle->pI2Cx, DISABLE);

                //Clear the ADDR flag (Read SR1 and Read SR2)
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else
        {
            //Clear the ADDR flag (Read SR1 and Read SR2)
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;            
        }
        
    }
    else
    {
        //Clear the ADDR flag (Read SR1 and Read SR2)
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1;              //SlaveAddr is Slave address + r/w bit = 1
    pI2Cx->DR = SlaveAddr;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->TxLen > 0)
    {
        //1. Load the data in to DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        //2. Decrement the TxLen
        pI2CHandle->TxLen--;

        //3. Increment the buffer address
        pI2CHandle->pTxBuffer++;
    }
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    //We have to do the data reception
    if(pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxSize > 1)
    {
        if(pI2CHandle->RxLen == 2)
        {
            //Clear the ACK Bit
            I2C_ManageAcKing(pI2CHandle->pI2Cx, DISABLE);
        }
        //Read the data from data register in to buffer
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        //Increment the buffer address
        pI2CHandle->pRxBuffer++;
        //Decrement the RxLen
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0)
    {
        //Close the I2C data reception and notify the applicatiomn

        //1. Generate the stop condition
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        //2. Close the I2C RX
        I2C_CloseReceiveData(pI2CHandle);

        //3. Notify the application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }     
    }
    else
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }          
    }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp = 0;

    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    //////////////////Begin CR1/////////////////////////
    //ACK
    temp |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
    pI2CHandle->pI2Cx->CR1 = temp;
    ///////////////////End CR1//////////////////////////

    //////////////////Begin CR2/////////////////////////
    //FREG CR2
    temp = 0;
    temp |= RCC_GetPCLK1Value() / 1000000U;

    pI2CHandle->pI2Cx->CR2 = temp & 0x3F;
    //////////////////End CR2///////////////////////////

    //////////////////Begin OAR1/////////////////////////
    temp = 0;
    temp |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    temp |= 1 << 14;        //Should always be kept at 1 by Sofware
    pI2CHandle->pI2Cx->OAR1 = temp;
    //////////////////End OAR1/////////////////////////

    //////////////////Begin CCR/////////////////////////
    temp = 0;
    uint16_t ccr_value = 0;

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        //Standard Mode
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        temp |= (ccr_value & 0xFFF);
    }
    else
    {
        //Fast Mode
        temp |= (1 << 15);
        temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        //Check DUTY
        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else
        {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }

        temp |= ccr_value & 0xFFF;
    }
    pI2CHandle->pI2Cx->OAR1 = temp;
    //////////////////End CCR/////////////////////////

    //////////////////Begin TRISE/////////////////////////
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        //Standard Mode
        temp = (RCC_GetPCLK1Value() / 1000000) + 1;
    }
    else
    {
        //Fast Mode
        temp = ((RCC_GetPCLK1Value() * 300) / 1000000000) + 1;
    }
    pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
    //////////////////End TRISE/////////////////////////
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t StatusflagName)
{
    uint8_t status = RESET;
    if(pI2Cx->SR1 & StatusflagName)
    {
        status = SET;
    }
    return status;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
    //Start Condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    //Checking the flag in the SR1 => CLK = LOW
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    //Send address data and bir read/write
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    //Checking the ADDR flag in the SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    //Clear the ADDR flag, SCL = LOW
    I2C_ClearADDRFlag(pI2CHandle);

    //Send the data untill len become 0
    while(len > 0)
    {
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till TXE is set
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;     
    }

    //when Len becomes 0 wait for flag TXE = 1 and BTF = 1 before ganerating the STOP condition
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till TXE is set
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); //Wait till BTF is set

    //Generate STOP condition
    //Note: generating STOP, automatically 

    if(Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

}

void I2C_ManageAcKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == I2C_ACK_ENABLE)
    {
        //Enable the ACK
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        //Disable the ACK
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
    //1. Start condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    //2. Confirm that start generation
    //Checking the flag in the SR1 => CLK = LOW
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    //3. Send the address of the slave with r/rw bit set to R(1) (total 8 bit)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    //Checking the ADDR flag in the SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    //Read only 1 byte from slave
    if(len == 1)
    {
        //Disable Acking
        I2C_ManageAcKing(pI2CHandle->pI2Cx, DISABLE);

        //Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        //Wait until RXNE set 1
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        //Generate STOP condition
        if(Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        //Read data in to buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;        
    }
    //When len > 1
    if(len > 1)
    {
        //Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);
        //Read the data until Len become Zero
        for (uint32_t i = len; i > 0; i--)
        {
            //Wait until RXNE set to 1
            while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if(i == 2)  //if last 2 byte are remaining
            {
                //Disable Acking
                I2C_ManageAcKing(pI2CHandle->pI2Cx, DISABLE); 
                //Generate STOP condition
                if(Sr == I2C_DISABLE_SR)
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }   
            }
            //Read the data from data register in to buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR; 
            pRxBuffer++;
        }
    }

    //Re-enable ACKing, because ACK is set when PE set 
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcKing(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

void I2C_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4; 

    //*(NVIC_PR_BASEADDR + ipr) &= ~(0xF << (8 * irq + 4));

    *(NVIC_PR_BASEADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;
    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DeVAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Start Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITBUFEN));

        //Implement the code to enable ITEVFEN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITEVTEN));

        //Implement the code to enable ITERREN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITERREN));
    }

    return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;
    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pRxBuffer;
        pI2CHandle->TxLen = len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DeVAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement the code to to Generation Start Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITBUFEN));

        //Implement the code to enable ITEVFEN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITEVTEN));

        //Implement the code to enable ITERREN control Bit 
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_CR2_ITERREN));
    }

    return busystate;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    //Implement the code to disable ITBUFEN control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    //Implement the code to disable ITBUFEN control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcKing(pI2CHandle->pI2Cx, ENABLE);
    }    
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    //Implement handling for both master and slave mode of device

    uint32_t temp1, temp2, temp3;
    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    //SB Flag
    if(temp1 && temp3)
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DeVAddr);
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DeVAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    //ADDR event
    //When Master mode: Address is sent
    //When Slave mode:  Địa chỉ khớp với địa chỉ riêng
    if(temp1 && temp3)
    {
        //Interrupt is generated because of ADDR event
        I2C_ClearADDRFlag(pI2CHandle);
    } 

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    //Byte Transfer Finished
    if(temp1 && temp3)
    {
        //BTF flag is set
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            //Make sure that TXE is also set
            if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
            {
                //BTF, TXE = 1
                if (pI2CHandle->TxLen == 0)
                {
                    //1. Generate the STOP condition
                    if (pI2CHandle->Sr == I2C_DISABLE_SR)
                    {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    //2.Reset all the member elements of the handle structure
                    I2C_CloseSendData(pI2CHandle);

                    //3. Notify the application about transmisstion complete
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                    
                }
                
            }
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            ;
        }
        
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    if(temp1 && temp3)
    {
        //STOPF flag is set
        //Clear the STOPF

        pI2CHandle->pI2Cx->CR1 |= 0x0000;
        //Notify the application that STOPF is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    if (temp1 && temp2 && temp3)
    {
        //Check for device mode
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            //TXE flag is set
            //We have to do data transmisstion
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            //Slave
            //Make sure that slave is really in transmistion mode
            if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }
    
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    if (temp1 && temp2 && temp3)
    {
        //Check for device mode
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            //RXNE flag is set
            //We have to do data transmisstion
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            //Slave
            //Make sure that slave is really in transmistion mode
            if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    //ITERREN control bit
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    //Check for Bus error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if(temp1 && temp2)
    {
        //This is Bus error

        //Implement the code to clear the bus error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    //Check for arbitration lost error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if(temp1 && temp2)
    {
        //This is arbitration lost error

        //Implement the code to clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    //Check for ACK failure error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if(temp1 && temp2)
    {
        //This is ACK failure error

        //Implement the code to clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    //Check for Overrun/Underrun error
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if(temp1 && temp2)
    {
        //This is Overrun/Underrun error

        //Implement the code to clear the Overrun/Underrun error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }  
    //Check for Timeout error 
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if(temp1 && temp2)
    {
        //This is Timeout error

        //Implement the code to clear the Timeout error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }  
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t) pI2Cx->DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);        
    }
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
}




