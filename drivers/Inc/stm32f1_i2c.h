/*
 * stm32f1_i2c.h
 *
 *  Created on: Aug 20, 2024
 *      Author: LENOVO
 */

#ifndef INC_STM32F1_I2C_H_
#define INC_STM32F1_I2C_H_

#include "stm32f1.h"
#include <stdint.h>
#include "stddef.h"

typedef struct 
{
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddress;
    uint8_t  I2C_ACKControl;
    uint8_t  I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
    I2C_RegDef_t        *pI2Cx;
    I2C_Config_t        I2C_Config;

    uint8_t             *pTxBuffer;
    uint8_t             *pRxBuffer;
    uint32_t            TxLen;
    uint32_t            RxLen;
    uint8_t             TxRxState;
    uint8_t             DeVAddr;        //Slave/ Device Address
    uint32_t            RxSize;
    uint8_t             Sr;             //Repeated start value

} I2C_Handle_t;

#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

//I2C_SCLSpeed
#define I2C_SCL_SPEED_SM        10000
#define I2C_SCL_SPEED_FM4K      40000

//I2C AclControl
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

//I2C DutyCycle
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1

//FLAG
#define I2C_FLAG_TXE            (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE           (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB             (1 << I2C_SR1_SB)
#define I2C_FLAG_OVR            (1 << I2C_SR1_OVR)
#define I2C_FLAG_AF             (1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO           (1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR           (1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF          (1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10          (1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF            (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR           (1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT        (1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR          RESET
#define I2C_ENABLE_SR           SET

//Application events
#define I2C_EV_TX_CMPLT         0
#define I2C_EV_RX_CMPLT         1
#define I2C_EV_STOP             2
#define I2C_ERROR_BERR          3
#define I2C_ERROR_ARLO          4
#define I2C_ERROR_AF            5
#define I2C_ERROR_OVR           6
#define I2C_ERROR_TIMEOUT       7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Init and Deinit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
/*
 * IRQ Configuration and ISR Handling 
 */
void I2C_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs 
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t StatusflagName);
void I2C_ManageAcKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Appication callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F1_I2C_H_ */
