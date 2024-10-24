/*
 * stm32f1_uart.h
 *
 *  Created on: Aug 16, 2024
 *      Author: LENOVO
 */

#ifndef INC_STM32F1_UART_H_
#define INC_STM32F1_UART_H_

#include "stm32f1.h"
#include <stdint.h>
#include "stddef.h"

typedef struct
{
    uint8_t USART_Mode;                      
    uint32_t USART_Baud;                      
    uint8_t USART_NoOfStopBit;               
    uint8_t USART_WordLength;                
    uint8_t USART_ParityControl;         
    uint8_t USART_HWFlowControl;            
} USART_Config_t;

typedef struct
{
    USART_Config_t      USART_Config;
    USART_RegDef_t      *pUSARTx;

    uint8_t             *pTxBuffer;
    uint8_t             *pRxBuffer;
    uint32_t            TxLen;
    uint32_t            RxLen;
    uint8_t             TxBusyState;
    uint8_t             RxBusyState;
} USART_Handle_t;


//Mode for USART
#define USART_MODE_ONLY_TX              0
#define USART_MODE_ONLY_RX              1
#define USART_MODE_TXRX                 2

//Baudrate
#define USART_STD_BAUD_1200             1200
#define USART_STD_BAUD_2400             2400
#define USART_STD_BAUD_9600             9600
#define USART_STD_BAUD_19200            19200
#define USART_STD_BAUD_38400            38400
#define USART_STD_BAUD_57600            57600
#define USART_STD_BAUD_115200           115200 
#define USART_STD_BAUD_230400           230400
#define USART_STD_BAUD_460800           460800
#define USART_STD_BAUD_921600           921600
#define USART_STD_BAUD_2M               2000000
#define USART_STD_BAUD_3M               3000000

//USART_ParityControl
#define USART_PARITY_DISABLE            0
#define USART_PARITY_EN_EVEN            1
#define USART_PARITY_EN_ODD             2

//WordLength            
#define USART_WORDLEN_8BIT              0
#define USART_WORDLEN_9BIT              1

//NoOfStopBits          
#define USART_STOPBITS_1                0
#define USART_STOPBITS_0_5              1
#define USART_STOPBITS_2                2
#define USART_STOPBITS_1_5              3

//HWFlowControl
#define USART_HW_FLOW_CTRL_NONE         0
#define USART_HW_FLOW_CTRL_CTS          1
#define USART_HW_FLOW_CTRL_RTS          2
#define USART_HW_FLOW_CTRL_CTS_RTS      3

//USART Flags
#define USART_FLAG_TXE                  (1 << USART_SR_TXE)
#define USART_FLAG_RXNE                 (1 << USART_SR_RXNE)
#define USART_FLAG_TC                   (1 << USART_SR_TC)

//Application state
#define USART_READY                     0
#define USART_BUSY_IN_RX                1
#define USART_BUSY_IN_TX                2

#define USART_EVENT_TX_CMPLT            0
#define USART_EVENT_RX_CMPLT            1
#define USART_EVENT_IDLE                2
#define USART_EVENT_CTS                 3
#define USART_EVENT_PE                  4
#define USART_ERR_FE                    5
#define USART_ERR_NE                    6
#define USART_ERR_ORE                   7

/*
 * Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and Deinit
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling 
 */
void USART_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs 
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusflagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusflagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_SetBaudrate(USART_RegDef_t *pUSARTx, uint32_t Baudrate);

/*
 * Appication callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);


























#endif /* INC_STM32F1_UART_H_ */
