#include <stdint.h>
#include <stm32f1.h>
#ifndef INC_STM32F1_SPI_H_
#define INC_STM32F1_SPI_H_

typedef struct
{
    uint8_t SPI_DeviceMode;         //Master or Slave
    uint8_t SPI_BusConfig;          //Full-duplex/Half_duplex/Simplex RXOnly
    uint8_t SPI_SclkSpeed;          //Speed of CLK
    uint8_t SPI_DFF;                //Data Frame Format
    uint8_t SPI_CPOL;               //CPOL in SPI protocol (usually 0)
    uint8_t SPI_CPHA;               //CPHA in SPI protocol (usually 0)
    uint8_t SPI_SSM;                //used in 1:1 communication between Master and Slave
} SPI_Config_t;

typedef struct
{
    SPI_Config_t    SPI_config;
    SPI_RegDef_t    *pSPIx;

    uint8_t         *pTxBuffer;
    uint8_t         *pRxBuffer;
    uint32_t        TxLen;
    uint32_t        RxLen;
    uint8_t         TxState;
    uint8_t         RxState;
} SPI_Handle_t;


/*
 * SPI Appication Event
 */
#define SPI_EVENT_TX_CMPLT          1
#define SPI_EVENT_RX_CMPLT          2
#define SPI_EVENT_OVR_ERR           3
#define SPI_EVENT_CRC_ERR           4

//SPI application states
#define SPI_READY                   0
#define SPI_BUSY_IN_TX              2
#define SPI_BUSY_IN_RX              1

#define SPI_DEVICE_MODE_MASTER      1
#define SPI_DEVICE_MODE_SLAVE       0

#define SPI_BUS_CONFIG_FD                   1
#define SPI_BUS_CONFIG_HD                   2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       3

#define SPI_SCLK_SPEED_DIV2                 0
#define SPI_SCLK_SPEED_DIV4                 1
#define SPI_SCLK_SPEED_DIV8                 2
#define SPI_SCLK_SPEED_DIV16                3
#define SPI_SCLK_SPEED_DIV32                4
#define SPI_SCLK_SPEED_DIV64                5
#define SPI_SCLK_SPEED_DIV128               6
#define SPI_SCLK_SPEED_DIV256               7

#define SPI_DFF_8BITS                       0
#define SPI_DFF_16BITS                      1


#define SPI_CPOL_HIGH                       1
#define SPI_CPOL_LOW                        0

#define SPI_CPHA_HIGH                       1
#define SPI_CPHA_LOW                        0

#define SPI_SSM_EN                          1
#define SPI_SSM_DI                          0

#define SPI_TXE_FLAG                        (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                       (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG                        (1 << SPI_SR_BSY)



void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data Send / Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

void SPI_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmition(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Appication callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);




#endif //INC_STM32F1_SPI_H_
