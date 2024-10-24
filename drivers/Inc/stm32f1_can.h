#include "stm32f1.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef INC_STM32F1_CAN_H_
#define INC_STM32F1_CAN_H_


typedef enum
{
   CAN_STATE_RESET            =0x00U,
   CAN_STATE_READY            =0x01U,
   CAN_STATE_LISTENING        =0x02U,
   CAN_STATE_SLEEP_PENDING    =0x03U,
   CAN_STATE_SLEEP_ACTIVE     =0x04U,
   CAN_STATE_ERROR            =0x05U
} CAN_State_t;

/*
 * CAN init structure definition
 */
typedef struct 
{
   uint32_t Prescaler;
   uint32_t Mode;
   uint32_t SyncJumpWidth;
   uint32_t TimeSeg1;
   uint32_t TimeSeg2;
   bool     TimeTriggeredMode;
   bool     AutoBusOff;
   bool     AutoWakeUp;
   bool     AutoRetransmission;
   bool     ReceiveFifoLocked;
   bool     TransmissFifoPriority;
} CAN_Init_t;

/*
 * CAN filter
 */
typedef struct 
{
   uint32_t FilterIDHigh;
   uint32_t FilterIDLow;
   uint32_t FilterMaskIDHigh;
   uint32_t FilterMaskIDLow;
   uint32_t FilterFIFONumber;
   uint32_t FilterBank;
   uint32_t FilterMode;
   uint32_t FilterScale;
   uint32_t FilterActivation;
   uint32_t SlaveStartFilterBank;
} CAN_Filter_t;

/*
 * CAN Tx message header
 */
typedef struct 
{
   uint32_t StdId;
   uint32_t ExtId;
   uint32_t IDE;
   uint32_t RTR;
   uint32_t DLC;
   bool     TransmissGlobalTime;
} CAN_TxHeader_t;

/*
 * CAN Rx message header
 */
typedef struct 
{
   uint32_t StdId;
   uint32_t ExtId;
   uint32_t IDE;
   uint32_t RTR;
   uint32_t DLC;
   uint32_t TimeStamp;
   bool     FilterMatchIndex;
} CAN_RxHeader_t;

/*
 * @brief CAN handle Structure definition
 */
typedef struct
{
    CAN_RegDef_t *pCANx;
    CAN_Init_t   CAN_Config;
    CAN_State_t  State;
    uint32_t     ErrorCode;
    void (* TxMailbox0CompleteCallback)(struct CAN_Handle *CANx);   /* CAN Tx Mailbox 0 complete callback*/
    void (* TxMailbox1CompleteCallback)(struct CAN_Handle *CANx);   /* CAN Tx Mailbox 1 complete callback*/
    void (* TxMailbox2CompleteCallback)(struct CAN_Handle *CANx);   /* CAN Tx Mailbox 2 complete callback*/
    void (* TxMailbox0AbortCallback)(struct CAN_Handle *CANx);      /* CAN Tx Mailbox 0 abort callback   */
    void (* TxMailbox1AbortCallback)(struct CAN_Handle *CANx);      /* CAN Tx Mailbox 1 abort callback   */
    void (* TxMailbox2AbortCallback)(struct CAN_Handle *CANx);      /* CAN Tx Mailbox 2 abort callback   */
    void (* RxFifo0MsgPendingCallback)(struct CAN_Handle *CANx);    /* CAN Rx FIFO 0 msg pending callback*/
    void (* RxFifo0FullCallback)(struct CAN_Handle *CANx);          /* CAN Rx FIFO 0 full callback       */
    void (* RxFifo1MsgPendingCallback)(struct CAN_Handle *CANx);    /* CAN Rx FIFO 1 msg pending callback*/
    void (* RxFifo1FullCallback)(struct CAN_Handle *CANx);          /* CAN Rx FIFO 1 full callback       */
    void (* SleepCallback)(struct CAN_Handle *CANx);                /* CAN Sleep callback                */
    void (* WakeUpFromRxMsgCallback)(struct CAN_Handle *CANx);      /* CAN Wake Up from Rx msg callback  */
    void (* ErrorCallback)(struct CAN_Handle *CANx);                /* CAN Error callback                */
    void (* MspInitCallback)(struct CAN_Handle *CANx);              /* CAN Msp Init callback             */
    void (* MspDeInitCallback)(struct CAN_Handle *CANx);            /* CAN Msp DeInitcallback            */
} CAN_Handle_t;

typedef enum
{
   CAN_TX_MAILBOX0_COMPLETE_CB_ID      = 0x00U,       /* CAN Tx Mailbox 0 complete callback ID     */       
   CAN_TX_MAILBOX1_COMPLETE_CB_ID      = 0x01U,       /* CAN Tx Mailbox 1 complete callback ID     */
   CAN_TX_MAILBOX2_COMPLETE_CB_ID      = 0x02U,       /* CAN Tx Mailbox 2 complete callback ID     */
   CAN_TX_MAILBOX0_ABORT_CB_ID         = 0x03U,       /* CAN Tx Mailbox 0 abort callback ID        */
   CAN_TX_MAILBOX1_ABORT_CB_ID         = 0x04U,       /* CAN Tx Mailbox 1 abort callback ID        */
   CAN_TX_MAILBOX2_ABORT_CB_ID         = 0x05U,       /* CAN Tx Mailbox 2 abort callback ID        */
   CAN_RX_FIFO0_MSG_PENDING_CB_ID      = 0x06U,       /* CAN Rx FIFO 0 message pending callback ID */
   CAN_RX_FIFO0_FULL_CB_ID             = 0x07U,       /* CAN Rx FIFO 0 full callback ID            */
   CAN_RX_FIFO1_MSG_PENDING_CB_ID      = 0x08U,       /* CAN Rx FIFO 1 message pending callback ID */
   CAN_RX_FIFO1_FULL_CB_ID             = 0x09U,       /* CAN Rx FIFO 1 full callback ID            */
   CAN_SLEEP_CB_ID                     = 0x0AU,       /* CAN Sleep callback ID                     */
   CAN_WAKEUP_FROM_RX_MSG_CB_ID        = 0x0BU,       /* CAN Wake Up from Rx msg callback IB       */
   CAN_ERROR_CB_ID                     = 0x0CU,       /* CAN Error callback ID                     */
   CAN_MSPINIT_CB_ID                   = 0x0DU,       /* CAN MspInit callback ID                   */
   CAN_MSPDEINIT_CB_ID                 = 0x0DU,       /* CAN MspDeInit callback ID                 */
} CAN_CallbackID_t;

/* ErrorCode*/
#define CAN_ERROR_NONE              (0x00000000U)  /* No error*/
#define CAN_ERROR_EWG               (0x00000001U)  /* Protocol Error Warning*/
#define CAN_ERROR_EPV               (0x00000002U)  /* Error Passive*/
#define CAN_ERROR_BOF               (0x00000004U)  /* Bus-off Error*/
#define CAN_ERROR_STF               (0x00000008U)  /* Stuff error*/
#define CAN_ERROR_FOR               (0x00000010U)  /* Form error*/
#define CAN_ERROR_ACK               (0x00000020U)  /* Acknowledgment error*/
#define CAN_ERROR_BR                (0x00000040U)  /* Bit recessive error*/
#define CAN_ERROR_BD                (0x00000080U)  /* Bit dominant error*/
#define CAN_ERROR_CRC               (0x00000100U)  /* CRC error*/
#define CAN_ERROR_RX_FOV0           (0x00000200U)  /* Rx FIFO0 overrun error*/
#define CAN_ERROR_RX_FOV1           (0x00000400U)  /* Rx FIFO1 overrun error*/
#define CAN_ERROR_TX_ALST0          (0x00000800U)  /* TxMailbox 0 transmit failure due to arbitration lost*/
#define CAN_ERROR_TX_TERR0          (0x00001000U)  /* TxMailbox 0 transmit failure due to transmit error*/
#define CAN_ERROR_TX_ALST1          (0x00002000U)  /* TxMailbox 1 transmit failure due to arbitration lost*/
#define CAN_ERROR_TX_TERR1          (0x00004000U)  /* TxMailbox 1 transmit failure due to transmit error*/
#define CAN_ERROR_TX_ALST2          (0x00008000U)  /* TxMailbox 2 transmit failure due to arbitration lost*/
#define CAN_ERROR_TX_TERR2          (0x00010000U)  /* TxMailbox 2 transmit failure due to transmit error*/
#define CAN_ERROR_TIMEOUT           (0x00020000U)  /* Timeout error*/
#define CAN_ERROR_NOT_INITIALIZED   (0x00040000U)  /* Peripheral not initialized*/
#define CAN_ERROR_NOT_READY         (0x00080000U)  /* Peripheral not ready*/
#define CAN_ERROR_NOT_STARTED       (0x00100000U)  /* Peripheral not started*/
#define CAN_ERROR_PARAM             (0x00200000U)  /* Parameter error*/
#define CAN_ERROR_INVALID_CALLBACK  (0x00400000U)  /* Invalid Callback error*/
#define CAN_ERROR_INTERNAL          (0x00800000U)  /* Internal error*/

/* CAN InitStatus */
#define CAN_INITSTATUS_FAILED       (0x00000000U)
#define CAN_INITSTATUS_SUCCESS      (0x00000001U)

/* Mode */
#define CAN_MODE_NORMAL             (0x00000000U)                             /* Nomal Mode */
#define CAN_MODE_LOOPBACK           ((uint32_t)CAN_BTR_LBKM)                  /* Loopback Mode */
#define CAN_MODE_SILENT             ((uint32_t)CAN_BTR_SILM)                  /* Silent Mode */
#define CAN_MODE_SILENT_LOOPBACK    ((uint32_t)CAN_BTR_SILM | CAN_BTR_LBKM)   /* Silent combined with Loopback Mode */

/* SyncJumpWidth */
#define CAN_SJW_1TQ                 (0x00000000)               /* 1 time quantum*/
#define CAN_SJW_2TQ                 ((uint32_t)CAN_BTR_SJW_0)  /* 2 time quantum*/
#define CAN_SJW_3TQ                 ((uint32_t)CAN_BTR_SJW_1)  /* 3 time quantum*/
#define CAN_SJW_4TQ                 ((uint32_t)CAN_BTR_SJW)    /* 4 time quantum*/

/* SyncJumpWidth */
#define CAN_SJW_1TQ                 (0x00000000U)               /* 1 time quantum */
#define CAN_SJW_2TQ                 ((uint32_t)CAN_BTR_SJW_0)   /* 2 time quantum */
#define CAN_SJW_3TQ                 ((uint32_t)CAN_BTR_SJW_1)   /* 3 time quantum */
#define CAN_SJW_4TQ                 ((uint32_t)CAN_BTR_SJW)     /* 4 time quantum */

/* TimeSeg1 */
#define CAN_BS1_1TQ                 (0x00000000U)                                                   /* 1 time quantum*/
#define CAN_BS1_2TQ                 ((uint32_t)CAN_BTR_TS1_0)                                       /* 2 time quantum*/
#define CAN_BS1_3TQ                 ((uint32_t)CAN_BTR_TS1_1)                                       /* 3 time quantum*/
#define CAN_BS1_4TQ                 ((uint32_t)(CAN_BTR_TS1_1 | CAN_BTR_TS1_0))                     /* 4 time quantum*/
#define CAN_BS1_5TQ                 ((uint32_t)CAN_BTR_TS1_2)                                       /* 5 time quantum*/
#define CAN_BS1_6TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_0))                     /* 6 time quantum*/
#define CAN_BS1_7TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_1))                     /* 7 time quantum*/
#define CAN_BS1_8TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_1 | CAN_BTR_TS1_0))     /* 8 time quantum*/
#define CAN_BS1_9TQ                 ((uint32_t)CAN_BTR_TS1_3)                                       /* 9 time quantum*/
#define CAN_BS1_10TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_0))                     /* 10 time quantum*/
#define CAN_BS1_11TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_1))                     /* 11 time quantum*/
#define CAN_BS1_12TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_1 | CAN_BTR_TS1_0))     /* 12 time quantum*/
#define CAN_BS1_13TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2))                     /* 13 time quantum*/
#define CAN_BS1_14TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2 | CAN_BTR_TS1_0))     /* 14 time quantum*/
#define CAN_BS1_15TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2 | CAN_BTR_TS1_1))     /* 15 time quantum*/
#define CAN_BS1_16TQ                ((uint32_t)CAN_BTR_TS1)                                         /* 16 time quantum*/

/* TimeSeg2*/
#define CAN_BS2_1TQ                 (0x00000000U)                                   /* 1 time quantum*/                       
#define CAN_BS2_2TQ                 ((uint32_t)CAN_BTR_TS2_0)                       /* 2 time quantum*/
#define CAN_BS2_3TQ                 ((uint32_t)CAN_BTR_TS2_1)                       /* 3 time quantum*/
#define CAN_BS2_4TQ                 ((uint32_t)(CAN_BTR_TS2_1 | CAN_BTR_TS2_0))     /* 4 time quantum*/
#define CAN_BS2_5TQ                 ((uint32_t)CAN_BTR_TS2_2)                       /* 5 time quantum*/
#define CAN_BS2_6TQ                 ((uint32_t)(CAN_BTR_TS2_2 | CAN_BTR_TS2_0))     /* 6 time quantum*/
#define CAN_BS2_7TQ                 ((uint32_t)(CAN_BTR_TS2_2 | CAN_BTR_TS2_1))     /* 7 time quantum*/
#define CAN_BS2_8TQ                 ((uint32_t)CAN_BTR_TS2)                         /* 8 time quantum*/

/* FilterMode */
#define CAN_FILTERMODE_IDMASK       (0x00000000U)   /* Identifier mask mode */
#define CAN_FILTERMODE_IDLIST       (0x00000001U)   /* Identifier list mode */

/* FilterScale */
#define CAN_FILTERSCALE_16BIT       (0x00000000U)   /* Two 16-bit filters */
#define CAN_FILTERSCALE_32BIT       (0x00000001U)   /* One 32-bit filter */

/* FilterActivation */
#define CAN_FILTER_DISABLE       (0x00000000U)   /* Disable filter*/
#define CAN_FILTER_ENABLE        (0x00000001U)   /* Enable filter*/

/* FilterFIFONumber */
#define CAN_FILTER_FIFO0       (0x00000000U)   /* Filter FIFO 0 number for filter x*/
#define CAN_FILTER_FIFO1       (0x00000001U)   /* Filter FIFO 1 number for filter x*/

/* IDE */
#define CAN_ID_STD         (0x00000000U)        /* Standard ID*/
#define CAN_ID_EXT         (0x00000004U)        /* Extended ID*/

/* RTR - Remote Transmission Request */
#define CAN_RTR_DATA       (0x00000000U)        /* Data frame */
#define CAN_RTR_REMOTE     (0x00000002U)        /* Remote frame */

/* CAN Receive FIFO Number*/
#define CAN_RX_FIFO0       (0x00000000U)        /* CAN Receive FIFO 0 */
#define CAN_RX_FIFO1       (0x00000001U)        /* CAN Receive FIFO 1 */

/* CAN Tx Mailboxes*/
#define CAN_TX_MAILBOX0    (0x00000001U)        /* Tx Mailbox 0*/  
#define CAN_TX_MAILBOX1    (0x00000002U)        /* Tx Mailbox 1*/  
#define CAN_TX_MAILBOX2    (0x00000004U)        /* Tx Mailbox 2*/  

/* Transmit Flags */
#define CAN_FLAG_RQCP0               (0x00000500U)         /* Request complete Mailbox 0 flag   */
#define CAN_FLAG_TXOK0               (0x00000501U)         /* Transmission OK Mailbox 0 flag    */
#define CAN_FLAG_ALST0               (0x00000502U)         /* Arbitration Lost Mailbox 0 flag   */
#define CAN_FLAG_TERR0               (0x00000503U)         /* Transmission error Mailbox 0 flag */
#define CAN_FLAG_RQCP1               (0x00000508U)         /* Request complete Mailbox 1 flag   */
#define CAN_FLAG_TXOK1               (0x00000509U)         /* Transmission OK Mailbox 1 flag    */
#define CAN_FLAG_ALST1               (0x0000050AU)         /* Arbitration Lost Mailbox 1 flag   */
#define CAN_FLAG_TERR1               (0x0000050BU)         /* Transmission error Mailbox 1 flag */
#define CAN_FLAG_RQCP2               (0x00000510U)         /* Request complete Mailbox 2 flag   */
#define CAN_FLAG_TXOK2               (0x00000511U)         /* Transmission OK Mailbox 2 flag    */
#define CAN_FLAG_ALST2               (0x00000512U)         /* Arbitration Lost Mailbox 2 flag   */
#define CAN_FLAG_TERR2               (0x00000513U)         /* Transmission error Mailbox 2 flag */
#define CAN_FLAG_TME0                (0x0000051AU)         /* Transmit mailbox 0 empty flag     */
#define CAN_FLAG_TME1                (0x0000051BU)         /* Transmit mailbox 1 empty flag     */
#define CAN_FLAG_TME2                (0x0000051CU)         /* Transmit mailbox 2 empty flag     */
#define CAN_FLAG_LOW0                (0x0000051DU)         /* Lowest priority mailbox 0 flag    */
#define CAN_FLAG_LOW1                (0x0000051EU)         /* Lowest priority mailbox 1 flag    */
#define CAN_FLAG_LOW2                (0x0000051FU)         /* Lowest priority mailbox 2 flag    */

/* Receive Flags */
#define CAN_FLAG_FF0                 (0x00000203U)         /* RX FIFO 0 Full flag               */
#define CAN_FLAG_FOV0                (0x00000204U)         /* RX FIFO 0 Overrun flag            */
#define CAN_FLAG_FF1                 (0x00000403U)         /* RX FIFO 1 Full flag               */
#define CAN_FLAG_FOV1                (0x00000404U)         /* RX FIFO 1 Overrun flag            */

/* Operating Mode Flags */
#define CAN_FLAG_INAK                (0x00000100U)         /*!< Initialization acknowledge flag   */
#define CAN_FLAG_SLAK                (0x00000101U)         /*!< Sleep acknowledge flag            */
#define CAN_FLAG_ERRI                (0x00000102U)         /*!< Error flag                        */
#define CAN_FLAG_WKU                 (0x00000103U)         /*!< Wake up interrupt flag            */
#define CAN_FLAG_SLAKI               (0x00000104U)         /*!< Sleep acknowledge interrupt flag  */

/* Error Flags */
#define CAN_FLAG_EWG                 (0x00000300U)         /*!< Error warning flag                */
#define CAN_FLAG_EPV                 (0x00000301U)         /*!< Error passive flag                */
#define CAN_FLAG_BOF                 (0x00000302U)         /*!< Bus-Off flag 





/** @defgroup CAN_Interrupts CAN Interrupts
  * @{
  */
/* Transmit Interrupt */
#define CAN_IT_TX_MAILBOX_EMPTY     ((uint32_t)CAN_IER_TMEIE)   /*!< Transmit mailbox empty interrupt */

/* Receive Interrupts */
#define CAN_IT_RX_FIFO0_MSG_PENDING ((uint32_t)CAN_IER_FMPIE0)  /*!< FIFO 0 message pending interrupt */
#define CAN_IT_RX_FIFO0_FULL        ((uint32_t)CAN_IER_FFIE0)   /*!< FIFO 0 full interrupt            */
#define CAN_IT_RX_FIFO0_OVERRUN     ((uint32_t)CAN_IER_FOVIE0)  /*!< FIFO 0 overrun interrupt         */
#define CAN_IT_RX_FIFO1_MSG_PENDING ((uint32_t)CAN_IER_FMPIE1)  /*!< FIFO 1 message pending interrupt */
#define CAN_IT_RX_FIFO1_FULL        ((uint32_t)CAN_IER_FFIE1)   /*!< FIFO 1 full interrupt            */
#define CAN_IT_RX_FIFO1_OVERRUN     ((uint32_t)CAN_IER_FOVIE1)  /*!< FIFO 1 overrun interrupt         */

/* Operating Mode Interrupts */
#define CAN_IT_WAKEUP               ((uint32_t)CAN_IER_WKUIE)   /*!< Wake-up interrupt                */
#define CAN_IT_SLEEP_ACK            ((uint32_t)CAN_IER_SLKIE)   /*!< Sleep acknowledge interrupt      */

/* Error Interrupts */
#define CAN_IT_ERROR_WARNING        ((uint32_t)CAN_IER_EWGIE)   /*!< Error warning interrupt          */
#define CAN_IT_ERROR_PASSIVE        ((uint32_t)CAN_IER_EPVIE)   /*!< Error passive interrupt          */
#define CAN_IT_BUSOFF               ((uint32_t)CAN_IER_BOFIE)   /*!< Bus-off interrupt                */
#define CAN_IT_LAST_ERROR_CODE      ((uint32_t)CAN_IER_LECIE)   /*!< Last error code interrupt        */
#define CAN_IT_ERROR                ((uint32_t)CAN_IER_ERRIE)   /*!< Error Interrupt                  */
/**
  * @}
  */

/**
  * @}
  */

static void CAN_PeriClockControl(CAN_RegDef_t *pCANx, uint8_t State);
uint8_t CAN_Init(CAN_Handle_t *pCANHandle);
uint8_t CAN_DeInit(CAN_Handle_t *pCANHandle);

uint8_t CAN_RegisterCallback(CAN_Handle_t *pCANHandle, CAN_CallbackID_t CallbackID, void (* pCallback)(CAN_Handle_t *pCANHandle));
uint8_t CAN_UnRegisterCallback(CAN_Handle_t *pCANHandle, CAN_CallbackID_t CallbackID);
uint8_t CAN_ConfigFilter(CAN_Handle_t *pCANHandle, CAN_Filter_t *sFilterConfig);

uint8_t CAN_Start(CAN_Handle_t *pCANHandle);
uint8_t CAN_Stop(CAN_Handle_t *pCANHandle);
uint8_t CAN_RequestSleep(CAN_Handle_t *pCANHandle);
uint8_t CAN_WakeUpFromSleep(CAN_Handle_t *pCANHandle);
uint8_t CAN_AddTxMessageToMailBox(CAN_Handle_t *pCANHandle, CAN_TxHeader_t *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
uint8_t CAN_AbortTxRequest(CAN_Handle_t *pCANHandle, uint32_t TxMailboxes);

//Check số mailbox đang trống
uint32_t CAN_GetNumberTxMailboxFree(CAN_Handle_t *pCANHandle);
uint32_t CAN_IsTxMessageMailboxPending(CAN_Handle_t *pCANHandle, uint32_t TxMailboxes);
uint32_t CAN_GetTxTimestamp(CAN_Handle_t *pCANHandle, uint32_t TxMailbox);
uint8_t  CAN_GetRxMessage(CAN_Handle_t *pCANHandle, uint32_t RxFifo, CAN_RxHeader_t *pHeader, uint8_t aData[]/*, ...*/);

//Trả về số lượng mesage bị pending trong Receive FIFOx
uint32_t CAN_GetNumberRxPendingInFIFO(CAN_Handle_t *pCANHandle, uint32_t RxFifo);
uint8_t  CAN_EnableInterrupt(CAN_Handle_t *pCANHandle, uint32_t InterruptType);
uint8_t  CAN_DisableInterrupt(CAN_Handle_t *pCANHandle, uint32_t InterruptType);
void CAN_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void CAN_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void CAN_IRQHandling(CAN_Handle_t *pCANHandle);

uint8_t  CAN_GetState(CAN_Handle_t *pCANHandle);
uint32_t CAN_GetError(CAN_Handle_t *pCANHandle);

/* Callback functions */
void CAN_TxMailbox0CompleteCallback(CAN_Handle_t *pCANHandle);
void CAN_TxMailbox1CompleteCallback(CAN_Handle_t *pCANHandle);
void CAN_TxMailbox2CompleteCallback(CAN_Handle_t *pCANHandle);
void CAN_TxMailbox0AbortCallback(CAN_Handle_t *pCANHandle);
void CAN_TxMailbox1AbortCallback(CAN_Handle_t *pCANHandle);
void CAN_TxMailbox2AbortCallback(CAN_Handle_t *pCANHandle);
void CAN_RxFifo0MsgPendingCallback(CAN_Handle_t *pCANHandle);
void CAN_RxFifo0FullCallback(CAN_Handle_t *pCANHandle);
void CAN_RxFifo1MsgPendingCallback(CAN_Handle_t *pCANHandle);
void CAN_RxFifo1FullCallback(CAN_Handle_t *pCANHandle);
void CAN_SleepCallback(CAN_Handle_t *pCANHandle);
void CAN_WakeUpFromRxMsgCallback(CAN_Handle_t *pCANHandle);
void CAN_ErrorCallback(CAN_Handle_t *pCANHandle);





























#endif /* INC_STM32F1_CAN_H_ */
