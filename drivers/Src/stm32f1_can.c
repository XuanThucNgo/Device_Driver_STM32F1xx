/*
 * stm32f1_can.c
 *
 *  Created on: Oct 10, 2024
 *      Author: ADMIN
 */

#include "stm32f1_can.h"
#include <stddef.h>

static void CAN_PeriClockControl(CAN_RegDef_t *pCANx, uint8_t State)
{
  if(State == ENABLE)
  {
    if(pCANx == CAN1)
    {
      CAN1_PCLK_EN();
    }
    else
    {
      CAN2_PCLK_EN();
    }
  }
  else
  {
    if(pCANx == CAN1)
    {
      CAN1_PCLK_DI();
    }
    else
    {
      CAN2_PCLK_DI();
    }    
  }
}

uint8_t CAN_Init(CAN_Handle_t *pCANHandle)
{
  CAN_PeriClockControl(pCANHandle->pCANx, ENABLE);

  pCANHandle->RxFifo1MsgPendingCallback   = CAN_RxFifo1MsgPendingCallback;
  pCANHandle->RxFifo0FullCallback         = CAN_RxFifo0FullCallback;
  pCANHandle->RxFifo1MsgPendingCallback   = CAN_RxFifo1MsgPendingCallback;
  pCANHandle->RxFifo1FullCallback         = CAN_RxFifo1FullCallback;
  pCANHandle->TxMailbox0CompleteCallback  = CAN_TxMailbox0CompleteCallback;
  pCANHandle->TxMailbox1CompleteCallback  = CAN_TxMailbox1CompleteCallback;
  pCANHandle->TxMailbox2CompleteCallback  = CAN_TxMailbox2CompleteCallback;
  pCANHandle->TxMailbox0AbortCallback     = CAN_TxMailbox0AbortCallback;
  pCANHandle->TxMailbox1AbortCallback     = CAN_TxMailbox1AbortCallback;
  pCANHandle->TxMailbox2AbortCallback     = CAN_TxMailbox2AbortCallback;
  pCANHandle->SleepCallback               = CAN_SleepCallback;
  pCANHandle->WakeUpFromRxMsgCallback     = CAN_WakeUpFromRxMsgCallback;
  pCANHandle->ErrorCallback               = CAN_ErrorCallback;

  /* Request initalization */
  pCANHandle->pCANx->MCR |= (0x1UL << CAN_MCR_INRQ);

  /* Exit from sleep mode */
  pCANHandle->pCANx->MCR &= ~(0x1UL << CAN_MCR_SLEEP);

  /* Check exit from sleep mode */
  if(pCANHandle->pCANx->MSR & CAN_MSR_SLAK != 0U)
  {
    /* Update Error Code */
    pCANHandle->ErrorCode |= CAN_ERROR_TIMEOUT;

    /* Change CAN state */
    pCANHandle->State = CAN_STATE_ERROR;

    return 0;
  }

  /* Set time triggered communication mode */
  if(pCANHandle->CAN_Config.TimeTriggeredMode == ENABLE)
  {
    pCANHandle->pCANx->MCR |= CAN_MCR_TTCM;
  }
  else
  {
    pCANHandle->pCANx->MCR &= ~CAN_MCR_TTCM;   
  }

  /* Set automatic bus off management */
  if(pCANHandle->CAN_Config.AutoBusOff == ENABLE)
  {
    pCANHandle->pCANx->MCR |= CAN_MCR_ABOM;
  }
  else
  {
    pCANHandle->pCANx->MCR &= ~CAN_MCR_ABOM;
  }

  /* Set automatic retransmisstion */
  if(pCANHandle->CAN_Config.AutoRetransmission == ENABLE)
  {
    pCANHandle->pCANx->MCR |= CAN_MCR_NART;
  }
  else
  {
    pCANHandle->pCANx->MCR &= ~CAN_MCR_NART;
  }

  /* Set receive FIFO locked mode */
  if(pCANHandle->CAN_Config.ReceiveFifoLocked == ENABLE)
  {
    pCANHandle->pCANx->MCR |= CAN_MCR_RFLM;
  }
  else
  {
    pCANHandle->pCANx->MCR &= ~CAN_MCR_RFLM;
  }

  /* Set transmit FIFO Priority */
  if(pCANHandle->CAN_Config.TransmissFifoPriority == ENABLE)
  {
    pCANHandle->pCANx->MCR |= CAN_MCR_TXFP;
  }
  else
  {
    pCANHandle->pCANx->MCR &= ~CAN_MCR_TXFP;
  }

  /* Set bit timing register */
  pCANHandle->pCANx->BTR = (uint32_t)(pCANHandle->CAN_Config.Mode | pCANHandle->CAN_Config.SyncJumpWidth | pCANHandle->CAN_Config.TimeSeg1
                                    | pCANHandle->CAN_Config.TimeSeg2 | (pCANHandle->CAN_Config.Prescaler - 1U));
  /* Set error code */
  pCANHandle->ErrorCode = CAN_ERROR_NONE;

  /* Set CAN state */
  pCANHandle->State = CAN_STATE_READY;

  return 1;
}

uint8_t CAN_DeInit(CAN_Handle_t *pCANHandle)
{
  /* Stop the CAN module */
  (void)CAN_Stop(pCANHandle);

  /* Reset the CAN peripheral */
  pCANHandle->pCANx->MCR |= CAN_MCR_RESET;

  /* Set error code */
  pCANHandle->ErrorCode = CAN_ERROR_NONE;

  /* Set CAN state */
  pCANHandle->State = CAN_STATE_RESET;

  return 1;
}

/* Start the CAN module */
uint8_t CAN_Start(CAN_Handle_t *pCANHandle)
{
  if(pCANHandle->State == CAN_STATE_READY)
  {
    /* Change CAN peripheral state */
    pCANHandle->State == CAN_STATE_LISTENING;

    /* Request leave initialisation */
    pCANHandle->pCANx->MCR &= ~(0x1UL << CAN_MCR_INRQ_Pos);

    /* Wait the acknowledge */

    uint16_t Timeout = 10000;
    while ((pCANHandle->pCANx->MSR & CAN_MSR_INAK) != 0U)
    {
      if(Timeout == 0)
      {
        /* Update error code */
        pCANHandle->ErrorCode |= CAN_ERROR_TIMEOUT;

        /* Change CAN state */
        pCANHandle->State = CAN_STATE_ERROR;

        return 0;
      }
      Timeout --;
    }

    /* Reset the CAN ErrorCode */
    pCANHandle->ErrorCode = CAN_ERROR_NONE;

    return 1;
  }
  else
  {
    /* Update error code */
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_READY;

    return 0;
  }
}

/* Stop the CAN module and enable access to configuration registers */
uint8_t CAN_Stop(CAN_Handle_t *pCANHandle)
{
  if(pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Request initalization */
    pCANHandle->pCANx->MCR |= (0x1UL << CAN_MCR_INRQ);

    /* Wait the acknowledge */
    if((pCANHandle->pCANx->MSR & CAN_MSR_INAK) == 0U)
    {
      /* Update Error Code */
      pCANHandle->ErrorCode |= CAN_ERROR_TIMEOUT;

      /* Change CAN state */
      pCANHandle->State = CAN_STATE_ERROR;

      return 0;
    }

    /* Exit from sleep mode */
    pCANHandle->pCANx->MCR &= ~(0x1UL << CAN_MCR_SLEEP);

    /* Change CAN peripheral state*/
    pCANHandle->State = CAN_STATE_READY;

    return 1;
  }
  else
  {
    /* Update error code */
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_STARTED;

    return 0;
  }
}

uint8_t CAN_RegisterCallback(CAN_Handle_t *pCANHandle, CAN_CallbackID_t CallbackID, void (* pCallback)(CAN_Handle_t *pCANHandle))
{
  uint8_t Status = 1;
  if( pCANHandle->State == CAN_STATE_READY)
  {
    if(CallbackID == CAN_TX_MAILBOX0_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox0CompleteCallback = pCallback;
    }
    else if (CallbackID == CAN_TX_MAILBOX1_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox1CompleteCallback = pCallback;
    }
    else if (CallbackID == CAN_TX_MAILBOX2_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox2CompleteCallback = pCallback;
    }
    else if (CallbackID == CAN_TX_MAILBOX0_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox0AbortCallback = pCallback;
    }
    else if (CallbackID == CAN_TX_MAILBOX1_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox1AbortCallback = pCallback;
    }
    else if (CallbackID == CAN_TX_MAILBOX2_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox2AbortCallback = pCallback;
    }
    else if (CallbackID == CAN_RX_FIFO0_MSG_PENDING_CB_ID)
    {
      pCANHandle->RxFifo0MsgPendingCallback = pCallback;
    }
    else if (CallbackID == CAN_RX_FIFO1_MSG_PENDING_CB_ID)
    {
      pCANHandle->RxFifo1MsgPendingCallback = pCallback;
    }
    else if (CallbackID == CAN_RX_FIFO0_FULL_CB_ID)
    {
      pCANHandle->RxFifo0FullCallback = pCallback;
    }
    else if (CallbackID == CAN_RX_FIFO1_FULL_CB_ID)
    {
      pCANHandle->RxFifo1FullCallback = pCallback;
    }
    else if (CallbackID == CAN_SLEEP_CB_ID)
    {
      pCANHandle->SleepCallback = pCallback;
    }
    else if (CallbackID == CAN_WAKEUP_FROM_RX_MSG_CB_ID)
    {
      pCANHandle->WakeUpFromRxMsgCallback = pCallback;
    }
    else if (CallbackID == CAN_ERROR_CB_ID)
    {
      pCANHandle->ErrorCallback = pCallback;
    }
  }
  else
  {
    Status = 0;
  }

  return Status;
}

uint8_t CAN_UnRegisterCallback(CAN_Handle_t *pCANHandle, CAN_CallbackID_t CallbackID)
{
  uint8_t Status = 1;
  if( pCANHandle->State == CAN_STATE_READY)
  {
    if(CallbackID == CAN_TX_MAILBOX0_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox0CompleteCallback = NULL;
    }
    else if (CallbackID == CAN_TX_MAILBOX1_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox1CompleteCallback = NULL;
    }
    else if (CallbackID == CAN_TX_MAILBOX2_COMPLETE_CB_ID)
    {
      pCANHandle->TxMailbox2CompleteCallback = NULL;
    }
    else if (CallbackID == CAN_TX_MAILBOX0_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox0AbortCallback = NULL;
    }
    else if (CallbackID == CAN_TX_MAILBOX1_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox1AbortCallback = NULL;
    }
    else if (CallbackID == CAN_TX_MAILBOX2_ABORT_CB_ID)
    {
      pCANHandle->TxMailbox2AbortCallback = NULL;
    }
    else if (CallbackID == CAN_RX_FIFO0_MSG_PENDING_CB_ID)
    {
      pCANHandle->RxFifo0MsgPendingCallback = NULL;
    }
    else if (CallbackID == CAN_RX_FIFO1_MSG_PENDING_CB_ID)
    {
      pCANHandle->RxFifo1MsgPendingCallback = NULL;
    }
    else if (CallbackID == CAN_RX_FIFO0_FULL_CB_ID)
    {
      pCANHandle->RxFifo0FullCallback = NULL;
    }
    else if (CallbackID == CAN_RX_FIFO1_FULL_CB_ID)
    {
      pCANHandle->RxFifo1FullCallback = NULL;
    }
    else if (CallbackID == CAN_SLEEP_CB_ID)
    {
      pCANHandle->SleepCallback = NULL;
    }
    else if (CallbackID == CAN_WAKEUP_FROM_RX_MSG_CB_ID)
    {
      pCANHandle->WakeUpFromRxMsgCallback = NULL;
    }
    else if (CallbackID == CAN_ERROR_CB_ID)
    {
      pCANHandle->ErrorCallback = NULL;
    }
  }
  else
  {
    Status = 0;
  }

  return Status;  
}

uint8_t CAN_ConfigFilter(CAN_Handle_t *pCANHandle, CAN_Filter_t *sFilterConfig)
{
  uint32_t FilterPos;
  CAN_RegDef_t *CAN_Ip = pCANHandle->pCANx;

  /* Initialisation mode for filter bank */
  pCANHandle->pCANx->FMR |= CAN_FMR_FINIT;

  if(pCANHandle->pCANx == CAN2)
  {
    /* Select the start filter number of CAN2 slave */
    /* When CAN2SB[5:0] is set to 0, no filters are assigned to CAN1 */
    pCANHandle->pCANx->FMR &= ~CAN_FMR_CAN2SB;
    /* What number bank filter CAN2 use */
    pCANHandle->pCANx->FMR |= (sFilterConfig->SlaveStartFilterBank << CAN_FMR_CAN2SB_Pos);
  }

  /* Convert filter number into bit position */
  /* Filter bank 0 - 27 */
  FilterPos = (uint32_t)1 << (sFilterConfig->FilterBank & 0x1FU);
  /* Filter Deactivation */
  pCANHandle->pCANx->FA1R &= ~(FilterPos);

  /* Filter Scale */
  if(sFilterConfig->FilterScale == CAN_FILTERSCALE_16BIT)
  {
    /* 16-bit scale for the filter */
    CAN_Ip->FS1R &= ~(FilterPos);
    /* First 16-bit identifier and First 16-bit mask */
    /* Or First 16-bit identifier and Second 16-bit identifier */
    CAN_Ip->sFilterRegister[sFilterConfig->FilterBank].FR1 = ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIDLow << 16U) |
                                                              (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIDLow));

    /* Second 16-bit identifier and Second 16-bit mask */
    /* Or Third 16-bit identifier and Fourth 16-bit identifier */
    CAN_Ip->sFilterRegister[sFilterConfig->FilterBank].FR2 = ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIDHigh << 16U) |
                                                              (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIDHigh));
  }

  if(sFilterConfig->FilterScale == CAN_FILTERSCALE_32BIT)
  {
    /* 32-bit scale for the filter */ 
    CAN_Ip->FS1R |= (FilterPos);
    /* 32-bit identifier or First 32-bit identifier */
    CAN_Ip->sFilterRegister[sFilterConfig->FilterBank].FR1 = ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterIDHigh << 16U) |
                                                              (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIDLow));

    /* 32-bit mask or Second 32-bit identifier */ 
    CAN_Ip->sFilterRegister[sFilterConfig->FilterBank].FR1 = ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIDHigh << 16U) |
                                                              (0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIDLow));  
  }

  /* Filter Mode */
  if(sFilterConfig->FilterMode == CAN_FILTERMODE_IDMASK)
  {
    /* Id/Mask mode for the filter*/
    CAN_Ip->FM1R &= ~(FilterPos);
  }
  else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FILTERMODE_IDLIST */
  {
    /* Identifier list mode for the filter*/
    CAN_Ip->FM1R |= (FilterPos);
  }

  /* Filter FIFO assignment */
  if(sFilterConfig->FilterFIFONumber == CAN_FILTER_FIFO0)
  {
    /* FIFO 0 assignation for the filter */
    CAN_Ip->FFA1R &= ~(FilterPos);
  }
  else
  {
    /* FIFO 1 assignation for the filter */
    CAN_Ip->FFA1R |= (FilterPos);     
  }

  /* Filter activation */
  if(sFilterConfig->FilterActivation == CAN_FILTER_ENABLE)
  {
    CAN_Ip->FA1R |= FilterPos;
  }

  /* Leave the initialisation mode for the filter */
  CAN_Ip->FFA1R &= ~CAN_FMR_FINIT;

  /* Return function status */
  return 1;

}

/* Request the sleep mode (low power) entry */
uint8_t CAN_RequestSleep(CAN_Handle_t *pCANHandle)
{
  uint8_t State = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Request Sleep Mode */
    pCANHandle->pCANx->MCR |= CAN_MCR_SLEEP;

    State = 1;
  }
  else
  {
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_INITIALIZED;

    State = 0;
  } 

  return State;
}
uint8_t CAN_WakeUpFromSleep(CAN_Handle_t *pCANHandle)
{
  uint8_t State = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Request Wakeup Mode */
    pCANHandle->pCANx->MCR |= CAN_MCR_SLEEP;
    /* Wait sleep mode is exited */
    for (uint16_t i = 0; i < 10000; i++)
    {
      State = 1;
    }
  }
  else
  {
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_INITIALIZED;

    State = 0;
  } 

  return State; 
}

/* Add message vào Tx mailbox đầu tiên và active yêu cầu tương ứng */
/* pTxMailbox: CAN Tx MailBox */
uint8_t CAN_AddTxMessageToMailBox(CAN_Handle_t *pCANHandle, CAN_TxHeader_t *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t TSR_Register = pCANHandle->pCANx->TSR;
  uint32_t TransmitMailbox;
  uint8_t Status = 0;

  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Check that all the Tx mailboxes are not full */
    if(((TSR_Register & CAN_TSR_TME0) != 0) || 
       ((TSR_Register & CAN_TSR_TME1) != 0) || 
       ((TSR_Register & CAN_TSR_TME2) != 0))
    {
      /* Select an empty transmit mailbox */
      TransmitMailbox = (TSR_Register & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
      
      /* Check transmit mailbox value */
      if(TransmitMailbox > 2U)
      {
        /* Update error code */
        pCANHandle->ErrorCode |= CAN_ERROR_INTERNAL;
        return 0;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << TransmitMailbox;

      /* Set up the Id */
      /* CAN TX mailbox identifier register (CAN_TIxR) (x=0..2) */
      if(pHeader->IDE == CAN_ID_STD)
      {
        pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) 
                                                             | pHeader->RTR);
      }
      else
      {
        pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TIR = ((pHeader->ExtId << CAN_TI0R_EXID_Pos)
                                                             | pHeader->IDE
                                                             | pHeader->RTR);                                                   
      }

      /* Set up DLC */
      pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TDTR = (pHeader->DLC);

      /* Set up Transmit Global Time mode */
      /* CAN mailbox data length control and time stamp register (CAN_TDTxR) (x=0..2) */
      /* TGT: Transmit global time */
      if(pHeader->TransmissGlobalTime == ENABLE)
      {
        pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TDTR |= CAN_TDT0R_TGT;
      }

      /* Set up trường data */
      /* CAN mailbox data high register (CAN_TDHxR) (x=0..2) */
      pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TDHR = ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) | \
                                                            ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) | \
                                                            ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) | \
                                                            ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos);
      /* CAN mailbox data low register (CAN_TDLxR) (x=0..2) */
      pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TDLR = ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) | \
                                                            ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) | \
                                                            ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) | \
                                                            ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos);
      /* Request transmisstion */
      /* CAN TX mailbox identifier register (CAN_TIxR) (x=0..2) */
      /* TXRQ: Transmit mailbox request */
      pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TIR |= CAN_TI0R_TXRQ;
      Status = 1;   
    }
    else
    {
      pCANHandle->ErrorCode |= CAN_ERROR_PARAM;
      Status = 0;
    }
  }
  else
  {
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_INITIALIZED;
    Status = 0;
  }
  return Status;
}
uint8_t CAN_AbortTxRequest(CAN_Handle_t *pCANHandle, uint32_t TxMailboxes)
{
  /* TSR: CAN transmit status register (CAN_TSR) */
  uint8_t Status = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* ABRQx: Abort request for mailbox x */  /* x = 0, 1, 2*/
    /* Check Tx Mailbox 0 */
    if((TxMailboxes & CAN_TX_MAILBOX0) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 0 */
      pCANHandle->pCANx->TSR |= CAN_TSR_ABRQ0;
    }

    /* Check Tx Mailbox 1 */
    if((TxMailboxes & CAN_TX_MAILBOX1) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 1 */
      pCANHandle->pCANx->TSR |= CAN_TSR_ABRQ1;
    }

    /* Check Tx Mailbox 2 */
    if((TxMailboxes & CAN_TX_MAILBOX2) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 2 */
      pCANHandle->pCANx->TSR |= CAN_TSR_ABRQ2;
    }

    Status = 1;
  }
  else
  {
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_INITIALIZED;
    Status = 0;
  }
  return Status;
}

uint32_t CAN_GetNumberTxMailboxFree(CAN_Handle_t *pCANHandle)
{
  /* TSR: CAN transmit status register (CAN_TSR) */
  uint32_t FreeNumber = 0U;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* TMEx: Transmit mailbox x empty */    /* x = 0, 1, 2*/
    /* Check Tx Mailbox 0 status */
    if(pCANHandle->pCANx->TSR & CAN_TSR_TME0 != 0)
    {
      FreeNumber++;
    }

    /* Check Tx Mailbox 1 status */
    if(pCANHandle->pCANx->TSR & CAN_TSR_TME1 != 0)
    {
      FreeNumber++;
    }

    /* Check Tx Mailbox 2 status */
    if(pCANHandle->pCANx->TSR & CAN_TSR_TME2 != 0)
    {
      FreeNumber++;
    }
  }
  return FreeNumber;
}

uint32_t CAN_IsTxMessageMailboxPending(CAN_Handle_t *pCANHandle, uint32_t TxMailboxes)
{
  uint8_t Status = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Check pending transmission request on the selected Tx Mailboxes */
    if((pCANHandle->pCANx->TSR & (TxMailboxes << CAN_TSR_TME0_Pos)) != (TxMailboxes << CAN_TSR_TME0_Pos))
    {
      Status = 1U;
    }
  }  
  return Status; 
}
/* Đảo ngược thứ tự bit của giá trị đã cho */
//__STATIC_FORCEINLINE 
uint32_t Revert_Bits(uint32_t Value)
{
  uint32_t Result;
  uint32_t Number_Bit = 31;
  Result = Value;

  for(Value = Value >> 1U; Value != 0U; Value = Value >> 1U)
  {
    Result = Result << 1U;
    Result |= Value & 1U;
    Number_Bit--;
  }
  Result = Result << Number_Bit;

  return Result;
}

/* Return timestamp of Tx message đã gửi, if the time trigger communication mode is enabled */
uint32_t CAN_GetTxTimestamp(CAN_Handle_t *pCANHandle, uint32_t TxMailbox)
{
  uint32_t TimesTamp = 0U;
  uint32_t TransmitMailbox;

  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    TransmitMailbox = (uint8_t)__builtin_clz(Revert_Bits(TxMailbox));

    /* Get timestamp */
    TimesTamp = (pCANHandle->pCANx->sTxMailBox[TransmitMailbox].TDTR & CAN_TDT0R_TIME) >> CAN_TDT0R_TIME_Pos;
  }
  return TimesTamp;
}
uint8_t  CAN_GetRxMessage(CAN_Handle_t *pCANHandle, uint32_t RxFifo, CAN_RxHeader_t *pHeader, uint8_t aData[])
{
  uint8_t Status = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Check the RX FIFO */
    if(RxFifo == CAN_RX_FIFO0)  
    {
      /* Check Rx FIFO không empty */
      if((pCANHandle->pCANx->RF0R & CAN_RF0R_FMP0) == 0U) 
      {
        /* Update error code */
        pCANHandle->ErrorCode |= CAN_ERROR_PARAM;
        return 0;
      }    
    }
    else
    {
      /* Check Rx FIF1 không empty */
      if((pCANHandle->pCANx->RF1R & CAN_RF1R_FMP1) == 0U) 
      {
        /* Update error code */
        pCANHandle->ErrorCode |= CAN_ERROR_PARAM;
        return 0;
      }       
    }

    /* Get header */
    pHeader->IDE = CAN_RI0R_IDE & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RIR;

    if(pHeader->IDE == CAN_ID_STD)
    {
      pHeader->StdId = (CAN_RI0R_STID & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos;
    }
    else
    {
      pHeader->ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_EXID_Pos;
    }

    pHeader->RTR              = (CAN_RI0R_RTR   & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RIR); 
    pHeader->DLC              = (CAN_RDT0R_DLC  & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    pHeader->FilterMatchIndex = (CAN_RDT0R_FMI  & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    pHeader->TimeStamp        = (CAN_RDT0R_TIME & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Get data */
    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos); 
    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos); 
    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos); 
    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos); 
    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDH0R_DATA4_Pos); 
    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDH0R_DATA5_Pos); 
    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & pCANHandle->pCANx->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDH0R_DATA7_Pos); 

    /* Release FIFO */
    if(RxFifo == CAN_RX_FIFO0)
    {
      /* Release FIFO 0 */
      pCANHandle->pCANx->RF0R |= CAN_RF0R_RFOM0;
    } 
    else
    {
      /* Release FIFO 1 */
      pCANHandle->pCANx->RF1R |= CAN_RF1R_RFOM1;
    }

    /* Return function status */
    Status = 1;
  }
  else
  {
    pCANHandle->ErrorCode |= CAN_ERROR_NOT_INITIALIZED;
    Status = 0;
  }

  return Status;
}

/* Trả về số lượng message bị pending trong Received FIFOx */
/* RxFifo CAN Receive FIFO number */
uint32_t CAN_GetNumberRxPendingInFIFO(CAN_Handle_t *pCANHandle, uint32_t RxFifo)
{
  uint32_t FillLever = 0U;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    if(RxFifo == CAN_RX_FIFO0)
    {
      FillLever = pCANHandle->pCANx->RF0R & CAN_RF0R_FMP0;
    }
    else  //RxFifo == CAN_RX_FIFO1
    {
      FillLever = pCANHandle->pCANx->RF1R & CAN_RF1R_FMP1;
    }
  }
  /* Return Rx FIFO fill level */
  return FillLever;
}

uint8_t  CAN_EnableInterrupt(CAN_Handle_t *pCANHandle, uint32_t InterruptType)
{
  uint8_t Status = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Enable the selected interrupt */
    pCANHandle->pCANx->IER |= InterruptType;
    Status = 1;
  }
  else
  {
    Status = 0;
  }

  return Status;
}
uint8_t  CAN_DisableInterrupt(CAN_Handle_t *pCANHandle, uint32_t InterruptType)
{
  uint8_t Status = 0;
  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Disable the selected interrupt */
    pCANHandle->pCANx->IER &= ~InterruptType;
    Status = 1;
  }
  else
  {
    Status = 0;
  }

  return Status;
}

void CAN_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void CAN_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4; 

    //*(NVIC_PR_BASEADDR + ipr) &= ~(0xF << (8 * irq + 4));

    *(NVIC_PR_BASEADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}
void CAN_IRQHandling(CAN_Handle_t *pCANHandle)
{
  uint32_t ErrorCode = CAN_ERROR_NONE;
  uint32_t Interrupts = pCANHandle->pCANx->IER;
  uint32_t MSRflags  = pCANHandle->pCANx->MSR;
  uint32_t TSRflags  = pCANHandle->pCANx->TSR;
  uint32_t RF0R_flags = pCANHandle->pCANx->RF0R;
  uint32_t RF1R_flags = pCANHandle->pCANx->RF1R;
  uint32_t ESR_flags  = pCANHandle->pCANx->ESR;

  /* Transmit Mailbox empty interrupt */
  if((Interrupts & CAN_IT_TX_MAILBOX_EMPTY) != 0U)
  {
    /* Transmit Mailbox 0 */
    if((TSRflags & CAN_TSR_RQCP0) != 0U)
    {
      /* Clear the Transmission Complete flag */
      pCANHandle->pCANx->TSR = (1U << (CAN_FLAG_RQCP0 & 0x000000FFU));

      /* Transmit thành công */
      if((TSRflags & CAN_TSR_ABRQ0) != 0U)
      {
        /* Transmission Mailbox 0 complete callback */
        pCANHandle->TxMailbox0CompleteCallback(pCANHandle);
      }
      else
      {
        /* Arbitration lost for Mailbox0 */
        if((TSRflags & CAN_TSR_ALST0) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_ALST0;
        }
        /* Transmission error of Mailbox0 */
        else if ((TSRflags & CAN_TSR_TERR0) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_TERR0;
        }
        else
        {
          /* Transmission Mailbox 0 abort callback */
          pCANHandle->TxMailbox0AbortCallback(pCANHandle);
        }
      }
    }

    /* Transmit Mailbox 1 */
    if((TSRflags & CAN_TSR_RQCP1) != 0U)
    {
      /* Clear the Transmission Complete flag */
      pCANHandle->pCANx->TSR = (1U << (CAN_FLAG_RQCP1 & 0x000000FFU));

      /* Transmit thành công */
      if((TSRflags & CAN_TSR_ABRQ1) != 0U)
      {
        /* Transmission Mailbox 1 complete callback */
        pCANHandle->TxMailbox1CompleteCallback(pCANHandle);
      }
      else
      {
        /* Arbitration lost for Mailbox1 */
        if((TSRflags & CAN_TSR_ALST1) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_ALST1;
        }
        /* Transmission error of Mailbox1 */
        else if ((TSRflags & CAN_TSR_TERR1) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_TERR1;
        }
        else
        {
          /* Transmission Mailbox 1 abort callback */
          pCANHandle->TxMailbox1AbortCallback(pCANHandle);
        }
      }
    }

    /* Transmit Mailbox 2 */
    if((TSRflags & CAN_TSR_RQCP2) != 0U)
    {
      /* Clear the Transmission Complete flag */
      pCANHandle->pCANx->TSR = (1U << (CAN_FLAG_RQCP2 & 0x000000FFU));

      /* Transmit thành công */
      if((TSRflags & CAN_TSR_ABRQ2) != 0U)
      {
        /* Transmission Mailbox 2 complete callback */
        pCANHandle->TxMailbox2CompleteCallback(pCANHandle);
      }
      else
      {
        /* Arbitration lost for Mailbox2 */
        if((TSRflags & CAN_TSR_ALST2) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_ALST2;
        }
        /* Transmission error of Mailbox2 */
        else if ((TSRflags & CAN_TSR_TERR2) != 0U)
        {
          ErrorCode |= CAN_ERROR_TX_TERR2;
        }
        else
        {
          /* Transmission Mailbox 1 abort callback */
          pCANHandle->TxMailbox1AbortCallback(pCANHandle);
        }
      }
    }
  }

  /* Receive FIFO 0 overrun interrupt */
  if((Interrupts & CAN_IT_RX_FIFO0_OVERRUN) != 0U)
  {
    if((RF0R_flags & CAN_RF0R_FOVR0) != 0U)
    {
      /* Set CAN error code to Rx Fifo 0 overrun error */
      ErrorCode |= CAN_ERROR_RX_FOV0;

      /* Clear FIFO0 Overrun Flag */
      pCANHandle->pCANx->RF0R = (1U << (CAN_FLAG_FOV0 & 0x000000FFU));
    }
  }

  /* Receive FIFO 0 full interrupt */
  if((Interrupts & CAN_IT_RX_FIFO0_FULL) != 0U)
  {
    if((RF0R_flags & CAN_RF0R_FULL0) != 0U)
    {
      /* Clear FIFO0 full Flag */
      pCANHandle->pCANx->RF0R = (1U << (CAN_FLAG_FF0 & 0x000000FFU));

      /* Receive FIFO 0 full Callback */
      pCANHandle->RxFifo0FullCallback(pCANHandle);     
    }
  }

  /* Receive FIFO 0 message pending interrupt */
  if((Interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
  {
    /* Check if message still pending */
    if((pCANHandle->pCANx->RF0R & CAN_RF0R_FMP0) != 0U)
    {
      /* Receive FIFO 0 message pending Callback */
      pCANHandle->RxFifo0MsgPendingCallback(pCANHandle);
    }
  }

/* Receive FIFO 1 overrun interrupt */
  if((Interrupts & CAN_IT_RX_FIFO1_OVERRUN) != 0U)
  {
    if((RF1R_flags & CAN_RF1R_FOVR1) != 0U)
    {
      /* Set CAN error code to Rx Fifo 1 overrun error */
      ErrorCode |= CAN_ERROR_RX_FOV1;

      /* Clear FIFO1 Overrun Flag */
      pCANHandle->pCANx->RF1R = (1U << (CAN_FLAG_FOV1 & 0x000000FFU));
    }
  }

  /* Receive FIFO 1 full interrupt */
  if((Interrupts & CAN_IT_RX_FIFO1_FULL) != 0U)
  {
    if((RF1R_flags & CAN_RF1R_FULL1) != 0U)
    {
      /* Clear FIFO1 full Flag */
      pCANHandle->pCANx->RF1R = (1U << (CAN_FLAG_FF1 & 0x000000FFU));

      /* Receive FIFO 1 full Callback */
      pCANHandle->RxFifo1FullCallback(pCANHandle);     
    }
  }

  /* Receive FIFO 1 message pending interrupt */
  if((Interrupts & CAN_IT_RX_FIFO1_MSG_PENDING) != 0U)
  {
    /* Check if message still pending */
    if((pCANHandle->pCANx->RF1R & CAN_RF1R_FMP1) != 0U)
    {
      /* Receive FIFO 1 message pending Callback */
      pCANHandle->RxFifo1MsgPendingCallback(pCANHandle);
    }
  }

  /* Sleep interrupt */
  if((Interrupts & CAN_IT_SLEEP_ACK) != 0U)
  {
    /* Check if message still pending */
    if((MSRflags & CAN_MSR_SLAKI) != 0U)
    {
      /* Clear Sleep interrupt Flag */
      pCANHandle->pCANx->MSR = (1U << (CAN_FLAG_SLAKI & 0x000000FFU));

      /* Sleep Callback */
      pCANHandle->SleepCallback(pCANHandle);
    }
  }

  /* Wakeup interrupt */
  if((Interrupts & CAN_IT_WAKEUP) != 0U)
  {
    /* Check if message still pending */
    if((MSRflags & CAN_MSR_WKUI) != 0U)
    {
      /* Clear Wakeup interrupt Flag */
      pCANHandle->pCANx->MSR = (1U << (CAN_FLAG_WKU & 0x000000FFU));

      /* Wakeup Callback */
      pCANHandle->WakeUpFromRxMsgCallback(pCANHandle);
    }
  }

  /* Error interrupts */
  if((Interrupts & CAN_IT_ERROR) != 0U)
  {
    if((MSRflags & CAN_MSR_ERRI) != 0U)
    {
      /* Check Error Warning Flag */
      if((((Interrupts & CAN_IT_ERROR_WARNING) != 0U) &&
         (ESR_flags & CAN_ESR_EWGF) != 0U));
      {
        /* Set CAN error code to Error Warning */
        ErrorCode |= CAN_ERROR_EWG;
      }

      /* Check Error Passive Flag */
      if((((Interrupts & CAN_IT_ERROR_PASSIVE) != 0U) &&
         (ESR_flags & CAN_ESR_EPVF) != 0U));
      {
        /* Set CAN error code to Error Passive */
        ErrorCode |= CAN_ERROR_EPV;
      }

      /* Check Bus-off Flag */
      if((((Interrupts & CAN_IT_BUSOFF) != 0U) &&
         (ESR_flags & CAN_ESR_BOFF) != 0U));
      {
        /* Set CAN error code to Bus-off */
        ErrorCode |= CAN_ERROR_BOF;
      }

      /* Check Last Error Code Flag */
      if((((Interrupts & CAN_IT_LAST_ERROR_CODE) != 0U) &&
         (ESR_flags & CAN_ESR_LEC) != 0U));
      {
        switch (ESR_flags & CAN_ESR_LEC)
        {
          case (CAN_ESR_LEC_0):
            /* Set CAN error code to Stuff error */
            ErrorCode |= CAN_ERROR_STF;
            break;
          case (CAN_ESR_LEC_1):
            /* Set CAN error code to Form error */
            ErrorCode |= CAN_ERROR_FOR;
            break;
          case (CAN_ESR_LEC_0 | CAN_ESR_LEC_1):
            /* Set CAN error code to Acknowledgement error */
            ErrorCode |= CAN_ERROR_ACK;
            break;
          case (CAN_ESR_LEC_2):
            /* Set CAN error code to Bit recessive error */
            ErrorCode |= CAN_ERROR_BR;
            break;
          case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
            /* Set CAN error code to Bit Dominant error */
            ErrorCode |= CAN_ERROR_BD;
            break;  
          case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
            /* Set CAN error code to CRC error */
            ErrorCode |= CAN_ERROR_CRC;
            break;       
          default:
            break;
        }

        /* Clear Last error code Flag */
        pCANHandle->pCANx->ESR &= ~CAN_ESR_LEC;
      }
    }
    /* Clear ERRI Flag */
    pCANHandle->pCANx->MSR = (1U << (CAN_FLAG_ERRI & 0x000000FFU));
  }

  /* Call the Error call Back in case of Errors */
  if(ErrorCode != CAN_ERROR_NONE)
  {
    /* Update error code in handle */
    pCANHandle->ErrorCode |= ErrorCode;

    /* Call Error callback function */
    pCANHandle->ErrorCallback(pCANHandle);
  }

}

uint8_t  CAN_GetState(CAN_Handle_t *pCANHandle)
{
  CAN_State_t State = pCANHandle->State;

  if(pCANHandle->State == CAN_STATE_READY || pCANHandle->State == CAN_STATE_LISTENING)
  {
    /* Check sleep mode acknowladge flag */
    if((pCANHandle->pCANx->MSR & CAN_MSR_SLAK) != 0U)
    {
      /* Sleep mode is active */
      State = CAN_STATE_SLEEP_ACTIVE;
    }
    /* Check sleep mode request flag */
    else if ((pCANHandle->pCANx->MCR & CAN_MCR_SLEEP) != 0U)
    {
      /* Sleep mode request is pending */
      State = CAN_STATE_SLEEP_PENDING;
    }
    else
    {
      /*KHONG CO GI*/
    }
  }

  /* Return CAN state */
  return State;
}
uint32_t CAN_GetError(CAN_Handle_t *pCANHandle)
{
  return pCANHandle->ErrorCode;
}

void CAN_TxMailbox0CompleteCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_TxMailbox1CompleteCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_TxMailbox2CompleteCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_TxMailbox0AbortCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_TxMailbox1AbortCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_TxMailbox2AbortCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_RxFifo0MsgPendingCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_RxFifo0FullCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_RxFifo1MsgPendingCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_RxFifo1FullCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_SleepCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_WakeUpFromRxMsgCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}
void CAN_ErrorCallback(CAN_Handle_t *pCANHandle)
{
  (void)pCANHandle;
}




