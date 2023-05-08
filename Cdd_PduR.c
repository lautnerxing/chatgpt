/**********************************************************************************************************************
 * COPYRIGHT
 * -------------------------------------------------------------------------------------------------------------------
 * \verbatim
 * Copyright (c) 2021 LinearX Technology Co.Ltd.                                                  All rights reserved.
 * 
 *                 This software is copyright protected and proprietary to LinearX Technology Co.Ltd.
 *                 LinearX Technology Co.Ltd grants to you only those rights as set out in the license conditions.
 *                 All other rights remain with LinearX Technology Co.Ltd.
 * \endverbatim
 * -------------------------------------------------------------------------------------------------------------------
 * LICENSE
 * -------------------------------------------------------------------------------------------------------------------
 *                 \Module: Cdd
 *                \Package: Sip-Tc39x-D00
 *                   \Type: Static
 *               \Customer: Smart 
 *            \Expiry Date:
 *                \Channel: Autosar\Tc39x
 *          \License Scope: The usage is rescricted to Order xxxx
 * 
 * -------------------------------------------------------------------------------------------------------------------
 * FILE DESCRIPTION
 * -------------------------------------------------------------------------------------------------------------------
 *                   \file: Cdd_PduR.c
 *                 \Author: Jxin
 *        \Generation Time: 2022-11-16 13:52:14
 *                \Project: Smart
 *               \Delivery: Order xxxx
 *           \Tool Version: 
 *                  \brief: static code implementation for Cdd module.
 * *******************************************************************************************************************/ 
/*********************************************************************************************************************
 * REVISION HISTORY
 * -------------------------------------------------------------------------------------------------------------------
 *  Version 0.0.1       Jxin        Initial Version
 *  Version 1.0.0       Jxin        Add Ring Buffer Initialization 
 *  Version 1.1.0       Jxin        Realize Can Message Forwarding To Soc
 *  Version 1.1.1       Jxin        Realize Flexray Message Forwarding To Soc
 *  Version 1.1.2       Jxin        Realize Cdd_PduR_TriggerTransmit
 *  Version 1.2.0       Jxin        Run QAC and Modified Code
 * -------------------------------------------------------------------------------------------------------------------
 * Refer to the module's header file.
 * -------------------------------------------------------------------------------------------------------------------
 * FILE VERSION
 * -------------------------------------------------------------------------------------------------------------------
 * Refer to the VERSION CHECK below.
**********************************************************************************************************************/
/**********************************************************************************************************************
 ! BETA VERSION                                                                                                       !
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 ! This version of Hostman and/or the related Basic Software Package is BETA software.               !
 ! BETA Software is basically operable, but not sufficiently tested, verified and/or qualified for use in series      !
 ! production and/or in vehicles operating on public or non-public roads.                                             !
 ! In particular, without limitation, BETA Software may cause unpredictable ECU behavior, may not provide all         !
 ! functions necessary for use in series production and/or may not comply with quality requirements which are         !
 ! necessary according to the state of the art. BETA Software must not be used in series production.                  !
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**********************************************************************************************************************/

/*********************************************************************************************************************
 * INCLUDES
**********************************************************************************************************************/
#include "Cdd_PduR.h"
#include "Gateway_Mapping.h"
#include "ring_buffer.h"
#include "App_if.h"
#include "Os_UserLock.h"

/*********************************************************************************************************************
 * PUBLIC DATA STORAGE
**********************************************************************************************************************/
# define CDD_PDUR_START_SEC_CODE
/*lint -save -esym(961, 19.1) */
# include "MemMap.h"    /* PRQA S 5087 */       /* MD_MSR_MemMap */
/*lint -restore */


/* public count for Seqnum */
uint16 count_Can = 0U;
uint16 count_Flex = 0U;
boolean CheckFlag = FALSE;

static uint32 Cdd_PduR_SpinlockGourp[7U] = {BUFFERUNLOCKED};

/**********************************************************************************************************************
  Tx_RB_Init
**********************************************************************************************************************/
/*! \brief         initialize the base ring buffer for ready receive Rx-PUD from PduR.
    \return        none
    \pre           The TASK is initialized and active.
    \context       The function can be called in interrupt and on task level and should not to interrupted by another
                   Cdd_PduR_RxIndication call for the same RxPduId.
    \note          The function is called by the PduR.
**********************************************************************************************************************/
void Tx_RB_Init(void)
{
  for (uint8 i=0U; i<7U; i++)
  {
    RB_Init(TxRB_List_Can[i].rb_handle, TxRB_List_Can[i].buffer_addr, TxRB_List_Can[i].buffer_size);
  }
  /* initialize the ring buffer of FlexRay */
  RB_Init(&TxRB_Flex, TxBuffer_Flex, TxBufferSize_Flex);
}

/**********************************************************************************************************************
  Cdd_PduR_RxIndication
**********************************************************************************************************************/
/*! \brief         The function is called to indicate the complete reception of a RX I-PDU.
    \param[in]     RxPduId      id of the IF CddPduRUpperLayerRxPdu.
    \param[in]     PduInfoPtr   Payload information of the received I-PDU (pointer to data and data length).
    \return        none
    \pre           The Cdd_PDUR is initialized and active.
    \context       The function can be called in interrupt and on task level and should not to interrupted by another
                   Cdd_PduR_RxIndication call for the same RxPduId.
    \note          The function is called by the PduR.
**********************************************************************************************************************/
FUNC(void, CDD_PDUR_CODE) Cdd_PduR_RxIndication(PduIdType RxPduId, P2CONST(PduInfoType, AUTOMATIC, CDD_PDUR_APPL_DATA) PduInfoPtr)
{
  if (PduInfoPtr == NULL_PTR){
    // error "invalid PduInfoPtr"
  }
  else{
    if (RxPduId < CAN_NUM_OF_RX_SYSTEM_ELEMENTS){
      CanElement * CanElementTx = RxElement_map_table[RxPduId].CanBufferPtr;
      if (CanElementTx != NULL){
        if (PduInfoPtr->SduLength != CanElementTx->CanDlc){
          CtCdUartTrace_printf("\r\n RxPduId id = %d \r\n", RxPduId);
          CtCdUartTrace_printf("\r\n CAN DLC error, SduLength = %d \r\n", PduInfoPtr->SduLength);
        }
        uint16 datalength_can = CanElementTx->CanDlc;
        memcpy(CanElementTx->payload, PduInfoPtr->SduDataPtr, datalength_can);
        RxElement_map_table[RxPduId].isReceived = TRUE;
        switch(RxElement_map_table[RxPduId].rb_number)
        {
          /* includes Asensing, ChassisCAN1, Private_CANFD6 */
          case RB_NUMBER_NOT_GROUPED:
            if (RB_Get_FreeSize(&TxRB_Can_NotGroup)){
              // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
              RB_Write_String(&TxRB_Can_NotGroup, (uint8*)&CanElementTx, PtrLength_Tc39x);
            }
            break;

          /* SafetyCANFD1 Group1 */
          case RB_NUMBER_GROUP1:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP1)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_FSRLSafetyCANFD1Frame37_oSafetyCANFD1_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[0U]);
                FLAG_GROUP1 = TRUE;
                FlagMsgTable[0U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[0U]);
              }
            }
            break;

          /* SafetyCANFD1 Group2 */
          case RB_NUMBER_GROUP2:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP2)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_FSRRSafetyCANFD1Frame37_oSafetyCANFD1_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[1U]);
                FLAG_GROUP2 = TRUE;
                FlagMsgTable[1U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[1U]);
              }
            }
            break;

          /* SafetyCANFD2 Group3 */
          case RB_NUMBER_GROUP3:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP3)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_FLRSafetyCANFD2Frame64_oSafetyCANFD2_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[2U]);
                FLAG_GROUP3 = TRUE;
                FlagMsgTable[2U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[2U]);
              }
            }
            break;

          /* SafetyCANFD3 Group4 */
          case RB_NUMBER_GROUP4:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP4)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_SODLSafetyCANFD3Frame37_oSafetyCANFD3_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[3U]);
                FLAG_GROUP4 = TRUE;
                FlagMsgTable[3U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[3U]);
              }
            }
            break;

          /* SafetyCANFD3 Group5 */
          case RB_NUMBER_GROUP5:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP5)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_SODRSafetyCANFD3Frame37_oSafetyCANFD3_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[4U]);
                FLAG_GROUP5 = TRUE;
                FlagMsgTable[4U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[4U]);
              }
            }
            break;

          /* SafetyCANFD10 Group6 */
          case RB_NUMBER_GROUP6:
            if (RB_Get_FreeSize(&TxRB_Can_GROUP6)){
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_FlcPrivateCanFD10Fr35_oPrivate_CANFD10_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[5U]);
                FLAG_GROUP6 = TRUE;
                FlagMsgTable[5U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[5U]);
              }
              if (RxPduId == Cdd_PduRConf_CddPduRUpperLayerRxPdu_FlcPrivateCanFD10Fr32_oPrivate_CANFD10_00588000_Rx)
              {
                GET_SPINLOCK(Cdd_PduR_SpinlockGourp[6U]);
                FLAG_GROUP6_OTHER = TRUE;
                FlagMsgTable[6U].timecount = 0U;
                RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[6U]);
              }
            }
            break;
        }
      }
    }
    else if (RxPduId < NUM_OF_RX_SYSTEM_ELEMENTS){
      FlexElement * FlexElementTx = RxElement_map_table[RxPduId].FlexBufferPtr;
      if (FlexElementTx != NULL_PTR){
        if (PduInfoPtr->SduLength != FlexElementTx->eraylc){
          CtCdUartTrace_printf("\r\n FR DLC error, SduLength = %d \r\n", PduInfoPtr->SduLength);
        }
        uint16 datalength_flex = FlexElementTx->eraylc;
        memcpy(FlexElementTx->payload, PduInfoPtr->SduDataPtr, datalength_flex);

        if (RB_Get_FreeSize(&TxRB_Flex)){
          // write flexray data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
          RB_Write_String(&TxRB_Flex, (uint8*)&FlexElementTx, PtrLength_Tc39x);
        }
      }
    }
    else{
      // error "invalid RxPduId";
    }
  }
}

/**********************************************************************************************************************
  WriteDataToRingBuffer
**********************************************************************************************************************/
/*! \brief         The function is called to write rxpdu data to ringbuffer about group msg.
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \context       The function can be called in interrupt and on task level and should not to interrupted by another
                   Cdd_PduR_TxConfirmation call for the same TxPduId.
    \note          The function is called by the FlagMsgTimeMonitor.
**********************************************************************************************************************/
void WriteDataToRingBuffer(void)
{
  if (FLAG_GROUP1 == TRUE){
    for (uint8 GROUP1_INDEX=GROUP1_START; GROUP1_INDEX <= GROUP1_END; GROUP1_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP1_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP1_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP1))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP1, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP1_INDEX].isReceived == FALSE;
      }
    }
    FLAG_GROUP1 = FALSE;
  }
  if (FLAG_GROUP2 == TRUE){
    for (uint8 GROUP2_INDEX=GROUP2_START; GROUP2_INDEX <= GROUP2_END; GROUP2_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP2_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP2_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP2))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP2, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP2_INDEX].isReceived == FALSE;
      }
    }
    FLAG_GROUP2 = FALSE;
  }
  if (FLAG_GROUP3 == TRUE){
    for (uint8 GROUP3_INDEX=GROUP3_START; GROUP3_INDEX <= GROUP3_END; GROUP3_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP3_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP3_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP3))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP3, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP3_INDEX].isReceived == FALSE;
      }
    }
    FLAG_GROUP3 = FALSE;
  }
  if (FLAG_GROUP4 == TRUE){
    for (uint8 GROUP4_INDEX=GROUP4_START; GROUP4_INDEX <= GROUP4_END; GROUP4_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP4_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP4_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP4))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP4, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP4_INDEX].isReceived == FALSE;
      }
    }
    FLAG_GROUP4 = FALSE;
  }
  if (FLAG_GROUP5 == TRUE){
    for (uint8 GROUP5_INDEX=GROUP5_START; GROUP5_INDEX <= GROUP5_END; GROUP5_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP5_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP5_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP5))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP5, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP5_INDEX].isReceived == FALSE;
      }
    }
    FLAG_GROUP5 = FALSE;
  }
  if (FLAG_GROUP6 == TRUE && CheckFlag == FALSE){
    for (uint8 GROUP6_INDEX=GROUP6_START; GROUP6_INDEX <= GROUP6_END; GROUP6_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP6_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP6_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP6))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP6, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP6_INDEX].isReceived == FALSE;
      }
      CheckFlag = TRUE;
    }
    FLAG_GROUP6 = FALSE;
  }
  else if (FLAG_GROUP6 == TRUE && FLAG_GROUP6_OTHER == TRUE &&CheckFlag == TRUE){
    for (uint8 GROUP6_INDEX=GROUP6_START; GROUP6_INDEX <= GROUP6_END; GROUP6_INDEX++){
      CanElement * CanElementTx = RxElement_map_table[GROUP6_INDEX].CanBufferPtr;
      if ((RxElement_map_table[GROUP6_INDEX].isReceived == TRUE) && (RB_Get_FreeSize(&TxRB_Can_GROUP6))){
        // write can data to the ring buffer & the pointer length is fixed at 4byte in Tc39x
        RB_Write_String(&TxRB_Can_GROUP6, (uint8*)&CanElementTx, PtrLength_Tc39x);
        /* !!! Do I need to set the isReceived to FALSE? There is no mention of this in the demand */
        //RxElement_map_table[GROUP6_INDEX].isReceived == FALSE;
      }
      CheckFlag = FALSE;
    }
    FLAG_GROUP6 = FALSE;
  }
}


/**********************************************************************************************************************
  Cdd_PduR_TxConfirmation
**********************************************************************************************************************/
/*! \brief         The function is called to confirm the complete transmission of a TX I-PDU.
    \param[in]     TxPduId      id of the IF CddPduRUpperLayerTxPdu.
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \context       The function can be called in interrupt and on task level and should not to interrupted by another
                   Cdd_PduR_TxConfirmation call for the same TxPduId.
    \note          The function is called by the PduR.
**********************************************************************************************************************/
FUNC(void, CDD_PDUR_CODE) Cdd_PduR_TxConfirmation(PduIdType TxPduId)
{
  
}

/**********************************************************************************************************************
  Cdd_PduR_TriggerTransmit
**********************************************************************************************************************/
/*! \brief         The function is called to indicate the complete reception of a RX I-PDU.
    \param[in]     TxPduId      id of the IF CddPduRUpperLayerTxPdu.
    \param[in,out] PduInfoPtr   Contains a pointer to a buffer (SduDataPtr) to where the SDU
                                data shall be copied, and the available buffer size in SduLengh.
                                On return, the service will indicate the length of the copied SDU
                                data in SduLength.
    \return        E_OK         SDU has been copied and SduLength indicates the number of copied bytes.
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by PduR.
**********************************************************************************************************************/
FUNC(Std_ReturnType, CDD_PDUR_CODE) Cdd_PduR_TriggerTransmit(PduIdType TxPduId, P2VAR(PduInfoType, AUTOMATIC, CDD_PDUR_APPL_DATA) PduInfoPtr)
{
  Std_ReturnType retval = E_NOT_OK;

  if (PduInfoPtr == NULL_PTR){
    // error "invalid PduInfoPtr"
  }
  else{
    if (TxPduId<CAN_NUM_OF_TX_SYSTEM_ELEMENTS){
      retval = E_NOT_OK;
    }
    else if (TxPduId<NUM_OF_TX_SYSTEM_ELEMENTS){
      FlexElement * TxDataPtr_Flex = TxElement_map_table[TxPduId].FlexBufferPtr;
      if (TxDataPtr_Flex != NULL_PTR){
        PduInfoPtr->SduLength = TxDataPtr_Flex->eraylc;
        memcpy(PduInfoPtr->SduDataPtr, TxDataPtr_Flex->payload, TxDataPtr_Flex->eraylc);

        retval = E_OK;
      }
      else{
        retval = E_NOT_OK;
      }
    }
  }

  return retval;
}

/**********************************************************************************************************************
  AssmbleCanFromRings
**********************************************************************************************************************/
/*! \brief         The function is Assmble Tx-Pdu include smart-header and caninfo info
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Core2 Task 5ms.
**********************************************************************************************************************/
void AssmbleCanFromRings(ring_buffer *TxRB_Can, uint16 *offset_addrPtr, uint8 *SoadSrcPduInfo)
{
  CanElement * TxDataPtr_Can = NULL_PTR;
  PduInfoType SoadSrcData;
  uint16 offset_addr = *offset_addrPtr;

  if (RB_Get_Length(TxRB_Can))
  {
    uint32 RB_length_Can = RB_Get_Length(TxRB_Can);
    /* get a pointer to tx data from the ringbuffer and write it to the soad src pdu info */
    for (uint16 i=0U; i<RB_length_Can; i+=PtrLength_Tc39x)
    {
      RB_Read_String(TxRB_Can, (uint8*)&TxDataPtr_Can, PtrLength_Tc39x);
      if (TxDataPtr_Can != NULL_PTR){
        if ((offset_addr+Info_Header_Length+TxDataPtr_Can->CanDlc) > UDP_MAXDATA_BYTES){
          /* when the maximum byte of one udp is exceeded the remaining data group is sent 
          to the next udp or several times but it is still in the same 2ms cycle */
          if (offset_addr <= UDP_MAXDATA_BYTES)
          {
            SoadSrcData.SduDataPtr = SoadSrcPduInfo;
            SoadSrcData.SduLength = offset_addr;
            /* socket has only one port, so pdu id is fixed at SOCKET_SOAD_PORT */
            SoAd_IfTransmit(SOCKET_SOAD_PORT, &SoadSrcData);

            // reset value of SoadSrcData & offset_addr
            memset(SoadSrcPduInfo, 0U, UDP_MAXDATA_BYTES);
            offset_addr = 0U;
          }
        }
        TxDataPtr_Can->Seqnum = count_Can++;
        memcpy(SoadSrcPduInfo+offset_addr, TxDataPtr_Can, Info_Header_Length); /* PRQA S 1495 */
        offset_addr += Info_Header_Length;
        memcpy(SoadSrcPduInfo+offset_addr, TxDataPtr_Can->payload, TxDataPtr_Can->CanDlc);
        offset_addr += TxDataPtr_Can->CanDlc;
      }
    }
    *offset_addrPtr = offset_addr;
  }
}

/**********************************************************************************************************************
  Cdd_AssmbleCanTxPdu_ToSoc
**********************************************************************************************************************/
/*! \brief         The function is Assmble Tx-Pdu include smart-header and caninfo info
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Core2 Task 5ms.
**********************************************************************************************************************/
void Cdd_AssmbleCanTxPdu_ToSoc(void)
{
  uint8 SoadSrcPduInfo[UDP_MAXDATA_BYTES];
  uint16 offset_addr = 0U;
  CanElement * TxDataPtr_Can = NULL_PTR;
  PduInfoType SoadSrcData;

  for (uint8 rb_index=0U; rb_index<7U; rb_index++)
  {
    switch (TxRB_List_Can[rb_index].rb_number)
    {
      case RB_NUMBER_NOT_GROUPED:
        AssmbleCanFromRings(&TxRB_Can_NotGroup, &offset_addr, SoadSrcPduInfo);
        break;
      
      case RB_NUMBER_GROUP1:
        //if (FLAG_GROUP1 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP1, &offset_addr, SoadSrcPduInfo);
        break;

      case RB_NUMBER_GROUP2:
        //if (FLAG_GROUP2 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP2, &offset_addr, SoadSrcPduInfo);
        break;

      case RB_NUMBER_GROUP3:
        //if (FLAG_GROUP3 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP3, &offset_addr, SoadSrcPduInfo);
        break;

      case RB_NUMBER_GROUP4:
        //if (FLAG_GROUP4 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP4, &offset_addr, SoadSrcPduInfo);
        break;

      case RB_NUMBER_GROUP5:
        //if (FLAG_GROUP5 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP5, &offset_addr, SoadSrcPduInfo);
        break;

      case RB_NUMBER_GROUP6:
        //if (FLAG_GROUP6 == TRUE)
          AssmbleCanFromRings(&TxRB_Can_GROUP6, &offset_addr, SoadSrcPduInfo);
        // if (FLAG_GROUP6 == TRUE && FLAG_GROUP6_OTHER == TRUE && CheckFlag == TRUE){
        //   AssmbleCanFromRings(&TxRB_Can_GROUP6, &offset_addr, SoadSrcPduInfo);
        //   CheckFlag = FALSE;
        // }
        break;
      
      default:
        break;
    }
    
  }

  if (offset_addr <= UDP_MAXDATA_BYTES)
  {
    SoadSrcData.SduDataPtr = SoadSrcPduInfo;
    SoadSrcData.SduLength = offset_addr;
    /* socket has only one port, so pdu id is fixed at SOCKET_SOAD_PORT */
    SoAd_IfTransmit(SOCKET_SOAD_PORT, &SoadSrcData);
  }
}

/**********************************************************************************************************************
  Cdd_AssmbleFlexTxPdu_ToSoc
**********************************************************************************************************************/
/*! \brief         The function is Assmble Tx-Pdu include smart-header and flexray info
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Core2 Task 5ms.
**********************************************************************************************************************/
void Cdd_AssmbleFlexTxPdu_ToSoc(void)
{
  uint8 SoadSrcPduInfo[UDP_MAXDATA_BYTES];
  uint16 offset_addr = 0U;
  FlexElement * TxDataPtr_Flex = NULL_PTR;
  PduInfoType SoadSrcData;

  if (RB_Get_Length(&TxRB_Flex))
  {
    uint16 RB_length_Flex = RB_Get_Length(&TxRB_Flex);
    /* get a pointer to tx data from the ringbuffer and write it to the soad src pdu info */
    for (uint16 i=0U; i<RB_length_Flex; i+=PtrLength_Tc39x)
    {
      RB_Read_String(&TxRB_Flex, (uint8*)&TxDataPtr_Flex, PtrLength_Tc39x);
      if (TxDataPtr_Flex != NULL_PTR){
        if ((offset_addr+Info_Header_Length+TxDataPtr_Flex->eraylc) > UDP_MAXDATA_BYTES){
          /* when the maximum byte of one udp is exceeded the remaining data group is sent 
          to the next udp or several times but it is still in the same 2ms cycle */
          if (offset_addr <= UDP_MAXDATA_BYTES)
          {
            SoadSrcData.SduDataPtr = SoadSrcPduInfo;
            SoadSrcData.SduLength = offset_addr;
            /* socket has only one port, so pdu id is fixed at SOCKET_SOAD_PORT */
            SoAd_IfTransmit(SOCKET_SOAD_PORT, &SoadSrcData);

            // reset value of SoadSrcData & offset_addr
            memset(SoadSrcPduInfo, 0U, UDP_MAXDATA_BYTES);
            offset_addr = 0U;
          }
        }
        TxDataPtr_Flex->Seqnum = count_Flex++;
        memcpy(SoadSrcPduInfo+offset_addr, TxDataPtr_Flex, Info_Header_Length); /* PRQA S 1495 */
        offset_addr += Info_Header_Length;
        memcpy(SoadSrcPduInfo+offset_addr, TxDataPtr_Flex->payload, TxDataPtr_Flex->eraylc);  /*PRQA S 1-4882 ++*/
        offset_addr += TxDataPtr_Flex->eraylc;
      }
    }
    if (offset_addr <= UDP_MAXDATA_BYTES)
    {
      SoadSrcData.SduDataPtr = SoadSrcPduInfo;
      SoadSrcData.SduLength = offset_addr;
      /* socket has only one port, so pdu id is fixed at SOCKET_SOAD_PORT */
      SoAd_IfTransmit(SOCKET_SOAD_PORT, &SoadSrcData);
    }
  }
}

/**********************************************************************************************************************
  FlexTimeReset
**********************************************************************************************************************/
/*! \brief         Reset flexray time count to 0u if soad get flexray message from soc.
    \param[in]     uint16 flexIndex Tx-Pdu id in Cbk.h
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Cdd_TransmitRxPdu_ToMcu.
**********************************************************************************************************************/
void FlexTimeReset(uint16 flexIndex){
  if (flexIndex < NUM_OF_TX_SYSTEM_ELEMENTS){
    TxElement_map_table_Offset[flexIndex].timecount = 0U;
  }
}

/**********************************************************************************************************************
  FlexIncreaseTime
**********************************************************************************************************************/
/*! \brief         Add flexray cycle time count++, every 2ms count += 1;
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Core0 Task CanFr.
**********************************************************************************************************************/
void FlexIncreaseTime(void){
  for (uint8 flexIndex=CAN_NUM_OF_TX_SYSTEM_ELEMENTS; flexIndex<NUM_OF_TX_SYSTEM_ELEMENTS; flexIndex++){
    if (TxElement_map_table_Offset[flexIndex].timecount < FLEX_USRTIMER_MAX){
      TxElement_map_table_Offset[flexIndex].timecount++;
    }
    #if(SIG_TIMEOUTCHECK == STD_ON)
    if (TxElement_map_table_Offset[flexIndex].pduId == Cdd_PduRConf_CddPduRUpperLayerTxPdu_ASDMBackBoneSignalIPdu20_00588000_Tx){
      if (AsyAutDrvCtrlTypDIMReq_timeout < AsyAutDrvCtrlTypDIMReq_Max)
        AsyAutDrvCtrlTypDIMReq_timeout++;
    }
    #endif
  }
}

/**********************************************************************************************************************
  FlagMsgTimeMonitor
**********************************************************************************************************************/
/*! \brief         Add flag msg cycle time count++, every 2ms count += 1;
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Cdd_PduR is initialized and active.
    \note          The function is called by Core0 Task CanFr.
**********************************************************************************************************************/
void FlagMsgTimeMonitor(void)
{
  for (uint8 msgindex=0U; msgindex<7U; msgindex++)
  {
    if (FlagMsgTable[msgindex].timecount < FlagMsgTable[msgindex].cycletime)
    {
      GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
      FlagMsgTable[msgindex].timecount++;
      RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
    } 
    else
    {
      switch (FlagMsgTable[msgindex].pduId)
      {
        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_FSRLSafetyCANFD1Frame37_oSafetyCANFD1_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP1 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_FSRRSafetyCANFD1Frame37_oSafetyCANFD1_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP2 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_FLRSafetyCANFD2Frame64_oSafetyCANFD2_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP3 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_SODLSafetyCANFD3Frame37_oSafetyCANFD3_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP4 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_SODRSafetyCANFD3Frame37_oSafetyCANFD3_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP5 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_FlcPrivateCanFD10Fr35_oPrivate_CANFD10_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP6 = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;

        case Cdd_PduRConf_CddPduRUpperLayerRxPdu_FlcPrivateCanFD10Fr32_oPrivate_CANFD10_00588000_Rx:
          GET_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          FLAG_GROUP6_OTHER = FALSE;
          RELEASE_SPINLOCK(Cdd_PduR_SpinlockGourp[msgindex]);
          break;
      
        default:
          break;
      }
    }
  }
  WriteDataToRingBuffer();
}

/**********************************************************************************************************************
  TimeMonitor
**********************************************************************************************************************/
/*! \brief         The function is Monitor Tx-Pdu flexray the cycle time exceed 1 second
    \param[in]     none
    \param[in,out] none
    \return        none
    \pre           The Core0 Task_CanFr_C0 is initialized and active.
    \note          The function is called by Core0 Task CanFr.
**********************************************************************************************************************/
void TimeMonitor(void){
  PduInfoType SoadSrcData;

  for (uint8 flexIndex=CAN_NUM_OF_TX_SYSTEM_ELEMENTS; flexIndex<NUM_OF_TX_SYSTEM_ELEMENTS; flexIndex++){
    FlexElement * TxDataPtr_Flex = TxElement_map_table[flexIndex].FlexBufferPtr;
    if (TxDataPtr_Flex != NULL_PTR){
      if (TxElement_map_table_Offset[flexIndex].pduId == Cdd_PduRConf_CddPduRUpperLayerTxPdu_ASDMBackBoneSignalIPdu20_00588000_Tx)
      {
        #if(SIG_TIMEOUTCHECK == STD_ON)
        if (AsyAutDrvCtrlTypDIMReq_timeout == AsyAutDrvCtrlTypDIMReq_Max && TxElement_map_table_Offset[flexIndex].timecount == FLEX_USRTIMER_MAX)
        {
          memset(TxDataPtr_Flex->payload, 0U, TxDataPtr_Flex->eraylc);
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else if (AsyAutDrvCtrlTypDIMReq_timeout != AsyAutDrvCtrlTypDIMReq_Max && TxElement_map_table_Offset[flexIndex].timecount == FLEX_USRTIMER_MAX)
        {
          uint8 AsyAutDrvCtrlTyp_temp;
          memcpy(&AsyAutDrvCtrlTyp_temp, TxDataPtr_Flex->payload+11U, 1U);
          memset(TxDataPtr_Flex->payload, 0U, TxDataPtr_Flex->eraylc);
          memcpy(TxDataPtr_Flex->payload+11U, &AsyAutDrvCtrlTyp_temp, 1U);
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else if (AsyAutDrvCtrlTypDIMReq_timeout == AsyAutDrvCtrlTypDIMReq_Max && TxElement_map_table_Offset[flexIndex].timecount != FLEX_USRTIMER_MAX)
        {
          memset(TxDataPtr_Flex->payload+11U, 0U, 1U);
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else if (AsyAutDrvCtrlTypDIMReq_timeout != AsyAutDrvCtrlTypDIMReq_Max && TxElement_map_table_Offset[flexIndex].timecount != FLEX_USRTIMER_MAX)
        {
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        #else
        if (TxElement_map_table_Offset[flexIndex].timecount == FLEX_USRTIMER_MAX){
          uint8 AsyAutDrvCtrlTyp_temp;
          memcpy(&AsyAutDrvCtrlTyp_temp, TxDataPtr_Flex->payload+11U, 1U);
          memset(TxDataPtr_Flex->payload, 0U, TxDataPtr_Flex->eraylc);
          memcpy(TxDataPtr_Flex->payload+11U, &AsyAutDrvCtrlTyp_temp, 1U);
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else if(TxElement_map_table_Offset[flexIndex].timecount != FLEX_USRTIMER_MAX){
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        #endif
      }
      else
      {
        if(TxElement_map_table_Offset[flexIndex].timecount == FLEX_USRTIMER_MAX){
          // if the time exceeds 1 second the locally stored buffer is cleared
          memset(TxDataPtr_Flex->payload, 0U, TxDataPtr_Flex->eraylc);
          // call PduR_Cdd_PduRTransmit to note
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else if(TxElement_map_table_Offset[flexIndex].timecount != FLEX_USRTIMER_MAX){
          SoadSrcData.SduLength = TxDataPtr_Flex->eraylc;
          SoadSrcData.SduDataPtr = TxDataPtr_Flex->payload;
          PduR_Cdd_PduRTransmit(TxElement_map_table_Offset[flexIndex].pduId_offset, &SoadSrcData);
        }
        else{
          // nothing to do
        }
      }
    }
  }
}


# define CDD_PDUR_STOP_SEC_CODE
/*lint -save -esym(961, 19.1) */
# include "MemMap.h"    /* PRQA S 5087 */       /* MD_MSR_MemMap */
/*lint -restore */
