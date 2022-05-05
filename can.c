//###########################################################################
//
// FILE:   can.c
//
// TITLE:  C28x CAN driver.
//
//###########################################################################
// $TI Release: F28004x Support Library v1.11.00.00 $
// $Release Date: Sun Oct  4 15:49:15 IST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################
#include "can.h"

//*****************************************************************************
//
// CAN_initModule
//
//*****************************************************************************
void
CAN_initModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
#ifndef __TMS320C2000__
    //
    // Place CAN controller in init state, regardless of previous state.  This
    // will put controller in idle, and allow the message object RAM to be
    // programmed.
    //
    HWREGH(base + CAN_O_CTL) |= ((uint16_t)CAN_CTL_INIT |
                                 (uint16_t)CAN_INIT_PARITY_DISABLE);

    //
    // Initialize the message RAM before using it.
    //
    CAN_initRAM(base);

    //
    // Force module to reset state
    //
    EALLOW;

    HWREGH(base + CAN_O_CTL) |=  CAN_CTL_SWR;
    EDIS;

    //
    // Delay for 14 cycles
    //
    SysCtl_delay(1U);

    //
    // Enable write access to the configuration registers
    //
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_CCE;
#else
    struct ECAN_REGS ECanaShadow;

    EALLOW;     // EALLOW enables access to protected bits

    ECanaShadow.CANTIOC.all = pECanaRegs->CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    pECanaRegs->CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = pECanaRegs->CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    pECanaRegs->CANRIOC.all = ECanaShadow.CANRIOC.all;

    // CANMC: Master Control Register
    ECanaShadow.CANMC.all = pECanaRegs->CANMC.all;
    ECanaShadow.CANMC.bit.SCB = 1;
    ECanaShadow.CANMC.bit.ABO = 1;
    ECanaShadow.CANMC.bit.DBO = 1;  //!< Intel Endianness!
    pECanaRegs->CANMC.all = ECanaShadow.CANMC.all;

    /* Initialize all bits of 'Message Control Register' to zero */
    // Some bits of MSGCTRL register come up in an unknown state. For proper operation,
    // all bits (including reserved bits) of MSGCTRL must be initialized to zero

    pECanaMboxes->MBOX0.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX1.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX2.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX3.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX4.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX5.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX6.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX7.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX8.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX9.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX10.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX11.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX12.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX13.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX14.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX15.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX16.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX17.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX18.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX19.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX20.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX21.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX22.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX23.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX24.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX25.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX26.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX27.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX28.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX29.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX30.MSGCTRL.all = 0x00000000;
    pECanaMboxes->MBOX31.MSGCTRL.all = 0x00000000;

    // TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
    //  as a matter of precaution.

    pECanaRegs->CANTA.all = 0xFFFFFFFF;   /* Clear all TAn bits */
    pECanaRegs->CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */
    pECanaRegs->CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
    pECanaRegs->CANGIF1.all = 0xFFFFFFFF;
    EDIS;
#endif
}

//*****************************************************************************
//
// CAN_setBitRate
//
//*****************************************************************************
void
CAN_setBitRate(uint32_t base, uint32_t clock, uint32_t bitRate,
               uint16_t bitTime)
{
    uint16_t brp;
    uint16_t tPhase;
    uint16_t phaseSeg2;
    uint16_t tSync = 1U;
    uint16_t tProp = 2U;
    uint16_t tSeg1;
    uint16_t tSeg2;
    uint16_t sjw;
    uint16_t prescaler;
    uint16_t prescalerExtension;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((bitTime > 7U) && (bitTime < 26U));
    ASSERT(bitRate <= 1000000U);

    //
    // Calculate bit timing values
    //
    brp = (uint16_t)(clock / (bitRate * bitTime));
    tPhase = bitTime - (tSync + tProp);
    if((tPhase / 2U) <= 8U)
    {
        phaseSeg2 = tPhase / 2U;
    }
    else
    {
        phaseSeg2 = 8U;
    }
    tSeg1 = ((tPhase - phaseSeg2) + tProp) - 1U;
    tSeg2 = phaseSeg2 - 1U;
    if(phaseSeg2 > 4U)
    {
        sjw = 3U;
    }
    else
    {
        sjw = tSeg2;
    }
    prescalerExtension = ((brp - 1U) / 64U);
    prescaler = ((brp - 1U) % 64U);

    //
    // Set the calculated timing parameters
    //
    CAN_setBitTiming(base, prescaler, prescalerExtension, tSeg1, tSeg2, sjw);
}

//*****************************************************************************
//
// CAN_setBitTiming
//
//*****************************************************************************
void
CAN_setBitTiming(uint32_t base, uint16_t prescaler,
                 uint16_t prescalerExtension, uint16_t tSeg1, uint16_t tSeg2,
                 uint16_t sjw)
{
    uint32_t bitReg;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT(prescaler < 64U);
    ASSERT((tSeg1 > 0U) && (tSeg1 < 16U));
    ASSERT(tSeg2 < 8U);
    ASSERT(sjw < 4U);
    ASSERT(prescalerExtension < 16U);

    //
    // To set the bit timing register, the controller must be placed in init
    // mode (if not already), and also configuration change bit enabled.
    // State of the init bit should be saved so it can be restored at the end.
    //
#ifndef __TMS320C2000__
    uint16_t savedInit;
    savedInit = HWREGH(base + CAN_O_CTL);
    HWREGH(base + CAN_O_CTL) = savedInit | CAN_CTL_INIT | CAN_CTL_CCE;
#endif
    //
    // Set the bit fields of the bit timing register
    //
    bitReg = (uint32_t)((uint32_t)prescaler & CAN_BTR_BRP_M);
    bitReg |= (uint32_t)(((uint32_t)sjw << CAN_BTR_SJW_S) & CAN_BTR_SJW_M);
    bitReg |= (uint32_t)(((uint32_t)tSeg1 << CAN_BTR_TSEG1_S) &
                         CAN_BTR_TSEG1_M);
    bitReg |= (uint32_t)(((uint32_t)tSeg2 << CAN_BTR_TSEG2_S) &
                         CAN_BTR_TSEG2_M);
    bitReg |= (uint32_t)(((uint32_t)prescalerExtension << CAN_BTR_BRPE_S) &
                         CAN_BTR_BRPE_M);

#ifndef __TMS320C2000__
    HWREG_BP(base + CAN_O_BTR) = bitReg;
    //
    // Clear the config change bit, and restore the init bit.
    //
    savedInit &= ~((uint16_t)CAN_CTL_CCE);

    //
    // If Init was not set before, then clear it.
    //
    if((savedInit & CAN_CTL_INIT) == CAN_CTL_INIT)
    {
        savedInit &= ~((uint16_t)CAN_CTL_INIT);
    }
    HWREGH(base + CAN_O_CTL) = savedInit;
#else
    struct ECAN_REGS ECanaShadow;

    EALLOW;

    /* Configure bit timing parameters for ECana*/

    ECanaShadow.CANMC.all = pECanaRegs->CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    pECanaRegs->CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
        {
        ECanaShadow.CANES.all = pECanaRegs->CANES.all;
        } while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;

    /* The following block for all 90 MHz SYSCLKOUT - default. Bit rate = 0.5 Mbps */
    ECanaShadow.CANBTC.bit.BRPREG = prescaler; // 5;
    //ECanaShadow.CANBTC.bit.BRPREG = 11; // 250khz
    ECanaShadow.CANBTC.bit.TSEG2REG = tSeg2; // 2;
    ECanaShadow.CANBTC.bit.TSEG1REG = tSeg1; // 10;

    ECanaShadow.CANBTC.bit.SAM = 0;
    pECanaRegs->CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = pECanaRegs->CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    pECanaRegs->CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU no longer has permission to change the configuration registers
    do
    {
        ECanaShadow.CANES.all = pECanaRegs->CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..

    /* Disable all Mailboxes  */
    pECanaRegs->CANME.all = 0;        // Required before writing the MSGIDs

    EDIS;
#endif
}


//*****************************************************************************
//
// CAN_clearInterruptStatus
//
//*****************************************************************************
void
CAN_clearInterruptStatus(uint32_t base, uint32_t intClr)
{
#ifndef __TMS320C2000__
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intClr == CAN_INT_INT0ID_STATUS) ||
           ((intClr >= 1U) && (intClr <= 32U)));

    if(intClr == (uint32_t)CAN_INT_INT0ID_STATUS)
    {
        //
        // Simply read and discard the status to clear the interrupt.
        //
        HWREGH(base + CAN_O_ES);
    }
    else
    {
        //
        // Wait to be sure that this interface is not busy.
        //
        while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) ==
              CAN_IF1CMD_BUSY)
        {
        }

        //
        // Only change the interrupt pending state by setting only the
        // CAN_IF1CMD_CLRINTPND bit.
        //
        // Send the clear pending interrupt command to the CAN controller.
        //
        HWREG_BP(base + CAN_O_IF1CMD) = ((uint32_t)CAN_IF1CMD_CLRINTPND |
                                        (intClr & CAN_IF1CMD_MSG_NUM_M));

        //
        // Wait to be sure that this interface is not busy.
        //
        while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) ==
              CAN_IF1CMD_BUSY)
        {
        }
    }
#else
    pECanaRegs->CANRMP.all |= (1ul << (intClr-1));
#endif
}

//*****************************************************************************
//
// CAN_setupMessageObject
//
//*****************************************************************************
void
CAN_setupMessageObject(uint32_t base, uint32_t objID, uint32_t msgID,
                       CAN_MsgFrameType frame, CAN_MsgObjType msgType,
                       uint32_t msgIDMask, uint32_t flags, uint16_t msgLen)
{
#ifndef __TMS320C2000__
    uint32_t cmdMaskReg = 0U;
    uint32_t maskReg = 0U;
    uint32_t arbReg = 0U;
    uint32_t msgCtrl = 0U;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID > 0U));
    ASSERT(msgLen <= 8U);

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    switch(msgType)
    {
        //
        // Transmit message object.
        //
        case CAN_MSG_OBJ_TYPE_TX:
        {
            //
            // Set message direction to transmit.
            //
            arbReg = CAN_IF1ARB_DIR;
            break;
        }

        //
        // Remote frame receive remote, with auto-transmit message object.
        //
        case CAN_MSG_OBJ_TYPE_RXTX_REMOTE:
        {
            //
            // Set message direction to Tx for remote receivers.
            //
            arbReg = CAN_IF1ARB_DIR;

            //
            // Set this object to auto answer if a matching identifier is seen.
            //
            msgCtrl = (uint32_t)((uint32_t)CAN_IF1MCTL_RMTEN |
                                 (uint32_t)CAN_IF1MCTL_UMASK);

            break;
        }

        //
        // Transmit remote request message object (CAN_MSG_OBJ_TYPE_TX_REMOTE)
        // or Receive message object (CAN_MSG_OBJ_TYPE_RX).
        //
        default:
        {
           //
           // Set message direction to read.
           //
           arbReg = 0U;

           break;
        }
    }

    //
    // Set values based on Extended Frame or Standard Frame
    //
    if(frame == CAN_MSG_FRAME_EXT)
    {
        //
        // Configure the Mask Registers for 29 bit Identifier mask.
        //
        if((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
            maskReg = msgIDMask & CAN_IF1MSK_MSK_M;
        }

        //
        // Set the 29 bit version of the Identifier for this message
        // object. Mark the message as valid and set the extended ID bit.
        //
        arbReg |= (msgID & CAN_IF1ARB_ID_M) | CAN_IF1ARB_MSGVAL |
                  CAN_IF1ARB_XTD;
    }
    else
    {
        //
        // Configure the Mask Registers for 11 bit Identifier mask.
        //
        if((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
           maskReg = ((msgIDMask << CAN_IF1ARB_STD_ID_S) &
                      CAN_IF1ARB_STD_ID_M);
        }

        //
        // Set the 11 bit version of the Identifier for this message
        // object. The lower 18 bits are set to zero. Mark the message as
        // valid.
        //
        arbReg |= ((msgID << CAN_IF1ARB_STD_ID_S) & CAN_IF1ARB_STD_ID_M) |
                  CAN_IF1ARB_MSGVAL;
    }

    //
    // If the caller wants to filter on the extended ID bit then set it.
    //
    maskReg |= (flags & CAN_MSG_OBJ_USE_EXT_FILTER);

    //
    // The caller wants to filter on the message direction field.
    //
    maskReg |= (flags & CAN_MSG_OBJ_USE_DIR_FILTER);

    //
    // If any filtering is requested, set the UMASK bit to use mask register
    //
    if(((flags & CAN_MSG_OBJ_USE_ID_FILTER) |
        (flags & CAN_MSG_OBJ_USE_DIR_FILTER) |
        (flags & CAN_MSG_OBJ_USE_EXT_FILTER)) != 0U)
    {
        msgCtrl |= CAN_IF1MCTL_UMASK;
    }

    //
    // Set the data length for the transfers.
    // This is applicable for Tx mailboxes.
    //
    msgCtrl |= ((uint32_t)msgLen & CAN_IF1MCTL_DLC_M);

    //
    // If this is a single transfer or the last mailbox of a FIFO, set EOB bit.
    // If this is not the last entry in a FIFO, leave the EOB bit as 0.
    //
    if((flags & CAN_MSG_OBJ_FIFO) == 0U)
    {
        msgCtrl |= CAN_IF1MCTL_EOB;
    }

    //
    // Enable transmit interrupts if they should be enabled.
    //
    msgCtrl |= (flags & CAN_MSG_OBJ_TX_INT_ENABLE);

    //
    // Enable receive interrupts if they should be enabled.
    //
    msgCtrl |= (flags & CAN_MSG_OBJ_RX_INT_ENABLE);

    //
    // Set the Control, Arb, and Mask bit so that they get transferred to the
    // Message object.
    //
    cmdMaskReg |= CAN_IF1CMD_ARB;
    cmdMaskReg |= CAN_IF1CMD_CONTROL;
    cmdMaskReg |= CAN_IF1CMD_MASK;
    cmdMaskReg |= CAN_IF1CMD_DIR;

    //
    // Write out the registers to program the message object.
    //
    HWREG_BP(base + CAN_O_IF1MSK) = maskReg;
    HWREG_BP(base + CAN_O_IF1ARB) = arbReg;
    HWREG_BP(base + CAN_O_IF1MCTL) = msgCtrl;

    //
    // Transfer data to message object RAM
    //
    HWREG_BP(base + CAN_O_IF1CMD) =
    cmdMaskReg | (objID & CAN_IF1CMD_MSG_NUM_M);
#else
    struct ECAN_REGS ECanaShadow;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID > 0U));
    ASSERT(msgLen <= 8U);

    volatile struct MBOX *mboxes = &pECanaMboxes->MBOX0;

    objID--;

    mboxes[objID].MSGID.all = 0;

    /**
     * Configura message-id per la mailbox
     */
    if (frame == CAN_MSG_FRAME_STD) {
        mboxes[objID].MSGID.bit.STDMSGID = msgID;
        mboxes[objID].MSGID.bit.IDE = 0;
    } else {
        mboxes[objID].MSGID.bit.EXTMSGID_L = msgID & 0x0000FFFF;
        mboxes[objID].MSGID.bit.EXTMSGID_H = (msgID & 0x00030000) >> 16;
        mboxes[objID].MSGID.bit.STDMSGID   = (msgID & 0x1FFC0000) >> 18;
        mboxes[objID].MSGID.bit.IDE = 1;
    }

    /**
    * Configura le mailbox in ricezione/trasmissione
    */
    ECanaShadow.CANMD.all = pECanaRegs->CANMD.all;
    if (msgType==CAN_MSG_OBJ_TYPE_RX)
        ECanaShadow.CANMD.all |= (1ul << objID);
    else
        ECanaShadow.CANMD.all &= ~(1ul << objID);
    pECanaRegs->CANMD.all = ECanaShadow.CANMD.all;

    /**
    * Configura la lunghezza dei messaggi in byte.
    */
    mboxes[objID].MSGCTRL.bit.DLC = msgLen;

    /**
    * Abilita le mailbox.
    */
    ECanaShadow.CANME.all = pECanaRegs->CANME.all;
    ECanaShadow.CANME.all |= (1ul << objID);
    pECanaRegs->CANME.all = ECanaShadow.CANME.all;

    /**
    * Configura le interruzioni
    */
    EALLOW;

    /**
    * Configura quali mailbox generano le interruzioni.
    */
    if (((msgType==CAN_MSG_OBJ_TYPE_RX) && (flags | CAN_MSG_OBJ_RX_INT_ENABLE))  ||
            ((msgType==CAN_MSG_OBJ_TYPE_TX) && (flags | CAN_MSG_OBJ_TX_INT_ENABLE))) {
        pECanaRegs->CANMIM.all |= 1ul << objID;
    } else {
        pECanaRegs->CANMIM.all &= ~(1ul << objID);
    }

    /**
    * Mappa le interruzioni sulle linee 0 e 1.
    * Le mailbox in rx generano le interruzioni su ECANINT1.
    */
    if (msgType==CAN_MSG_OBJ_TYPE_RX) {
        pECanaRegs->CANMIL.all |= 1ul << objID;
    }

    EDIS;
#endif
}

//*****************************************************************************
//
// CAN_sendMessage
//
//*****************************************************************************
void
CAN_sendMessage(uint32_t base, uint32_t objID, uint16_t msgLen,
                const uint16_t *msgData)
{

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID > 0U));
    ASSERT(msgLen <= 8U);

#ifndef __TMS320C2000__
    uint32_t msgCtrl = 0U;
    //
    // Set IF command to read message object control value
    //
    // Set up the request for data from the message object.
    // Transfer the message object to the IF register.
    //
    HWREG_BP(base + CAN_O_IF1CMD) = ((uint32_t)CAN_IF1CMD_CONTROL |
                                     (objID & CAN_IF1CMD_MSG_NUM_M));

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    //
    // Read IF message control
    //
    msgCtrl = HWREGH(base + CAN_O_IF1MCTL);

    //
    // Check provided DLC size with actual Message DLC size
    //
    ASSERT((msgCtrl & CAN_IF1MCTL_DLC_M) == msgLen);

    //
    // Write the data out to the CAN Data registers.
    //
    CAN_writeDataReg(msgData, (base + CAN_O_IF1DATA),
                     (msgCtrl & CAN_IF1MCTL_DLC_M));

    //
    //  Set Data to be transferred from IF
    //
    if(msgLen > 0U)
    {
        msgCtrl = CAN_IF1CMD_DATA_B | CAN_IF1CMD_DATA_A;
    }
    else
    {
        msgCtrl = 0U;
    }

    //
    // Set Direction to write
    //
    // Set Tx Request Bit
    //
    // Transfer the message object to the message object specified by
    // objID.
    //
    HWREG_BP(base + CAN_O_IF1CMD) = (msgCtrl | (uint32_t)CAN_IF1CMD_DIR |
                                     (uint32_t)CAN_IF1CMD_TXRQST |
                                     (objID & CAN_IF1CMD_MSG_NUM_M));
#else
    volatile struct MBOX *mboxes = &pECanaMboxes->MBOX0;
    struct ECAN_REGS ECanaShadow;

    objID--;

    //!< don't stuck in CANTA wait if bus-off
    if ((
            pECanaRegs->CANES.bit.BO == 1)      // There is an abnormal rate of errors on the CAN bus. This condition occurs when the transmit error
                                                // counter (CANTEC) has reached the limit of 256. During Bus Off, no messages can be received or
                                                // transmitted. The bus-off state can be exited by clearing the CCR bit in CANMC register or if the
                                                // Auto Bus On (ABO) (CANMC.7) bit is set, after 128 * 11 receive bits have been received. After
                                                // leaving Bus Off, the error counters are cleared.
            || (pECanaRegs->CANES.bit.ACKE == 1) // The CAN module received no acknowledge.
            || (pECanaRegs->CANES.bit.EP == 1)
            || (pECanaRegs->CANES.bit.SMA == 1)
            || (pECanaRegs->CANES.bit.SA1 == 1)
            || (pECanaRegs->CANES.bit.BE == 1)) {
        pECanaRegs->CANES.all = 0xffffffff; // clear all
        return;
    }

    //!< wait previous transmission to be completed
    uint32_t timeout=10000;
    do {
        ECanaShadow.CANTRS.all = pECanaRegs->CANTRS.all;
        if(--timeout == 0U) {
            break;
        }
    } while ((ECanaShadow.CANTRS.all & (1ul << objID)) == 1);

    ECanaShadow.CANTA.all = 1ul << objID;       /* Clear TA bit */
    pECanaRegs->CANTA.all = ECanaShadow.CANTA.all;

    mboxes[objID].MDH.all = *(Uint32*)&msgData[2];
    mboxes[objID].MDL.all = *(Uint32*)&msgData[0];

    ECanaShadow.CANTRS.all = 1ul << objID;
    pECanaRegs->CANTRS.all = ECanaShadow.CANTRS.all;

#endif
}

//*****************************************************************************
//
// CAN_readMessage
//
//*****************************************************************************
bool
CAN_readMessage(uint32_t base, uint32_t objID,
                uint16_t *msgData)
{
    __attribute__((unused)) bool status=true;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID != 0U));

#ifndef __TMS320C2000__
    uint16_t msgCtrl = 0U;
    //
    // Set the Message Data A, Data B, and control values to be read
    // on request for data from the message object.
    //
    // Transfer the message object to the message object IF register.
    //
    HWREG_BP(base + CAN_O_IF2CMD) =
    ((uint32_t)CAN_IF2CMD_DATA_A | (uint32_t)CAN_IF2CMD_DATA_B |
     (uint32_t)CAN_IF2CMD_CONTROL | (objID & CAN_IF2CMD_MSG_NUM_M) |
	 (uint32_t)CAN_IF2CMD_ARB);

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
    {
    }

    //
    // Read out the IF control Register.
    //
    msgCtrl = HWREGH(base + CAN_O_IF2MCTL);

    //
    // See if there is new data available.
    //
    if((msgCtrl & CAN_IF2MCTL_NEWDAT) == CAN_IF2MCTL_NEWDAT)
    {
        //
        // Read out the data from the CAN registers.
        //
        CAN_readDataReg(msgData, (base + CAN_O_IF2DATA),
                        (msgCtrl & CAN_IF2MCTL_DLC_M));

        status = true;

        //
        // Now clear out the new data flag
        //
        HWREG_BP(base + CAN_O_IF2CMD) = ((uint32_t)CAN_IF2CMD_TXRQST |
                                        (objID & CAN_IF2CMD_MSG_NUM_M));

        //
        // Wait for busy bit to clear
        //
        while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) ==
               CAN_IF2CMD_BUSY)
        {
        }
    }
    else
    {
        status = false;
    }

#else
    volatile struct MBOX *mboxes = &pECanaMboxes->MBOX0;

    objID--;

    *(Uint32*)&msgData[2] = mboxes[objID].MDH.all;
    *(Uint32*)&msgData[0] = mboxes[objID].MDL.all;

    return(status);
#endif
}

bool CAN_readMessageWithID(uint32_t base,
                           uint32_t objID,
                           CAN_MsgFrameType *frameType,
                           uint32_t *msgID,
                           uint16_t *msgData)
{
    __attribute__((unused)) bool status=true;


    //
    // Check the arguments.
    //
    ASSERT(msgID != 0U);
    ASSERT(frameType != 0U);

#ifndef __TMS320C2000__
    //
    //Read the message first this fills the IF2 registers
    //with received message for that mailbox
    //
    status = CAN_readMessage(base, objID, msgData);
    //
    // See if there is new data available.
    //
    if(status == true)
    {
        if((HWREG_BP(base + CAN_O_IF2ARB) & CAN_IF2ARB_XTD) != 0U)
        {
            *frameType = CAN_MSG_FRAME_EXT;
            *msgID = ((HWREG_BP(base + CAN_O_IF2ARB)) & CAN_IF2ARB_ID_M);
        }
        else
        {
            *frameType = CAN_MSG_FRAME_STD;
            *msgID = (((HWREG_BP(base + CAN_O_IF2ARB)) &
                       CAN_IF2ARB_STD_ID_M) >>
                      CAN_IF2ARB_STD_ID_S);
        }
    }
#else
    ASSERT(false); // "TO BE IMPLEMENTED"
#endif
    return(status);
}
//*****************************************************************************
//
// CAN_transferMessage
//
//*****************************************************************************
void
CAN_transferMessage(uint32_t base, uint16_t interface, uint32_t objID,
                    bool direction, bool dmaRequest)
{

    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID >= 1U) && (objID <= 32U));
    ASSERT((interface == 1U) || (interface == 2U));

#ifndef __TMS320C2000__
    uint32_t cmdMaskReg;
    //
    // This is always a read to the Message object as this call is setting a
    // message object.
    //
    cmdMaskReg =
    ((uint32_t)CAN_IF1CMD_DATA_A | (uint32_t)CAN_IF1CMD_DATA_B |
     (uint32_t)CAN_IF1CMD_TXRQST | (uint32_t)CAN_IF1CMD_CONTROL |
     (uint32_t)CAN_IF1CMD_MASK | (uint32_t)CAN_IF1CMD_ARB) |
    (direction ? CAN_IF1CMD_DIR : 0U) |
    (dmaRequest ? CAN_IF1CMD_DMAACTIVE : 0U);

    //
    // Ensure this IF isn't busy
    //
    while((HWREGH(base + ((interface == 2U) ? CAN_O_IF2CMD : CAN_O_IF1CMD)) &
          CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    //
    // Set up the request for data from the message object. Transfer the
    // message object to the message object specified by objID.
    //
    HWREG_BP(base + ((interface == 2U) ? CAN_O_IF2CMD : CAN_O_IF1CMD)) =
                                (cmdMaskReg | (objID & CAN_IF1CMD_MSG_NUM_M));

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + ((interface == 2U) ? CAN_O_IF2CMD : CAN_O_IF1CMD)) &
          CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }
#else
    ASSERT(false); // "TO BE IMPLEMENTED"
#endif
}

//*****************************************************************************
//
// CAN_clearMessage
//
//*****************************************************************************
void
CAN_clearMessage(uint32_t base, uint32_t objID)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID >= 1U) && (objID <= 32U));

#ifndef __TMS320C2000__
    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    //
    // Clear the message valid bit in the arbitration register. This indicates
    // the message is not valid.
    //
    HWREG_BP(base + CAN_O_IF1ARB) = 0U;

    //
    // Initiate programming the message object
    //
    HWREG_BP(base + CAN_O_IF1CMD) =
    (((uint32_t)CAN_IF1CMD_DIR | (uint32_t)CAN_IF1CMD_ARB) |
     (objID & CAN_IF1CMD_MSG_NUM_M));
#else
    ASSERT(false); // "TO BE IMPLEMENTED"
#endif
}
