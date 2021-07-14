//###########################################################################
//
// FILE:   sysctl.c
//
// TITLE:  C28x system control driver.
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

#include "sysctl.h"
#include "dcc.h"

//
// Define to isolate inline assembly
//
#define SYSCTL_DELAY        __asm(" .if __TI_EABI__\n"                         \
                                  " .asg    SysCtl_delay    , _SysCtl_delay\n" \
                                  " .endif\n"                                  \
                                  " .def _SysCtl_delay\n"                      \
                                  " .sect \".TI.ramfunc\"\n"                   \
                                  " .global  _SysCtl_delay\n"                  \
                                  "_SysCtl_delay:\n"                           \
                                  " SUB    ACC,#1\n"                           \
                                  " BF     _SysCtl_delay, GEQ\n"               \
                                  " LRETR\n")


//
// Macro used for adding delay between 2 consecutive writes to CLKSRCCTL1
// register.
// Delay = 300 NOPs
//
#define SYSCTL_CLKSRCCTL1_DELAY  asm(" RPT #250 || NOP \n RPT #50 || NOP")

//*****************************************************************************
//
// SysCtl_delay()
//
//*****************************************************************************
SYSCTL_DELAY;

#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)90000000)) - 9.0L) / 5.0L)

//*****************************************************************************
//
// SysCtl_pollX1Counter()
//
//*****************************************************************************
#ifndef __TMS320C2000__
static void
SysCtl_pollX1Counter(void)
{
    uint16_t loopCount = 0U;

    //
    // Delay for 1 ms while the XTAL powers up
    //
    // 2000 loops, 5 cycles per loop + 9 cycles overhead = 10009 cycles
    //
    SysCtl_delay(2000);

    //
    // Clear and saturate X1CNT 4 times to guarantee operation
    //
    do
    {
        //
        // Keep clearing the counter until it is no longer saturated
        //
        while(HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) > 0x1FFU)
        {
            HWREG(CLKCFG_BASE + SYSCTL_O_X1CNT) |= SYSCTL_X1CNT_CLR;
        }

        //
        // Wait for the X1 clock to saturate
        //
        while(HWREGH(CLKCFG_BASE + SYSCTL_O_X1CNT) != SYSCTL_X1CNT_X1CNT_M)
        {
            //
            // If your application is stuck in this loop, please check if the
            // input clock source is valid.
            //
        }

        //
        // Increment the counter
        //
        loopCount++;
    }while(loopCount < 4U);
}
#endif

//*****************************************************************************
//
// SysCtl_getClock()
//
//*****************************************************************************
uint32_t
SysCtl_getClock(uint32_t clockInHz)
{
    __attribute__((unused)) uint32_t temp;
    uint32_t oscSource;
    uint32_t clockOut;

    //
    // Don't proceed if an MCD failure is detected.
    //
    if(SysCtl_isMCDClockFailureDetected())
    {
        //
        // OSCCLKSRC2 failure detected. Returning the INTOSC1 rate. You need
        // to handle the MCD and clear the failure.
        //
        clockOut = SYSCTL_DEFAULT_OSC_FREQ;
    }
    else
    {
        //
        // If one of the internal oscillators is being used, start from the
        // known default frequency.  Otherwise, use clockInHz parameter.
        //
#ifndef __TMS320C2000__
        oscSource = HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
                    (uint32_t)SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

        if((oscSource == ((uint32_t)SYSCTL_OSCSRC_OSC2 >> SYSCTL_OSCSRC_S)) ||
           (oscSource == ((uint32_t)SYSCTL_OSCSRC_OSC1 >> SYSCTL_OSCSRC_S)))
#else
        // oscSource = SysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL | SysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL;
        oscSource = pSysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL;

        if(oscSource != 0)
#endif
        {
            clockOut = SYSCTL_DEFAULT_OSC_FREQ;
        }
        else
        {
            clockOut = clockInHz;
        }

        //
        // If the PLL is enabled calculate its effect on the clock
        //
#ifndef __TMS320C2000__
        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &
            (SYSCTL_SYSPLLCTL1_PLLEN | SYSCTL_SYSPLLCTL1_PLLCLKEN)) == 3U)
        {
            //
            // Calculate portion from fractional multiplier
            //
            temp = (clockInHz * ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                                  SYSCTL_SYSPLLMULT_FMULT_M) >>
                                 SYSCTL_SYSPLLMULT_FMULT_S)) / 4U;

            //
            // Calculate integer multiplier and fixed divide by 2
            //
            clockOut = clockOut * ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                                    SYSCTL_SYSPLLMULT_IMULT_M) >>
                                   SYSCTL_SYSPLLMULT_IMULT_S);

            //
            // Add in fractional portion
            //
            clockOut += temp;
        }

        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
            SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) != 0U)
        {
            clockOut /= (2U * (HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                               SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M));
        }
#else
        if(pSysCtrlRegs->PLLCR.bit.DIV != 0)
        {
            //
            // Calculate portion from fractional multiplier
            //
            temp = (clockInHz * pSysCtrlRegs->PLLSTS.bit.DIVSEL) / 4U;

            //
            // Calculate integer multiplier and fixed divide by 2
            //
            clockOut = clockOut * pSysCtrlRegs->PLLCR.bit.DIV / 4;

            //
            // Add in fractional portion
            //
            // ???clockOut += temp;
        }
#endif
    }

    return(clockOut);
}

//*****************************************************************************
//
// SysCtl_setClock()
//
//*****************************************************************************
bool
SysCtl_setClock(uint32_t config)
{
    uint16_t divSel;
    uint16_t pllMult;
    __attribute__((unused)) uint32_t retries, oscSource, pllLockStatus;
    __attribute__((unused))     uint32_t timeout;
    bool status = false;
    uint16_t mult;

    //
    // Check the arguments.
    //
    ASSERT((config & SYSCTL_OSCSRC_M) <= SYSCTL_OSCSRC_M);

    //
    // Don't proceed to the PLL initialization if an MCD failure is detected.
    //
    if(SysCtl_isMCDClockFailureDetected())
    {
        //
        // OSCCLKSRC2 failure detected. Returning false. You'll need to clear
        // the MCD error.
        //
        status = false;
    }
    else
    {
        //
        // Configure oscillator source
        //
        oscSource = config & SYSCTL_OSCSRC_M;
        SysCtl_selectOscSource(oscSource);

#ifndef __TMS320C2000__
        //
        // Bypass PLL
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
            ~SYSCTL_SYSPLLCTL1_PLLCLKEN;
        EDIS;

        //
        // Delay of at least 60 OSCCLK cycles required post PLL bypass
        //
        SysCtl_delay(11U);
#endif

        //
        // Get the PLL multiplier settings from config
        //
        pllMult  = (uint16_t)((config & SYSCTL_IMULT_M) <<
                              SYSCTL_SYSPLLMULT_IMULT_S);

        pllMult |= (uint16_t)(((config & SYSCTL_FMULT_M) >>
                               SYSCTL_FMULT_S) <<
                              SYSCTL_SYSPLLMULT_FMULT_S);

        ASSERT(pllMult==18);
        //
        // Get the PLL multipliers currently programmed
        //
#ifndef __TMS320C2000__
        mult  = (uint16_t)((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                            (uint32_t)SYSCTL_SYSPLLMULT_IMULT_M) >>
                           (uint32_t)SYSCTL_SYSPLLMULT_IMULT_S);

        mult |= (uint16_t)(HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) &
                                 SYSCTL_SYSPLLMULT_FMULT_M);
#else
        mult  = pSysCtrlRegs->PLLCR.bit.DIV;
#endif

        //
        // Lock PLL only if the multipliers need update
        //
        if(mult !=  pllMult)
        {
            //
            // Configure PLL if enabled
            //
            if((config & SYSCTL_PLL_ENABLE) == SYSCTL_PLL_ENABLE)
            {
                //
                // Set dividers to /1
                //
                EALLOW;
#ifndef __TMS320C2000__
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0U;
#else
                // DIVSEL MUST be 0 before PLLCR can be changed from
                // 0x0000. It is set to 0 by an external reset XRSn
                // This puts us in 1/4
                if (pSysCtrlRegs->PLLSTS.bit.DIVSEL != 0)
                {
                    pSysCtrlRegs->PLLSTS.bit.DIVSEL = 0;
                }
#endif
                EDIS;

#ifdef __TMS320C2000__
                //
                // Before setting PLLCR turn off missing clock detect logic
                //
                EALLOW;
                pSysCtrlRegs->PLLSTS.bit.MCLKOFF = 1;
                pSysCtrlRegs->PLLCR.bit.DIV = pllMult;
                EDIS;

                while(pSysCtrlRegs->PLLSTS.bit.PLLLOCKS != 1) {
                    // wait..
                }

                EALLOW;
                pSysCtrlRegs->PLLSTS.bit.MCLKOFF = 0;
                EDIS;

                status = true; // CHECK PLL?
#else
                //
                // Loop to retry locking the PLL should the DCC module
                // indicate that it was not successful.
                //
                for(retries = 0U; (retries < SYSCTL_PLL_RETRIES); retries++)
                {
                    //
                    // Turn off PLL
                    //
                    EALLOW;
                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &=
                        ~SYSCTL_SYSPLLCTL1_PLLEN;

                    SysCtl_delay(3U);

                    //
                    // Write multiplier, which automatically turns on the PLL
                    //
                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) = pllMult;

                    //
                    // Wait for the SYSPLL lock counter or a timeout
                    //
                    timeout = SYSCTL_PLLLOCK_TIMEOUT;
                    pllLockStatus = HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLSTS) &
                                    SYSCTL_SYSPLLSTS_LOCKS;

                    while((pllLockStatus != 1U) && (timeout != 0U))
                    {
                        pllLockStatus = HWREGH(CLKCFG_BASE +
                                               SYSCTL_O_SYSPLLSTS) &
                                        SYSCTL_SYSPLLSTS_LOCKS;
                        timeout--;
                    }
                    EDIS;

                    //
                    // Check PLL Frequency using DCC
                    //
                    status = SysCtl_isPLLValid(oscSource,
                                              (config &
                                              ((uint32_t)SYSCTL_IMULT_M |
                                               (uint32_t)SYSCTL_FMULT_M)));

                    //
                    // Check DCC Status, if no error break the loop
                    //
                    if(status)
                    {
                        break;
                    }
                }
#endif
            }
            else
            {
                status = true;
            }
        }
      else
        {
            status = true;
        }


        //
        // If PLL locked successfully, configure the dividers
        //
        if(status)
        {
            //
            // Set divider to produce slower output frequency to limit current
            // increase.
            //
            divSel = (uint16_t)(config & SYSCTL_SYSDIV_M) >> SYSCTL_SYSDIV_S;

#ifdef __TMS320C2000__
            //
            // If switching to 1/2
            //
            if((divSel == 1)||(divSel == 2))
            {
                EALLOW;
                pSysCtrlRegs->PLLSTS.bit.DIVSEL = divSel;
                EDIS;
            }

            //
            // If switching to 1/1
            // * First go to 1/2 and let the power settle
            //   The time required will depend on the system, this is only an example
            // * Then switch to 1/1
            //
            if(divSel == 3)
            {
                EALLOW;
                pSysCtrlRegs->PLLSTS.bit.DIVSEL = 2;
                SysCtl_delay(50L);
                pSysCtrlRegs->PLLSTS.bit.DIVSEL = 3;
                EDIS;
            }
#else
            EALLOW;
            if(divSel != (126U / 2U))
            {
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(uint16_t)SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) |
                    (divSel + 1U);
            }
            else
            {
                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                    (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                     ~(uint16_t)SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | divSel;
            }

            EDIS;

            //
            // Enable PLLSYSCLK is fed from system PLL clock
            //
            EALLOW;
            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |=
                SYSCTL_SYSPLLCTL1_PLLCLKEN;
            EDIS;

            //
            // ~200 PLLSYSCLK delay to allow voltage regulator to stabilize
            // prior to increasing entire system clock frequency.
            //
            SysCtl_delay(40U);

            //
            // Set the divider to user value
            //
            EALLOW;
            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) =
                (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | divSel;
            EDIS;
#endif
        }
    }

    ASSERT(status);

    return(status);
}
//*****************************************************************************
//
// SysCtl_selectXTAL()
//
//*****************************************************************************
void
SysCtl_selectXTAL(void)
{

#ifndef __TMS320C2000__
    //
    // Turn on XTAL and select crystal mode
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_SE;
    EDIS;

    //
    // Wait for the X1 clock to saturate
    //
    SysCtl_pollX1Counter();

    //
    // Turn off XCLKIN
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XCLKCR_OSCOFF;

    //
    // Select XTAL as the oscillator source
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
    ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
      (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
     ((uint32_t)SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
    EDIS;

    //
    // If a missing clock failure was detected, try waiting for the X1 counter
    // to saturate again. Consider modifying this code to add a 10ms timeout.
    //
    while(SysCtl_isMCDClockFailureDetected())
    {
        //
        // Clear the MCD failure
        //
        SysCtl_resetMCD();

        //
        // Wait for the X1 clock to saturate
        //
        SysCtl_pollX1Counter();

        //
        // Select XTAL as the oscillator source
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
        ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
          (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
         ((uint32_t)SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
        EDIS;
    }
#else
    EALLOW;
    pSysCtrlRegs->CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC

    //
    // Wait for 1ms while XTAL starts up
    //
    DEVICE_DELAY_US(1000u);

    pSysCtrlRegs->CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
    pSysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL = 0;  // Switch to external clock

    //
    // Switch from INTOSC1 to INTOSC2/ext clk
    //
    pSysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL = 1;

    //
    // Clock Watchdog off of INTOSC1 always
    //
    pSysCtrlRegs->CLKCTL.bit.WDCLKSRCSEL = 0;

    pSysCtrlRegs->CLKCTL.bit.INTOSC2OFF = 1;     // Turn off INTOSC2
    pSysCtrlRegs->CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
    EDIS;
#endif
}

//*****************************************************************************
//
// SysCtl_selectXTALSingleEnded()
//
//*****************************************************************************
#ifndef __TMS320C2000__
void
SysCtl_selectXTALSingleEnded(void)
{
    //
    // Turn on XTAL and select single-ended mode.
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) &= ~SYSCTL_XTALCR_OSCOFF;
    HWREGH(CLKCFG_BASE + SYSCTL_O_XTALCR) |= SYSCTL_XTALCR_SE;
    EDIS;

    //
    // Wait for the X1 clock to saturate
    //
    SysCtl_pollX1Counter();

    //
    // Select XTAL as the oscillator source
    //
    EALLOW;
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
    ((HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
      (~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M)) |
     ((uint32_t)SYSCTL_OSCSRC_XTAL >> SYSCTL_OSCSRC_S));
    EDIS;

    //
    // Something is wrong with the oscillator module. Replace the ESTOP0 with
    // an appropriate error-handling routine.
    //
    while(SysCtl_isMCDClockFailureDetected())
    {
        ESTOP0;
    }
}
#endif

//*****************************************************************************
//
// SysCtl_selectOscSource()
//
//*****************************************************************************
void
SysCtl_selectOscSource(uint32_t oscSource)
{
    ASSERT((oscSource == SYSCTL_OSCSRC_OSC1) |
           (oscSource == SYSCTL_OSCSRC_OSC2) |
           (oscSource == SYSCTL_OSCSRC_XTAL)
#ifndef __TMS320C2000__
           | (oscSource == SYSCTL_OSCSRC_XTAL_SE)
#endif
           );

    //
    // Select the specified source.
    //
    EALLOW;
    switch(oscSource)
    {
#ifndef __TMS320C2000__
        case SYSCTL_OSCSRC_OSC2:
            //
            // Turn on INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                ~SYSCTL_CLKSRCCTL1_INTOSC2OFF;

            SYSCTL_CLKSRCCTL1_DELAY;

            //
            // Clk Src = INTOSC2
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &=
                ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

            break;

        case SYSCTL_OSCSRC_XTAL:
            //
            // Select XTAL in crystal mode and wait for it to power up
            //
            SysCtl_selectXTAL();
            break;

        case SYSCTL_OSCSRC_XTAL_SE:
            //
            // Select XTAL in single-ended mode and wait for it to power up
            //
            SysCtl_selectXTALSingleEnded();
            break;

        case SYSCTL_OSCSRC_OSC1:
            //
            // Clk Src = INTOSC1
            //
            HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) =
                   (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &
                    ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M) |
                   ((uint32_t)SYSCTL_OSCSRC_OSC1 >> SYSCTL_OSCSRC_S);

            break;
#else
        case SYSCTL_OSCSRC_OSC2:
            ASSERT(false); // TODO
            break;

        case SYSCTL_OSCSRC_XTAL:
            //
            // Select XTAL in crystal mode and wait for it to power up
            //
            SysCtl_selectXTAL();
            break;

        case SYSCTL_OSCSRC_OSC1:
            ASSERT(false); // TODO
            break;
#endif
        default:
            //
            // Do nothing. Not a valid oscSource value.
            //
            break;
    }
    EDIS;
}

//*****************************************************************************
//
// SysCtl_getLowSpeedClock()
//
//*****************************************************************************
uint32_t
SysCtl_getLowSpeedClock(uint32_t clockInHz)
{
    uint32_t clockOut;

    //
    // Get the main system clock
    //
    clockOut = SysCtl_getClock(clockInHz);

    //
    // Apply the divider to the main clock
    //
#ifndef _TMS320C2000
    if((HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) &
        SYSCTL_LOSPCP_LSPCLKDIV_M) != 0U)
    {
        clockOut /= (2U * (HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) &
                            SYSCTL_LOSPCP_LSPCLKDIV_M));
    }
#else
    if(pSysCtrlRegs->LOSPCP.bit.LSPCLK != 0)
    {
        //clockOut /= (2U * SysCtrlRegs->LOSPCP.bit.LSPCLK);
    }
#endif

    return(clockOut);
}

//*****************************************************************************
//
// SysCtl_getDeviceParametric()
//
//*****************************************************************************
uint16_t
SysCtl_getDeviceParametric(SysCtl_DeviceParametric parametric)
{
    uint32_t value;
#ifdef __TMS320C2000__
    ASSERT(false);
#endif
    //
    // Get requested parametric value
    //
    switch(parametric)
    {
        case SYSCTL_DEVICE_QUAL:
            //
            // Qualification Status
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_QUAL_M) >> SYSCTL_PARTIDL_QUAL_S);
            break;

        case SYSCTL_DEVICE_PINCOUNT:
            //
            // Pin Count
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_PIN_COUNT_M) >>
                     SYSCTL_PARTIDL_PIN_COUNT_S);
            break;

        case SYSCTL_DEVICE_INSTASPIN:
            //
            // InstaSPIN Feature Set
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_INSTASPIN_M) >>
                     SYSCTL_PARTIDL_INSTASPIN_S);
            break;

        case SYSCTL_DEVICE_FLASH:
            //
            // Flash Size (KB)
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                      SYSCTL_PARTIDL_FLASH_SIZE_M) >>
                     SYSCTL_PARTIDL_FLASH_SIZE_S);
            break;
        case SYSCTL_DEVICE_FAMILY:
            //
            // Device Family
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_FAMILY_M) >> SYSCTL_PARTIDH_FAMILY_S);
            break;

        case SYSCTL_DEVICE_PARTNO:
            //
            // Part Number
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_PARTNO_M) >> SYSCTL_PARTIDH_PARTNO_S);
            break;

        case SYSCTL_DEVICE_CLASSID:
            //
            // Class ID
            //
            value = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) &
                      SYSCTL_PARTIDH_DEVICE_CLASS_ID_M) >>
                     SYSCTL_PARTIDH_DEVICE_CLASS_ID_S);
            break;

        default:
            //
            // Not a valid value for PARTID register
            //
            value = 0U;
            break;
    }

    return((uint16_t)value);
}
//*****************************************************************************
//
// SysCtl_isPLLValid()
//
//*****************************************************************************
#ifndef __TMS320C2000__
bool
SysCtl_isPLLValid(uint32_t oscSource, uint32_t pllMult)
{
    uint32_t imult, fmult, base;

    DCC_Count0ClockSource dccClkSrc0;
    DCC_Count1ClockSource dccClkSrc1;
    uint32_t dccCounterSeed0, dccCounterSeed1, dccValidSeed0;

    switch(oscSource)
    {
        case SYSCTL_OSCSRC_OSC2:
            //
            // Select DCC Clk Src0 as INTOSC2
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC2;
            break;
        case SYSCTL_OSCSRC_XTAL:
        case SYSCTL_OSCSRC_XTAL_SE:
            //
            // Select DCC Clk Src0 as XTAL
            //
            dccClkSrc0 = DCC_COUNT0SRC_XTAL;
            break;
        case SYSCTL_OSCSRC_OSC1:
            //
            // Select DCC Clk Src0 as INTOSC1
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC1;
            break;
        default:
            //
            // Select DCC Clk Src0 as INTOSC1
            //
            dccClkSrc0 = DCC_COUNT0SRC_INTOSC1;
            break;
    }

    //
    // Setting Counter0 & Valid Seed Value with +/-2% tolerance
    //
    dccCounterSeed0 = (uint32_t)SYSCTL_DCC_COUNTER0_WINDOW - 2U;
    dccValidSeed0 = 4U;

    //
    // Select DCC0 for PLL validation
    //
    base = DCC0_BASE;

    //
    // Select DCC Clk Src1 as SYSPLL
    //
    dccClkSrc1 = DCC_COUNT1SRC_PLL;

    imult = pllMult & SYSCTL_IMULT_M;
    fmult = pllMult & SYSCTL_FMULT_M;

    //
    // Multiplying Counter-0 window with PLL Integer Multiplier
    //
    dccCounterSeed1 = SYSCTL_DCC_COUNTER0_WINDOW * imult;

    //
    // Multiplying Counter-0 window with PLL Fractional Multiplier
    //
    switch(fmult)
    {
        case SYSCTL_FMULT_1_4:
            //
            // FMULT * CNTR0 Window = 0.25 * 100 = 25, gets added to cntr0
            // seed value
            //
            dccCounterSeed1 = dccCounterSeed1 + 25U;
            break;
        case SYSCTL_FMULT_1_2:
            //
            // FMULT * CNTR0 Window = 0.5 * 100 = 50, gets added to cntr0
            // seed value
            //
            dccCounterSeed1 = dccCounterSeed1 + 50U;
            break;
        case SYSCTL_FMULT_3_4:
            //
            // FMULT * CNTR0 Window = 0.75 * 100 = 75, gets added to cntr0
            // seed value
            //
            dccCounterSeed1 = dccCounterSeed1 + 75U;
            break;
        default:
            //
            // No fractional multiplier
            //
            dccCounterSeed1 = dccCounterSeed1;
            break;
    }


    //
    // Enable peripheral clock to DCC
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);

    //
    // Clear Error & Done Flag
    //
    DCC_clearErrorFlag(base);
    DCC_clearDoneFlag(base);

    //
    // Disable DCC
    //
    DCC_disableModule(base);

    //
    // Disable Error Signal
    //
    DCC_disableErrorSignal(base);

    //
    // Disable Done Signal
    //
    DCC_disableDoneSignal(base);

    //
    // Configure Clock Source0 to whatever set as a clock source for PLL
    //
    DCC_setCounter0ClkSource(base, dccClkSrc0);

    //
    // Configure Clock Source1 to PLL
    //
    DCC_setCounter1ClkSource(base, dccClkSrc1);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(base, dccCounterSeed0, dccValidSeed0,
                        dccCounterSeed1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(base, DCC_MODE_COUNTER_ZERO);

    //
    // Enable Error Signal
    //
    DCC_enableErrorSignal(base);

    //
    // Enable Done Signal
    //
    DCC_enableDoneSignal(base);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(base);

    //
    // Wait until Error or Done Flag is generated
    //
    while((HWREGH(base + DCC_O_STATUS) &
           (DCC_STATUS_ERR | DCC_STATUS_DONE)) == 0U)
    {
    }

    //
    // Returns true if DCC completes without error
    //
    return((HWREGH(base + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE);
}
#endif

