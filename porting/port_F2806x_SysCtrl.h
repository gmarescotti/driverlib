//###########################################################################
//
// FILE:   F2806x_Sysctrl.h
//
// TITLE:  F2806x Device SYSCTRL Register Definitions.
//
//###########################################################################
// $TI Release: F2806x Support Library v2.04.00.00 $
// $Release Date: Thu Oct 18 15:47:20 CDT 2018 $
// $Copyright:
// Copyright (C) 2009-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef F2806x_SYSCTRL_H
#define F2806x_SYSCTRL_H

#ifdef __cplusplus
extern "C" {
#endif


//
// SYSCTRL Individual Register Bit Definitions:
//
struct XCLK_BITS
{
    uint16_t	XCLKOUTDIV:2;			// 1:0 	XCLKOUT Divide Ratio
	uint16_t	rsvd1:4;				// 5:2	Reserved
	uint16_t	XCLKINSEL:1;			// 6	XCLKIN Source Select
	uint16_t	rsvd2:9;				// 15:7	Reserved
};

union XCLK_REG
{
	uint16_t all;
	struct XCLK_BITS	bit;
};

struct PLLSTS_BITS
{
	uint16_t	PLLLOCKS:1;			// 0	PLL lock status
	uint16_t	rsvd1:1;			// 1	Reserved
	uint16_t	PLLOFF:1;			// 2	PLL off bit
	uint16_t	MCLKSTS:1;			// 3	Missing clock status bit
	uint16_t	MCLKCLR:1;			// 4	Missing clock clear bit
	uint16_t	OSCOFF:1;			// 5	Oscillator clock off
	uint16_t	MCLKOFF:1;			// 6	Missing clock detect
	uint16_t	DIVSEL:2;			// 8:7	Divide select (/4 default)
	uint16_t	rsvd2:6;			// 14:9	Reserved
	uint16_t	NORMRDYE:1;			// 15	VREG NORMRDY enable bit
};

union PLLSTS_REG
{
    uint16_t all;
	struct PLLSTS_BITS	bit;
};

struct CLKCTL_BITS
{
	uint16_t	OSCCLKSRCSEL:1;	    // 0	Oscillator clock source select bit
	uint16_t	OSCCLKSRC2SEL:1;	// 1	Oscillator 2 clock source select bit
	uint16_t	WDCLKSRCSEL:1;		// 2	Watchdog clock source select bit
	uint16_t	TMR2CLKSRCSEL:2;	// 4:3	CPU timer 2 clock source select bit
	uint16_t	TMR2CLKPRESCALE:3;	// 7:5	CPU timer 2 clock pre-scale value
	uint16_t	INTOSC1OFF:1;		// 8	Internal oscillator off bit
	
    //
    // 9	Internal oscillator 1 halt mode ignore bit
    //
    uint16_t	INTOSC1HALTI:1;
    
	uint16_t	INTOSC2OFF:1;		// 10	Internal oscillator 2 off bit
	
    //
    // 11	Internal oscillator 2 halt mode ignore bit
    //
    uint16_t	INTOSC2HALTI:1;
    
	uint16_t	WDHALTI:1;			// 12	Watchdog halt mode ignore bit
	uint16_t	XCLKINOFF:1;		// 13	XCLKIN off bit
	uint16_t	XTALOSCOFF:1;		// 14	Crystal (External) oscillator off bit
	uint16_t	NMIRESETSEL:1;		// 15	NMI reset select bit
};

union CLKCTL_REG
{
	uint16_t all;
	struct CLKCTL_BITS	bit;
};

struct INTOSC1TRIM_BITS
{
	uint16_t	COARSETRIM:8;		// 7:0	8-bit coarse trim value
	uint16_t	rsvd1:1;			// 8	Reserved
	uint16_t	FINETRIM:6;			// 14:9	6-bit fine trim value
	uint16_t	rsvd2:1;			// 15	Reserved
};

union INTOSC1TRIM_REG
{
	uint16_t all;
	struct INTOSC1TRIM_BITS	bit;
};

struct INTOSC2TRIM_BITS
{
	uint16_t	COARSETRIM:8;		// 7:0	8-bit coarse trim value
	uint16_t	rsvd1:1;			// 8	Reserved
	uint16_t	FINETRIM:6;			// 14:9	6-bit fine trim value
	uint16_t	rsvd2:1;			// 15	Reserved
};

union INTOSC2TRIM_REG
{
	uint16_t all;
	struct INTOSC2TRIM_BITS	bit;
};

struct PCLKCR2_BITS
{
	uint16_t	rsvd1:8;			// 7:0	Reserved
	uint16_t	HRCAP1ENCLK:1;		// 8 	HRCAP1 Clock Enable
	uint16_t	HRCAP2ENCLK:1;		// 9	HRCAP2 Clock Enable
	uint16_t	HRCAP3ENCLK:1;		// 10	HRCAP3 Clock Enable
	uint16_t	HRCAP4ENCLK:1;		// 11	HRCAP4 Clock Enable
	uint16_t	rsvd2:4;			// 15:12 Reserved
};

union PCLKCR2_REG
{
	uint16_t all;
	struct PCLKCR2_BITS	bit;
};

struct LOSPCP_BITS
{
	uint16_t	LSPCLK:3;			// 2:0	Rate relative to SYSCLKOUT
	uint16_t	rsvd1:13;			// 15:3	Reserved
};

union LOSPCP_REG
{
	uint16_t all;
	struct LOSPCP_BITS	bit;
};

struct PCLKCR0_BITS
{
	uint16_t	HRPWMENCLK:1;		// 0	HRPWM Clock Enable
	uint16_t	rsvd1:1;			// 1	Reserved
	uint16_t	TBCLKSYNC:1;		// 2	EWPM Module TBCLK enable/sync
	uint16_t	ADCENCLK:1;			// 3	Enable high speed clk to ADC1
	uint16_t	I2CAENCLK:1;		// 4	I2C-A Clock Enable
	uint16_t	rsvd2:1;			// 5	Reserved
	uint16_t	rsvd3:2;			// 7:6	Reserved
	uint16_t	SPIAENCLK:1;		// 8	SPI A Clock Enable
	uint16_t	SPIBENCLK:1;		// 9	SPI B Clock Enable
	uint16_t	SCIAENCLK:1;		// 10	SCI A Clock Enable
	uint16_t	SCIBENCLK:1;		// 11	SCI B Clock Enable
	uint16_t	MCBSPAENCLK:1;		// 12	McBSP-A Clock Enable
	uint16_t	rsvd4:1;			// 13	Reserved
	uint16_t	ECANAENCLK:1;		// 14	CAN A Clock Enable
	uint16_t	rsvd5:1;			// 15	Reserved
};

union PCLKCR0_REG
{
	uint16_t all;
	struct PCLKCR0_BITS	bit;
};

struct PCLKCR1_BITS
{
	uint16_t	EPWM1ENCLK:1;			// 0	EPWM1 Clock Enable
	uint16_t	EPWM2ENCLK:1;			// 1	EPWM2 Clock Enable
	uint16_t	EPWM3ENCLK:1;			// 2	EPWM3 Clock Enable
	uint16_t	EPWM4ENCLK:1;			// 3	EPWM4 Clock Enable
	uint16_t	EPWM5ENCLK:1;			// 4	EPWM5 Clock Enable
	uint16_t	EPWM6ENCLK:1;			// 5	EPWM6 Clock Enable
	uint16_t	EPWM7ENCLK:1;			// 6	EPWM7 Clock Enable
	uint16_t	EPWM8ENCLK:1;			// 7	EPWM8 Clock Enable
	uint16_t	ECAP1ENCLK:1;			// 8	ECAP1 Clock Enable
	uint16_t	ECAP2ENCLK:1;			// 9	ECAP2 Clock Enable
	uint16_t	ECAP3ENCLK:1;			// 10	ECAP3 Clock Enable
	uint16_t	rsvd1:3;				// 13:11	Reserved
	uint16_t	EQEP1ENCLK:1;			// 14	EQEP1 Clock Enable
	uint16_t	EQEP2ENCLK:1;			// 15	EQEP2 Clock Enable
};

union PCLKCR1_REG
{
	uint16_t all;
	struct PCLKCR1_BITS	bit;
};

struct PCLKCR3_BITS
{
	uint16_t	COMP1ENCLK:1;			// 0	COMP1 and DAC1 Clock Enable
	uint16_t	COMP2ENCLK:1;			// 1	COMP2 and DAC2 Clock Enable
	uint16_t	COMP3ENCLK:1;			// 2	COMP3 and DAC3 Clock Enable
	uint16_t	rsvd1:5;				// 7:3	Reserved
	uint16_t	CPUTIMER0ENCLK:1;		// 8	Enable SYSCLKOUT to CPUTIMER0
	uint16_t	CPUTIMER1ENCLK:1;		// 9	Enable SYSCLKOUT to CPUTIMER1
	uint16_t	CPUTIMER2ENCLK:1;		// 10	Enable SYSCLKOUT to CPUTIMER2
	uint16_t	DMAENCLK:1;				// 11	DMA Clock Enable
	uint16_t	rsvd2:1;				// 12	Reserved
	uint16_t	rsvd3:1;				// 13	Reserved
	uint16_t	CLA1ENCLK:1;			// 14	CLA Clock Enable
	uint16_t	USB0ENCLK:1;			// 15	USB0 Clock Enable
};

union PCLKCR3_REG
{
	uint16_t all;
	struct PCLKCR3_BITS	bit;
};

struct PLLCR_BITS
{
	uint16_t	DIV:5;				// 4:0	Set Clock ratio for the PLL
	uint16_t	rsvd1:11;			// 15:4	Reserved
};

union PLLCR_REG
{
	uint16_t all;
	struct PLLCR_BITS	bit;
};

struct JTAGDEBUG_BITS
{
	uint16_t	JTAGDIS:1;		// JTAG Port Disable Bit
	uint16_t	rsvd1:15;		// Reserved
};

union JTAGDEBUG_REG
{
	uint16_t	all;
	struct	JTAGDEBUG_BITS	bit;
};

struct LPMCR0_BITS
{
	uint16_t	LPM:2;				// 1:0	Set the low power mode
	uint16_t	QUALSTDBY:6;		// 7:2	Qualification
	uint16_t	rsvd1:7;			// 14:8	Reserved
	
    //
    // 15	Enables WD to wake the device from STANDBY
    //
    uint16_t	WDINTE:1;
};

union LPMCR0_REG
{
	uint16_t all;
	struct LPMCR0_BITS	bit;
};

struct PLL2CTL_BITS
{
	uint16_t	PLL2CLKSRCSEL:2;	// 1:0	PLL2 Clock Source Select Bits
	uint16_t	PLL2EN:1;			// 2	PLL2 Enable Bit
	uint16_t	rsvd1:13;			// 15:3	Reserved
};

union PLL2CTL_REG
{
	uint16_t all;
	struct PLL2CTL_BITS	bit;
};

struct PLL2MULT_BITS
{
	uint16_t	PLL2MULT:4;		// 3:0	PLL2 Integer Multiplier
	uint16_t	rsvd1:12;		// 15:4	Reserved
};

union PLL2MULT_REG
{
	uint16_t all;
	struct PLL2MULT_BITS	bit;
};

struct PLL2STS_BITS
{
	uint16_t	PLL2LOCKS:1;	// 0	PLL2 Lock Status Bit
	uint16_t	rsvd1:15;		// 15:1	Reserved
};

union PLL2STS_REG
{
	uint16_t all;
	struct PLL2STS_BITS	bit;
};

struct EPWMCFG_BITS
{
	uint16_t	CONFIG:1;			// 0	EPWM to DMA/CLA Enable Bit
	uint16_t	rsvd1:15;			// 15:1	Reserved
};

union EPWMCFG_REG
{
    uint16_t all;
	struct EPWMCFG_BITS	bit;
};

//
// System Power Control Registers
//

//
// BOR configuration register bit definitions
//
struct BORCFG_BITS
{
    uint16_t BORENZ:1;      // 0     BOR enable active low bit
    uint16_t rsvd1:15;      // 15:1  reserved
};

union BORCFG_REG
{
    uint16_t              all;
    struct BORCFG_BITS  bit;
};

struct SYS_PWR_CTRL_REGS
{
    union    BORCFG_REG   BORCFG;       // 0 BOR Configuration Register
    uint16_t   rsvd1[2];                  // 1-2
};

//
// CSM Status & Control register bit definitions
//
struct  CSMSCR_BITS
{
    uint16_t     SECURE:1;    // 0     Secure flag
    uint16_t     rsvd1:14;    // 14:1  Reserved
    uint16_t     FORCESEC:1;  // 15    Force Secure control bit
};

//
// Allow access to the bit fields or entire register
//
union CSMSCR_REG
{
    uint16_t             all;
    struct CSMSCR_BITS bit;
};

//
// CSM Register File
//
struct  CSM_REGS
{
    uint16_t           KEY0;    // KEY reg bits 15-0
    uint16_t           KEY1;    // KEY reg bits 31-16
    uint16_t           KEY2;    // KEY reg bits 47-32
    uint16_t           KEY3;    // KEY reg bits 63-48
    uint16_t           KEY4;    // KEY reg bits 79-64
    uint16_t           KEY5;    // KEY reg bits 95-80
    uint16_t           KEY6;    // KEY reg bits 111-96
    uint16_t           KEY7;    // KEY reg bits 127-112
    uint16_t           rsvd1;   // Reserved
    uint16_t           rsvd2;   // Reserved
    uint16_t           rsvd3;   // Reserved
    uint16_t           rsvd4;   // Reserved
    uint16_t           rsvd5;   // Reserved
    uint16_t           rsvd6;   // Reserved
    uint16_t           rsvd7;   // Reserved
    union CSMSCR_REG CSMSCR;  // CSM Status & Control register
};

//
// Password locations
//
struct  CSM_PWL
{
    uint16_t   PSWD0;  // PSWD bits 15-0
    uint16_t   PSWD1;  // PSWD bits 31-16
    uint16_t   PSWD2;  // PSWD bits 47-32
    uint16_t   PSWD3;  // PSWD bits 63-48
    uint16_t   PSWD4;  // PSWD bits 79-64
    uint16_t   PSWD5;  // PSWD bits 95-80
    uint16_t   PSWD6;  // PSWD bits 111-96
    uint16_t   PSWD7;  // PSWD bits 127-112
};

//
// Flash Registers
//
#define FLASH_SLEEP   0x0000;
#define FLASH_STANDBY 0x0001;
#define FLASH_ACTIVE  0x0003;

struct FOPT_BITS
{
    uint16_t	ENPIPE:1;			// 0	Enable Pipeline Mode
	uint16_t	rsvd1:15;			// 15:1	Reserved
};

union FOPT_REG
{
	uint16_t all;
	struct FOPT_BITS	bit;
};

struct FPWR_BITS
{
	uint16_t	PWR:2;				// 1:0	Power Mode Bits
	uint16_t	rsvd1:14;			// 15:2	Reserved
};

union FPWR_REG
{
	uint16_t all;
	struct FPWR_BITS	bit;
};

struct FSTATUS_BITS
{
    uint16_t	PWRS:2;				// 1:0	Power Mode Status Bits
	
    //
    // 2	Bank/Pump Sleep to Standby Wait Counter Status Bits
    //
    uint16_t	STDBYWAITS:1;
    
    //
	// 3	Bank/Pump Standby to Active Wait Counter Status Bits
    //
    uint16_t	ACTIVEWAITS:1;
    
	uint16_t	rsvd1:4;			// 7:4	Reserved
	uint16_t	V3STAT:1;			// 8	VDD3V Status Latch Bit
	uint16_t	rsvd2:7;			// 15:9	Reserved
};

union FSTATUS_REG
{
	uint16_t all;
	struct FSTATUS_BITS	bit;
};

struct FSTDBYWAIT_BITS
{
	uint16_t	STDBYWAIT:9;	// 8:0	Bank/Pump Sleep to Standby Wait Count Bits
    uint16_t	rsvd1:7;		// 15:9	Reserved
};

union FSTDBYWAIT_REG
{
    uint16_t all;
	struct FSTDBYWAIT_BITS	bit;
};

struct FACTIVEWAIT_BITS
{
	uint16_t	ACTIVEWAIT:9;	// 8:0	Bank/Pump Standby to Active Wait Count Bits
	uint16_t	rsvd1:7;		// 15:9	Reserved
};

union FACTIVEWAIT_REG
{
    uint16_t all;
	struct FACTIVEWAIT_BITS	bit;
};

struct FBANKWAIT_BITS
{
	uint16_t	RANDWAIT:4;			// 3:0	Flash Random Read Wait State Bits
	uint16_t	rsvd1:4;			// 7:4	Reserved
	uint16_t	PAGEWAIT:4;			// 11:8	Flash Paged Read Wait State Bits
	uint16_t	rsvd2:4;			// 15:12	Reserved
};

union FBANKWAIT_REG
{
	uint16_t all;
	struct FBANKWAIT_BITS	bit;
};

struct FOTPWAIT_BITS
{
	uint16_t	OTPWAIT:5;			// 4:0	OTP Read Wait State Bits
	uint16_t	rsvd1:11;			// 15:5	Reserved
};

union FOTPWAIT_REG
{
	uint16_t all;
	struct FOTPWAIT_BITS	bit;
};

struct SYS_CTRL_REGS
{
	union	XCLK_REG		XCLK;		 // XCLKOUT Control
	union	PLLSTS_REG		PLLSTS;		 // PLL Status Register
	union	CLKCTL_REG		CLKCTL;		 // Clock Control Register
	uint16_t					PLLLOCKPRD;	 // PLL Lock Period Register
	union	INTOSC1TRIM_REG	INTOSC1TRIM; // Internal Oscillator 1 Trim Register
	uint16_t					rsvd1;		 // Reserved
	union	INTOSC2TRIM_REG	INTOSC2TRIM; // Internal Oscillator 2 Trim
	uint16_t					rsvd2[2];	 // Reserved
	union	PCLKCR2_REG		PCLKCR2;	 // Peripheral Clock Control Regsiter 2
	uint16_t					rsvd3;		 // Reserved
	
    //
    // Low-Speed Peripheral Clock Pre-Scaler Register
    //
    union	LOSPCP_REG		LOSPCP;		 
    
	union	PCLKCR0_REG		PCLKCR0;	 // Peripheral Clock Control Register 0
	union	PCLKCR1_REG		PCLKCR1;	 // Peripheral Clock Control Register 1
	union	LPMCR0_REG		LPMCR0;		 // Low Power Mode Control Register 0
	uint16_t					rsvd4;		 // Reserved
	union	PCLKCR3_REG		PCLKCR3;	 // Peripheral Clock Control Register 3
	union	PLLCR_REG		PLLCR;		 // PLL Control Register
	uint16_t					SCSR;		 // System Control and Status Register
	uint16_t					WDCNTR;		 // Watchdog Counter Register
	uint16_t					rsvd5;		 // Reserved
	uint16_t					WDKEY;		 // Watchdog Reset Key Register
	uint16_t					rsvd6[3];	 // Reserved
	uint16_t					WDCR;		 // Watchdog Control Register
	union	JTAGDEBUG_REG	JTAGDEBUG;	 // JTAG Port Debug Register
	uint16_t					rsvd7[5];	 // Reserved
	union	PLL2CTL_REG		PLL2CTL;	 // PLL2 Configuration Register
	uint16_t					rsvd8;		 // Reserved
	union	PLL2MULT_REG	PLL2MULT;	 // PLL2 Multiplier Register
	uint16_t					rsvd9;		 // Reserved
	union	PLL2STS_REG		PLL2STS;	 // PLL2 Lock Status Register
	uint16_t					rsvd10;		 // Reserved
	uint16_t					SYSCLK2CNTR; // SYSCLK2 Clock Counter Register
	uint16_t					rsvd11[3];	 // Reserved
	union	EPWMCFG_REG		EPWMCFG;	 // EPWM DMA/CLA Configuration Register
	uint16_t					rsvd12[5];	 // Reserved
};

struct FLASH_REGS
{
	union	FOPT_REG		FOPT;			// Option Register
	uint16_t					rsvd1;			// Reserved
	union	FPWR_REG		FPWR;			// Power Modes Register
	union	FSTATUS_REG		FSTATUS;		// Status Register
	
    //
    // Pump/Bank Sleep to Standby Wait State Register
    //
    union	FSTDBYWAIT_REG	FSTDBYWAIT;		
    
    //
    // Pump/Bank Standby to Active Wait State Register
    //
	union	FACTIVEWAIT_REG	FACTIVEWAIT;	
    
	//
    // Bank Read Access Wait State Register
    //
    union	FBANKWAIT_REG	FBANKWAIT;		
    
    //
    // OTP Read Access Wait State Register
    //
	union	FOTPWAIT_REG	FOTPWAIT;		
};

//
// Sysctrl External References & Function Declarations:
//
static volatile struct SYS_CTRL_REGS *SysCtrlRegs = (struct SYS_CTRL_REGS*)0x7010;

extern volatile struct SYS_PWR_CTRL_REGS SysPwrCtrlRegs;
extern volatile struct CSM_PWL CsmPwl;
extern volatile struct CSM_REGS CsmRegs;
extern volatile struct FLASH_REGS FlashRegs;

#ifdef __cplusplus
}
#endif /* extern "C" */


#endif  // end of F2806x_SYSCTRL_H definition

//
// End of file
//

