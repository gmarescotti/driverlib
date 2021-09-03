# Porting of C2000 _**Driverlib**_ to F2806x and F2803x

Since Piccolo F280049, Texas Instruments offers an API software to configure and access hardware without the need to know the intrinsic details of any register of the DSP. This simplifies a lot writing the code and make the software easy to understand and to maintain.

Unfortunately these API are not available for **TMS320F28069** and **TMS320F28035** DSP microcontrollers, there is just a called "peripheral driver library" in the ControlSuite with just few peripheral and a different API prototypes (google for "F2806x Peripheral Driver Library").

Therefore, the current project implements a back porting of the driverlib API, at present some modules are "ported" and working, others are to be completed and tested, others are still to be "translated", so there is a lot of works to do. **_Any help or collaboration is very much welcomed_**.

The work so far consisted of different types of "translation" of any of the modules witten for a more powerful DSP (F280049) provided with "new" peripherals or "improved" peripherals. The modification types can be grouped as:

- removed of the module because that hardware is not present in the DSP
- nothing to do as the peripheral is exactly the same as in the previous piccolo DSP
- remapped the address of the register as they was moved, but let the code the same
- completely rewritten the code but let unmodified the API interface

In the "Work Progress" can be seen the status of the work for every modules.

## Work Progress

- Tests have been done with the TMDSDOCK28069 F28069 Piccolo Experimenter Kit.
- The F28035 has still not been tested.
- sample projects from F280049 will be soon adapted to F28069 and linked here

#### Legend

- ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO`
modules not yet ported
- ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE`
modules ported and tested
- ![PARTIAL](https://via.placeholder.com/15/F9F000/000000?text=+) `(partially) DONE`
modules ported and partially tested
- ![SKIP](https://via.placeholder.com/15/DEDEDE/000000?text=+) `SKIP`
modules skipped as not present in the F2806x or F2803x DSP

#### Module list

| Module Name | Work | State |
| ------ | ------ | -- |
|CAN | reimplemented using bitfield registers | ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE` |
|GPIO | - | ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE` |
|SPI | - | ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE` |
|EPWM | - | ![PARTIAL](https://via.placeholder.com/15/F9F000/000000?text=+) `(partially) DONE` |
|SYSCTL | - | ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE` |
|SCI | - | ![DONE](https://via.placeholder.com/15/c5f015/000000?text=+) `DONE` |
|ADC | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|ASysCtl | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|CLA | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|CPU Timer | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|DAC | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|DCC | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|DCSM | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|DMA | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|ECAP | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|HRCAP | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|HRPWM | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|EQEP | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|FLASH | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|FSI | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|I2C | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|Interrupt | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|LIN | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|MEMCFG | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|PGA | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|PMBUS | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|SDFM | - | ![TODO](https://via.placeholder.com/15/f03c15/000000?text=+) `TODO` |
|CLAPROMCRC | - | ![SKIP](https://via.placeholder.com/15/DEDEDE/000000?text=+) `SKIP` |
|CLB | - | ![SKIP](https://via.placeholder.com/15/DEDEDE/000000?text=+) `SKIP` |
|CMPSS | - | ![SKIP](https://via.placeholder.com/15/DEDEDE/000000?text=+) `SKIP` |
|XBAR | - | ![SKIP](https://via.placeholder.com/15/DEDEDE/000000?text=+) `SKIP` |

## Why using driverlib instead of bitfield registers

Texas explain the transition from bitfield to driverlib here: [C2000ware/drivers.html:](https://software-dl.ti.com/C2000/docs/software_guide/c2000ware/drivers.html)
> ...
> Starting with C2000Ware, the recommended approach
> to access peripherals is using driverlibs. However, to assist in a smooth transition from bitfield to driverlib, bitfield
> software continues to be packaged as part of C2000Ware
> ...

## Links and references

- [F28004x: A Comparison to the F2806x and F2803x](https://www.ti.com/lit/sprt731)
- [C2000ware/drivers.html:](https://software-dl.ti.com/C2000/docs/software_guide/c2000ware/drivers.html)

## LICENSE

```
Copyright (c) 2013, Texas Instruments
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Texas Instruments nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```
