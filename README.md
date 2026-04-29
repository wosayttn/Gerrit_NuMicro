# M2003 Series CMSIS BSP

This BSP folder


## .\Document\

- CMSIS.html<br>
	Document of CMSIS version 6.1.0.

- NuMicro M2003 Series CMSIS BSP Driver Reference Guide.chm<br>
	This document describes the usage of drivers in M2003 BSP.

- NuMicro M2003 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of M2003 BSP.


## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V6.1.0 definitions by Arm® Corp.

- Device<br>
	CMSIS compliant device header file.

- StdDriver<br>
	All peripheral driver header and source files.


## .\Sample Code\

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler shows some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- PowerManagement<br>
	Power management sample code.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Sample code to demonstrate the usage of M2003 series MCU peripheral driver APIs.

- Template<br>
	A project template for M2003 series MCU.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
M2003 BSP files are provided under the Apache-2.0 license.
