![VSCode](https://github.com/OpenNuvoton/M3351BSP/actions/workflows/VSCode.yml/badge.svg)

# M3351 Series CMSIS BSP

To experience the powerful features of M3351 series in few minutes, please select the sample code to download and execute on the NuMaker-M3351 board. Open the project files to build them with Keil® MDK, IAR or VS Code, and then download and trace them on the NuMaker board to see how it works.


## .\Document\

- CMSIS.html<br>
	Document of CMSIS.

- NuMicro M3351 Series CMSIS BSP Driver Reference Guide.chm<br>
	This document describes the usage of drivers in M3351 Series BSP.

- NuMicro M3351 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of M3351 Series BSP.

- VS Code Quick Start Guide
	This documents guide to install, configure and use VS Code.


## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V6.1.0 definitions by Arm® Corp.<p>
	M3351 CMSIS-Drivers do not guarantee thread safety and Cache coherence. The source and RTE_Device header files are in the "Driver\Source" sub-folder. Please add source files and copy RTE_Device header files into your project. Projects can define PRJ_RTE_DEVICE_HEADER macro to include the private RTE_Device.h.

- Commu<br>
	Helper functions of communication protocols, e.g., XMODEM.

- CryptoAccelerator<br>
	Crypto accelerator source code for mbedtls.

- Device<br>
	CMSIS compliant device header files.

- LlsiYcableLib<br>
	LLSI library source code.
  
- StdDriver<br>
	All peripheral driver header and source files.

- Storage<br>
	Disk I/O modules for FatFs.

- UsbHostLib<br>
	USB host library source code.


## .\Sample Code\

- Crypto<br>
	Crypto sample code using Mbed TLS library.

- FreeRTOS<br>
	Simple FreeRTOS™ demo code.
	
- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler shows some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample code for In-System-Programming.

- NuMaker-M3351KI<br>
	Sample code for NuMaker-M3351KI board.

- PowerManagement<br>
	Sample code for power management.

- SecureApplication<br>
	Sample code for secure application.<p>
	VS Code projects require Python 3.12 at least for post-build.

- StdDriver<br>
	Sample code to demonstrate the usage of M3351 series MCU peripheral driver APIs.

- Template<br>
	Project template for M3351 series MCU.

- TrustZone<br>
	Demo of secure code and non-secure code.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\ThirdParty\

- FatFs<br>
	An open-source FAT/exFAT file system library. A generic FAT file system module for small embedded systems.

- FreeRTOS<br>
	Real-time operating system for microcontrollers.

- mbedtls<br>
	Mbed TLS offers an SSL library with an intuitive API and readable source code, so you can actually understand what the code does.


## .\Tool\

- imgtool.exe<br>
	Used to perform the operations that are necessary to manage keys and sign images.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
M3351 Series BSP files are provided under the Apache-2.0 license.
