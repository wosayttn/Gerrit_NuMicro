
#ifndef __UART_EMU_H__
#define __UART_EMU_H__

#ifdef DEBUG_PORT
#ifdef DBG_UUART0
#undef DEBUG_PORT
#define DEBUG_PORT      UUART0       /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif
//#else
//#define DEBUG_PORT      UUART0       /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif

#define RXBUFSIZE             256


#define USE_NMM_MODE   1   //0:AAD, 1:NMM
#define ADDRESS		  	  0xC0		//0xC0, 0x40

#define Test_Baud_Rate      150
#define Test_Baud_Rate_End  100

#define GPIO_MODE(pin, mode)    ((mode) << ((pin)<<1)) /*!< Generate the PMD mode setting for each pin  */
#define _GPIO_SET_PIN_MODE(port, pin, mode)     ((port)->MODE = ((port)->MODE & ~GPIO_MODE(pin, GPIO_MODE_QUASI)) | GPIO_MODE(pin, mode))

typedef void (PFN_DRVUART_CALLBACK)(uint32_t userData);   
	 
/*=== BaudRate comvent divider table === */
#define UART_BASE    UART0_BASE
//#define TEST_PORT    UART_PORT0

#define TEST_PORT0   UART_PORT0 
#define TEST_PORT1   UART_PORT1 
#define TEST_PORT2   UART_PORT2 

//CASE3
#define BAUD_2400_3   (0x1A09|0x30000000)
#define BAUD_4800_3   (0xD03|0x30000000)
#define BAUD_9600_3   (0x681|0x30000000)
#define BAUD_14400_3  (0x455|0x30000000)
#define BAUD_19200_3  (0x33F|0x30000000)
#define BAUD_28800_3  (0x22A|0x30000000)
#define BAUD_38400_3  (0x19F|0x30000000)
#define BAUD_57600_3  (0x114|0x30000000)
#define BAUD_115200_3 (0x89|0x30000000)
#define BAUD_230400_3 (0x43|0x30000000)
#define BAUD_460800_3  (0x21|0x30000000) 
#define BAUD_921600_3  (0xF|0x30000000) 
#define BAUD_1228800_3 (0xB|0x30000000) 

//CASE2
#define BAUD_2400_2   (0x29000298)//(0x290001F2)
#define BAUD_4800_2   (0x2900014B)//(0x290000F7)
#define BAUD_9600_2   (0x290000A4)
#define BAUD_14400_2  (0x2900006D)
#define BAUD_19200_2  (0x29000051)
//#define BAUD_28800_2  (0x2900002A)
#define BAUD_28800_2  (0x29000035)		// 20180103 Test OK.
#define BAUD_38400_2  (0x29000027)
//#define BAUD_57600_2  (0x29000016)
#define BAUD_57600_2  (0x29000019)		// 20180103 Test OK.
#define BAUD_115200_2 (0x2900000C)
#define BAUD_230400_2 (0x29000005)
//#define BAUD_460800_2 (0x29000001)
#define BAUD_460800_2 (0x29000001)		// 20180103 Test OK.

//CASE1
// for 16 Mhz input clock
#define BAUD_2400_1  (415)//(310)
#define BAUD_4800_1  (206)//(154)
#define BAUD_9600_1  (102)
#define BAUD_14400_1 (67)
#define BAUD_19200_1 (50)
#define BAUD_28800_1 (33)
#define BAUD_38400_1 (24)
#define BAUD_57600_1 (15)
#define BAUD_115200_1 (7)
#define BAUD_230400_1 (2)
#define BAUD_460800_1 (0)  

/*---------------------------------------------------------------------------------------------------------*/
/*  Define UART Channel Sturcture                                                                          */
/*---------------------------------------------------------------------------------------------------------*/

typedef enum 
{
	UART_PORT0 = 0x0, 
	UART_PORT1 = 0x1000,
	UART_PORT2 = 0x2000,
} E_UART_PORT;	

typedef enum 
{
	UART_CLOCK_HXT = 0x0, 
	UART_CLOCK_PLL,
	UART_CLOCK_LXT,
	UART_CLOCK_HIRC,
	UART_CLOCK_MIRC,
	UART_CLOCK_LIRC,	
} E_UART_CLOCK_SRC;

/*---------------------------------------------------------------------------------------------------------*/
/*  Define UART Multi-function Pin Setting Set                                                             */
/*---------------------------------------------------------------------------------------------------------*/

typedef enum
{
  E_UART0_A1_A0,/*tx,rx*/
	E_UART0_A5_A4_A1_A0,/*cts,rts,tx,rx*/
	E_UART0_A5_A4,/*tx,rx*/
	E_UART0_A7_A6,/*tx,rx*/
	E_UART0_A5_A4_A7_A6,/*cts,rts,tx,rx*/
	E_UART0_A14_A15/*tx,rx*/,
	E_UART0_A5_A4_A14_A15,/*cts,rts,tx,rx*/
  E_UART0_B9_B8,/*tx,rx*/
	E_UART0_B11_B10_B9_B8,/**cts,rts,tx,rx*/
	E_UART0_B13_B12,/*tx,rx*/
	E_UART0_B15_B14_B13_B12,/**cts,rts,tx,rx*/
	E_UART0_D3_D2,/*tx,rx*/
	E_UART0_C7_C6_D3_D2,/*cts,rts,tx,rx*/
	E_UART0_F5_F4_F3_F2,/*cts,rts,tx,rx*/
	E_UART0_C7_C6_F3_F2,/*cts,rts,tx,rx*/
	E_UART0_F0_F1,/*tx,rx*/	
	/* new for Uart-0 */
	E_UART0_C12_C11,/*tx,rx*/	
	E_UART0_H10_H11,/*tx,rx*/		
	
	E_UART1_A3_A2,/*tx,rx*/
  E_UART1_A1_A0_A3_A2,/*cts,rts,tx,rx*/
	E_UART1_A9_A8,/*tx,rx*/
  E_UART1_A1_A0_A9_A8,/*cts,rts,tx,rx*/
  E_UART1_B3_B2,/*tx,rx*/
	E_UART1_B9_B8_B3_B2,/*cts,rts,tx,rx*/
	E_UART1_B7_B6,/*tx,rx*/
	E_UART1_B9_B8_B7_B6,/*cts,rts,tx,rx*/
	E_UART1_F0_F1,/*tx,rx*/
	/* new for uart-1 */
	E_UART1_E11_E12_D11_D10,/*cts,rts,tx,rx*/
	E_UART1_E13_C8,/*tx,rx*/
	E_UART1_D7_D6,/*tx,rx*/	
	E_UART1_H8_H9,/*tx,rx*/	
	
	E_UART2_B1_B0,/*tx,rx*/
	E_UART2_C2_C3_B1_B0,/*cts,rts,tx,rx*/
	E_UART2_C1_C0,/*tx,rx*/
	E_UART2_C2_C3_C1_C0,/*cts,rts,tx,rx*/	
	E_UART2_C5_C4,/*tx,rx*/ 
  E_UART2_C2_C3_C5_C4,/*cts,rts,tx,rx*/
	E_UART2_B5_B4,/*tx,rx*/ 
	E_UART2_F5_F4_B5_B4,/*cts,rts,tx,rx*/
  E_UART2_F4_F5,/*tx,rx*/ 
	
	/* new for uart-2 */
	E_UART2_D9_D8_E8_D12,/*cts,rts,tx,rx*/
  E_UART2_E8_E9,/*tx,rx*/ 
  E_UART2_E14_E15,/*tx,rx*/	
  E_UART2_F0_F1,/*tx,rx*/ //new for M2U51	
		
} E_UART_SET;


typedef enum 
{
	MODE_NONE = 0x0, 
	MODE_TX=1,
	MODE_RX=2,
	MODE_TXRX=3
} E_OPERATION_MODE;	

typedef enum 
{
	SRC_TMR0 = 0x0, 
	SRC_TMR1,
	SRC_TMR2,
	SRC_TMR3,
	SRC_WKIOA,
	SRC_WKIOB,    
	SRC_WKIOC,    
	SRC_WKIOD,    
	SRC_TPWM0,
	SRC_TPWM1, 
	SRC_TPWM2, 
	SRC_TPWM3, 	
} E_TRIGGER_SOURCE;

/*---------------------------------------------------------------------------------------------------------*/
/*  Define PDMA Channel for UART                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

#define PDMA0_UART0_TX   (4 << PDMA_REQSEL0_3_REQSRC0_Pos)
#define PDMA0_UART0_RX   (5 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART1_TX   (6 << PDMA_REQSEL0_3_REQSRC0_Pos)   
#define PDMA0_UART1_RX   (7 << PDMA_REQSEL0_3_REQSRC0_Pos)  
#define PDMA0_UART2_TX   (8 << PDMA_REQSEL0_3_REQSRC0_Pos)  
#define PDMA0_UART2_RX   (9 << PDMA_REQSEL0_3_REQSRC0_Pos)
#define PDMA0_UART3_TX   (10 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART3_RX   (11 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART4_TX   (12 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART4_RX   (13 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART5_TX   (14 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART5_RX   (15 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART6_TX   (66 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART6_RX   (67 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART7_TX   (68 << PDMA_REQSEL0_3_REQSRC0_Pos) 
#define PDMA0_UART7_RX   (69 << PDMA_REQSEL0_3_REQSRC0_Pos) 

#define PDMA1_UART0_TX   (4 << PDMA_REQSEL0_3_REQSRC1_Pos)
#define PDMA1_UART0_RX   (5 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART1_TX   (6 << PDMA_REQSEL0_3_REQSRC1_Pos)   
#define PDMA1_UART1_RX   (7 << PDMA_REQSEL0_3_REQSRC1_Pos)  
#define PDMA1_UART2_TX   (8 << PDMA_REQSEL0_3_REQSRC1_Pos)  
#define PDMA1_UART2_RX   (9 << PDMA_REQSEL0_3_REQSRC1_Pos)
#define PDMA1_UART3_TX   (10 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART3_RX   (11 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART4_TX   (12 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART4_RX   (13 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART5_TX   (14 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART5_RX   (15 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART6_TX   (66 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART6_RX   (67 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART7_TX   (68 << PDMA_REQSEL0_3_REQSRC1_Pos) 
#define PDMA1_UART7_RX   (69 << PDMA_REQSEL0_3_REQSRC1_Pos) 

#define PDMA2_UART0_TX   (4 << PDMA_REQSEL0_3_REQSRC2_Pos)
#define PDMA2_UART0_RX   (5 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART1_TX   (6 << PDMA_REQSEL0_3_REQSRC2_Pos)   
#define PDMA2_UART1_RX   (7 << PDMA_REQSEL0_3_REQSRC2_Pos)  
#define PDMA2_UART2_TX   (8 << PDMA_REQSEL0_3_REQSRC2_Pos)  
#define PDMA2_UART2_RX   (9 << PDMA_REQSEL0_3_REQSRC2_Pos)
#define PDMA2_UART3_TX   (10 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART3_RX   (11 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART4_TX   (12 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART4_RX   (13 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART5_TX   (14 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART5_RX   (15 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART6_TX   (66 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART6_RX   (67 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART7_TX   (68 << PDMA_REQSEL0_3_REQSRC2_Pos) 
#define PDMA2_UART7_RX   (69 << PDMA_REQSEL0_3_REQSRC2_Pos)

#define PDMA3_UART0_TX   (4 << PDMA_REQSEL0_3_REQSRC3_Pos)
#define PDMA3_UART0_RX   (5 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART1_TX   (6 << PDMA_REQSEL0_3_REQSRC3_Pos)   
#define PDMA3_UART1_RX   (7 << PDMA_REQSEL0_3_REQSRC3_Pos)  
#define PDMA3_UART2_TX   (8 << PDMA_REQSEL0_3_REQSRC3_Pos)  
#define PDMA3_UART2_RX   (9 << PDMA_REQSEL0_3_REQSRC3_Pos)
#define PDMA3_UART3_TX   (10 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART3_RX   (11 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART4_TX   (12 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART4_RX   (13 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART5_TX   (14 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART5_RX   (15 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART6_TX   (66 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART6_RX   (67 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART7_TX   (68 << PDMA_REQSEL0_3_REQSRC3_Pos) 
#define PDMA3_UART7_RX   (69 << PDMA_REQSEL0_3_REQSRC3_Pos) 

#define PDMA4_UART0_TX   (4 << PDMA_REQSEL4_7_REQSRC4_Pos)
#define PDMA4_UART0_RX   (5 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART1_TX   (6 << PDMA_REQSEL4_7_REQSRC4_Pos)   
#define PDMA4_UART1_RX   (7 << PDMA_REQSEL4_7_REQSRC4_Pos)  
#define PDMA4_UART2_TX   (8 << PDMA_REQSEL4_7_REQSRC4_Pos)  
#define PDMA4_UART2_RX   (9 << PDMA_REQSEL4_7_REQSRC4_Pos)
#define PDMA4_UART3_TX   (10 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART3_RX   (11 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART4_TX   (12 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART4_RX   (13 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART5_TX   (14 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART5_RX   (15 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART6_TX   (66 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART6_RX   (67 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART7_TX   (68 << PDMA_REQSEL4_7_REQSRC4_Pos) 
#define PDMA4_UART7_RX   (69 << PDMA_REQSEL4_7_REQSRC4_Pos) 

#define PDMA5_UART0_TX   (4 << PDMA_REQSEL4_7_REQSRC5_Pos)
#define PDMA5_UART0_RX   (5 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART1_TX   (6 << PDMA_REQSEL4_7_REQSRC5_Pos)   
#define PDMA5_UART1_RX   (7 << PDMA_REQSEL4_7_REQSRC5_Pos)  
#define PDMA5_UART2_TX   (8 << PDMA_REQSEL4_7_REQSRC5_Pos)  
#define PDMA5_UART2_RX   (9 << PDMA_REQSEL4_7_REQSRC5_Pos)
#define PDMA5_UART3_TX   (10 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART3_RX   (11 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART4_TX   (12 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART4_RX   (13 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART5_TX   (14 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART5_RX   (15 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART6_TX   (66 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART6_RX   (67 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART7_TX   (68 << PDMA_REQSEL4_7_REQSRC5_Pos) 
#define PDMA5_UART7_RX   (69 << PDMA_REQSEL4_7_REQSRC5_Pos) 

#define PDMA6_UART0_TX   (4 << PDMA_REQSEL4_7_REQSRC6_Pos)
#define PDMA6_UART0_RX   (5 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART1_TX   (6 << PDMA_REQSEL4_7_REQSRC6_Pos)   
#define PDMA6_UART1_RX   (7 << PDMA_REQSEL4_7_REQSRC6_Pos)  
#define PDMA6_UART2_TX   (8 << PDMA_REQSEL4_7_REQSRC6_Pos)  
#define PDMA6_UART2_RX   (9 << PDMA_REQSEL4_7_REQSRC6_Pos)
#define PDMA6_UART3_TX   (10 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART3_RX   (11 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART4_TX   (12 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART4_RX   (13 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART5_TX   (14 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART5_RX   (15 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART6_TX   (66 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART6_RX   (67 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART7_TX   (68 << PDMA_REQSEL4_7_REQSRC6_Pos) 
#define PDMA6_UART7_RX   (69 << PDMA_REQSEL4_7_REQSRC6_Pos) 

#define PDMA7_UART0_TX   (4 << PDMA_REQSEL4_7_REQSRC7_Pos)
#define PDMA7_UART0_RX   (5 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART1_TX   (6 << PDMA_REQSEL4_7_REQSRC7_Pos)   
#define PDMA7_UART1_RX   (7 << PDMA_REQSEL4_7_REQSRC7_Pos)  
#define PDMA7_UART2_TX   (8 << PDMA_REQSEL4_7_REQSRC7_Pos)  
#define PDMA7_UART2_RX   (9 << PDMA_REQSEL4_7_REQSRC7_Pos)
#define PDMA7_UART3_TX   (10 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART3_RX   (11 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART4_TX   (12 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART4_RX   (13 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART5_TX   (14 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART5_RX   (15 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART6_TX   (66 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART6_RX   (67 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART7_TX   (68 << PDMA_REQSEL4_7_REQSRC7_Pos) 
#define PDMA7_UART7_RX   (69 << PDMA_REQSEL4_7_REQSRC7_Pos) 
/*
#define PDMA8_UART0_TX   (4 << PDMA_REQSEL8_11_REQSRC8_Pos)
#define PDMA8_UART0_RX   (5 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART1_TX   (6 << PDMA_REQSEL8_11_REQSRC8_Pos)   
#define PDMA8_UART1_RX   (7 << PDMA_REQSEL8_11_REQSRC8_Pos)  
#define PDMA8_UART2_TX   (8 << PDMA_REQSEL8_11_REQSRC8_Pos)  
#define PDMA8_UART2_RX   (9 << PDMA_REQSEL8_11_REQSRC8_Pos)
#define PDMA8_UART3_TX   (10 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART3_RX   (11 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART4_TX   (12 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART4_RX   (13 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART5_TX   (14 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART5_RX   (15 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART6_TX   (66 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART6_RX   (67 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART7_TX   (68 << PDMA_REQSEL8_11_REQSRC8_Pos) 
#define PDMA8_UART7_RX   (69 << PDMA_REQSEL8_11_REQSRC8_Pos)

#define PDMA9_UART0_TX   (4 << PDMA_REQSEL8_11_REQSRC9_Pos)
#define PDMA9_UART0_RX   (5 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART1_TX   (6 << PDMA_REQSEL8_11_REQSRC9_Pos)   
#define PDMA9_UART1_RX   (7 << PDMA_REQSEL8_11_REQSRC9_Pos)  
#define PDMA9_UART2_TX   (8 << PDMA_REQSEL8_11_REQSRC9_Pos)  
#define PDMA9_UART2_RX   (9 << PDMA_REQSEL8_11_REQSRC9_Pos)
#define PDMA9_UART3_TX   (10 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART3_RX   (11 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART4_TX   (12 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART4_RX   (13 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART5_TX   (14 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART5_RX   (15 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART6_TX   (66 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART6_RX   (67 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART7_TX   (68 << PDMA_REQSEL8_11_REQSRC9_Pos) 
#define PDMA9_UART7_RX   (69 << PDMA_REQSEL8_11_REQSRC9_Pos)  

#define PDMA10_UART0_TX   (4 << PDMA_REQSEL8_11_REQSRC10_Pos)
#define PDMA10_UART0_RX   (5 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART1_TX   (6 << PDMA_REQSEL8_11_REQSRC10_Pos)   
#define PDMA10_UART1_RX   (7 << PDMA_REQSEL8_11_REQSRC10_Pos)  
#define PDMA10_UART2_TX   (8 << PDMA_REQSEL8_11_REQSRC10_Pos)  
#define PDMA10_UART2_RX   (9 << PDMA_REQSEL8_11_REQSRC10_Pos)
#define PDMA10_UART3_TX   (10 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART3_RX   (11 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART4_TX   (12 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART4_RX   (13 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART5_TX   (14 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART5_RX   (15 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART6_TX   (66 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART6_RX   (67 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART7_TX   (68 << PDMA_REQSEL8_11_REQSRC10_Pos) 
#define PDMA10_UART7_RX   (69 << PDMA_REQSEL8_11_REQSRC10_Pos)  

#define PDMA11_UART0_TX   (4 << PDMA_REQSEL8_11_REQSRC11_Pos)
#define PDMA11_UART0_RX   (5 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART1_TX   (6 << PDMA_REQSEL8_11_REQSRC11_Pos)   
#define PDMA11_UART1_RX   (7 << PDMA_REQSEL8_11_REQSRC11_Pos)  
#define PDMA11_UART2_TX   (8 << PDMA_REQSEL8_11_REQSRC11_Pos)  
#define PDMA11_UART2_RX   (9 << PDMA_REQSEL8_11_REQSRC11_Pos)
#define PDMA11_UART3_TX   (10 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART3_RX   (11 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART4_TX   (12 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART4_RX   (13 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART5_TX   (14 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART5_RX   (15 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART6_TX   (66 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART6_RX   (67 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART7_TX   (68 << PDMA_REQSEL8_11_REQSRC11_Pos) 
#define PDMA11_UART7_RX   (69 << PDMA_REQSEL8_11_REQSRC11_Pos)  

#define PDMA12_UART0_TX   (4 << PDMA_REQSEL12_15_REQSRC12_Pos)
#define PDMA12_UART0_RX   (5 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART1_TX   (6 << PDMA_REQSEL12_15_REQSRC12_Pos)   
#define PDMA12_UART1_RX   (7 << PDMA_REQSEL12_15_REQSRC12_Pos)  
#define PDMA12_UART2_TX   (8 << PDMA_REQSEL12_15_REQSRC12_Pos)  
#define PDMA12_UART2_RX   (9 << PDMA_REQSEL12_15_REQSRC12_Pos)
#define PDMA12_UART3_TX   (10 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART3_RX   (11 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART4_TX   (12 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART4_RX   (13 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART5_TX   (14 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART5_RX   (15 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART6_TX   (66 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART6_RX   (67 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART7_TX   (68 << PDMA_REQSEL12_15_REQSRC12_Pos) 
#define PDMA12_UART7_RX   (69 << PDMA_REQSEL12_15_REQSRC12_Pos)  

#define PDMA13_UART0_TX   (4 << PDMA_REQSEL12_15_REQSRC13_Pos)
#define PDMA13_UART0_RX   (5 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART1_TX   (6 << PDMA_REQSEL12_15_REQSRC13_Pos)   
#define PDMA13_UART1_RX   (7 << PDMA_REQSEL12_15_REQSRC13_Pos)  
#define PDMA13_UART2_TX   (8 << PDMA_REQSEL12_15_REQSRC13_Pos)  
#define PDMA13_UART2_RX   (9 << PDMA_REQSEL12_15_REQSRC13_Pos)
#define PDMA13_UART3_TX   (10 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART3_RX   (11 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART4_TX   (12 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART4_RX   (13 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART5_TX   (14 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART5_RX   (15 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART6_TX   (66 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART6_RX   (67 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART7_TX   (68 << PDMA_REQSEL12_15_REQSRC13_Pos) 
#define PDMA13_UART7_RX   (69 << PDMA_REQSEL12_15_REQSRC13_Pos)  

#define PDMA14_UART0_TX   (4 << PDMA_REQSEL12_15_REQSRC14_Pos)
#define PDMA14_UART0_RX   (5 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART1_TX   (6 << PDMA_REQSEL12_15_REQSRC14_Pos)   
#define PDMA14_UART1_RX   (7 << PDMA_REQSEL12_15_REQSRC14_Pos)  
#define PDMA14_UART2_TX   (8 << PDMA_REQSEL12_15_REQSRC14_Pos)  
#define PDMA14_UART2_RX   (9 << PDMA_REQSEL12_15_REQSRC14_Pos)
#define PDMA14_UART3_TX   (10 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART3_RX   (11 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART4_TX   (12 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART4_RX   (13 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART5_TX   (14 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART5_RX   (15 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART6_TX   (66 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART6_RX   (67 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART7_TX   (68 << PDMA_REQSEL12_15_REQSRC14_Pos) 
#define PDMA14_UART7_RX   (69 << PDMA_REQSEL12_15_REQSRC14_Pos)  

#define PDMA15_UART0_TX   (4 << PDMA_REQSEL12_15_REQSRC15_Pos)
#define PDMA15_UART0_RX   (5 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART1_TX   (6 << PDMA_REQSEL12_15_REQSRC15_Pos)   
#define PDMA15_UART1_RX   (7 << PDMA_REQSEL12_15_REQSRC15_Pos)  
#define PDMA15_UART2_TX   (8 << PDMA_REQSEL12_15_REQSRC15_Pos)  
#define PDMA15_UART2_RX   (9 << PDMA_REQSEL12_15_REQSRC15_Pos)
#define PDMA15_UART3_TX   (10 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART3_RX   (11 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART4_TX   (12 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART4_RX   (13 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART5_TX   (14 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART5_RX   (15 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART6_TX   (66 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART6_RX   (67 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART7_TX   (68 << PDMA_REQSEL12_15_REQSRC15_Pos) 
#define PDMA15_UART7_RX   (69 << PDMA_REQSEL12_15_REQSRC15_Pos)  
*/

#define PDMA_REQSEL0_3_Msk  0x7F7F7F7F
#define PDMA_REQSEL4_7_Msk  0x7F7F7F7F
//#define PDMA_REQSEL8_11_Msk  0x7F7F7F7F
//#define PDMA_REQSEL12_15_Msk  0x7F7F7F7F


#define PDMA0_CHANEL   0
#define PDMA1_CHANEL   1
#define PDMA2_CHANEL   2
#define PDMA3_CHANEL   3
#define PDMA4_CHANEL   4
#define PDMA5_CHANEL   5
#define PDMA6_CHANEL   6
#define PDMA7_CHANEL   7
/*
#define PDMA8_CHANEL   8
#define PDMA9_CHANEL   9
#define PDMA10_CHANEL   10
#define PDMA11_CHANEL   11
#define PDMA12_CHANEL   12
#define PDMA13_CHANEL   13
#define PDMA14_CHANEL   14
#define PDMA15_CHANEL   15
*/

/**
 * @brief      Set UART line control 
 *
 * @param[in]  UART         Structure pointer of UART Channel selected,should be:  
 *                          - UART0 : UART Channel 0
 *                          - UART1 : UART Channel 1 
 * @param[in]  u32PortSettings  Line control value: Use "|" to combine your settings
 *                          - Word Length Select : eg. UART_WORD_LEN_5 
 *                          - Parity Bit         : eg. UART_PARITY_NONE
 *                          - Stop Bit           : eg. UART_STOPBIT_1
 * @return     None
 *
 * @details    The function is used to set UART data format (Word Length Select /Parity Bit/Stop Bit).\n
 *             Example: _UART_SET_DATA_FORMAT(UART0, UART_WORD_LEN_5 | UART_PARITY_NONE | UART_STOPBIT_1)
 */
#define _UART_SET_DATA_FORMAT(UART, u32PortSettings)  ((UART)->LINE = (u32PortSettings))

/**
 * @brief      Wait specified uart port transmission is over 
 *
 * @param[in]  UART         Structure pointer of UART Channel  
 *                          - UART0 : UART Channel 0
 *                          - UART1 : UART Channel 1 
 *
 * @return     None 
 *
 * @details    The function is used to polling FSR[28] and waiting the TX_FIFO and 
 *             TX_Shift_Reigster is empty. Wait FSR[28] bit is set to exit the while loop.\n
 *             Example: _UART_WAIT_TX_EMPTY(UART0)
 */
#define _UART_WAIT_TX_EMPTY(UART)        while(!((((UART)->FIFOSTS) & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos))
//#define UART_IS_TX_EMPTY(uart)    (((uart)->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos)


/**
 * @brief      Get CTS Pin value
 *
 * @param[in]  UART         Structure pointer of UART Channel
 *                          - UART0 : UART Channel 0
 *                          - UART1 : UART Channel 1
 * @retval     1 = CTS pin value is HIGH
 * @retval     0 = CTS pin value is LOW
 *                                                           
 * @details    The function is used to get CTS pin value.
 *             Example: _UART_GET_CTSPIN(UART0)
 *
 */
#define _UART_GET_CTSPIN(UART)   (((UART)->MODEMSTS & UART_MODEMSTS_CTSSTS_Msk )>>UART_MODEMSTS_CTSSTS_Pos)

#ifndef __USCI_INIT_H__
#define __USCI_INIT_H__
/*---------------------------------------------------------------------------------------------------------*/
/*  Define SYS_SysTickDelay if necessary                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static __INLINE void SYS_SysTickDelay(uint32_t us)
{
    if (us > 100000)
        us = 100000;
    
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Define if USCI port is used for debug port                                                             */
/*---------------------------------------------------------------------------------------------------------*/
static void USCI_Init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_USCI0CKEN_Msk ;	 
 	
    //SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
    //                (SYS_GPB_MFPH_PB13MFP_USCI0_DAT0 | SYS_GPB_MFPH_PB14MFP_USCI0_DAT1);
    outp32(0x4000003C, (inp32(0x4000003C) & 0xF00FFFFF) | 0x05500000);      // Set M460HD MFP for PB14(RXD) & PB.13(TXD)  

		//UUART_Open(UUART0, 115200);
	
#if 1
 /* Set USCI to UART mode */ 
    UUART0->CTL = 2;                  //UART protocol    
    UUART0->PROTCTL |= (1UL<<31);     //UART protocol enable      
    UUART0->LINECTL = (8<<8) | 1;     //8 data bit and LSB      
    UUART0->DATIN0 = (2<<3);      //Setting for UART Rx     
    //UUART0->BRGEN = (1<<16) | (12<<10) | (3<<8);  //115200bps   
    UUART0->BRGEN = (28<<16) | (4<<10) | (0<<8);  //115200bps @HCLK=16MHz
 #endif
}
#endif

#define _UUART_IS_TX_EMPTY(UUART)    (((UUART)->BUFSTS &  UUART_BUFSTS_TXEMPTY_Msk)>> UUART_BUFSTS_TXEMPTY_Pos)

#define UUART_WAIT_TXEMPTYF_RXIDLE(uart) \
    while( ((uart)->BUFSTS & (UUART_BUFSTS_TXEMPTY_Msk)) \
                            !=(UUART_BUFSTS_TXEMPTY_Msk))		

/*---------------------------------------------------------------------------------------------------------*/
/*  DrvUART.c Define function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void DrvUART_EnableInt(E_UART_PORT u32Port, uint32_t u32InterruptFlag, PFN_DRVUART_CALLBACK pfncallback);
void DrvUART_DisableInt(E_UART_PORT u32Port,uint32_t u32InterruptFlag);



#endif //__UART_EMU_H__

