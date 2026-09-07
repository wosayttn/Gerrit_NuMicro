/**************************************************************************//**
 * @file     retarget.c
 * @version  V3.00
 * @brief    M471 Series Debug Port and Semihost Setting Source File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/


#include <stdio.h>
#include "NuMicro.h"

#ifndef   __WEAK
  #define __WEAK                                 __attribute__((weak))
#endif
#ifndef   __NO_RETURN
  #define __NO_RETURN                            __attribute__((noreturn))
#endif

#if(defined(__ICCARM__) && (__VER__ >= 9020000))
#include <LowLevelIOInterface.h>
#endif

#if defined (__ICCARM__)
#pragma diag_suppress=Pm150
#endif

#if defined ( __CC_ARM   )
#if (__ARMCC_VERSION < 400000)
#else
/* Insist on keeping widthprec, to avoid X propagation by benign code in C-lib */
#pragma import _printf_widthprec
#endif
#endif

/* Uncomment this line to disable all printf and getchar. getchar() will always return 0x00*/
/* #define DISABLE_UART */

#if defined(DEBUG_ENABLE_SEMIHOST)
    #ifndef DISABLE_UART
        #define DISABLE_UART
    #endif
#endif


#define DEBUG_PORT   UART0

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6040000)) || (defined(__ICCARM__) && (__VER__ >= 8000000))
struct __FILE { int handle; /* Add whatever you need here */ };
#endif

/* [MISRA8.4] Provide compatible declarations for external linkage objects. */
extern FILE __stdout;
extern FILE __stdin;
extern FILE __stderr;

FILE __stdout;
FILE __stdin;

int kbhit(void);
int IsDebugFifoEmpty(void);
void _ttywrch(int ch);
/* cppcheck-suppress misra-c2012-21.2 */
int fputc(int ch, FILE *stream);
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)
/* cppcheck-suppress misra-c2012-21.2 */
int fgetc(FILE *stream);
/* cppcheck-suppress misra-c2012-21.2 */
int ferror(FILE *stream);
#endif

char GetChar(void);
void SendChar_ToUART(int ch);
void SendChar(int ch);

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
# ifdef __MICROLIB

__WEAK __NO_RETURN
void __aeabi_assert(const char* expr, const char* file, int line)
{
    char str[12], * p;

    fputs("*** assertion failed: ", stderr);
    fputs(expr, stderr);
    fputs(", file ", stderr);
    fputs(file, stderr);
    fputs(", line ", stderr);

    p = str + sizeof(str);
    *--p = '\0';
    *--p = '\n';
    while(line > 0)
    {
        *--p = '0' + (line % 10);
        line /= 10;
    }
    fputs(p, stderr);

    for (;;)
    {
        /* stay here */
    }
}


__WEAK
void abort(void)
{
    for (;;)
    {
        /* stay here */
    }
}

# else
__asm("  .global __ARM_use_no_argv\n");
__asm("  .global __use_no_semihosting\n");


FILE __stderr;

void _sys_exit(int return_code)__attribute__((noreturn));
void _sys_exit(int return_code)
{
    (void) return_code;
    for (;;)
    {
        /* stay here */
    }
}

# endif
#endif // defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)

/*---------------------------------------------------------------------------------------------------------*/
/* Routine to write a char                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#if defined(__ICCARM__)
#ifndef DEBUG_ENABLE_SEMIHOST
size_t __write(int handle, const unsigned char *buf, size_t bufSize);
size_t __read(int handle, unsigned char *buf, size_t bufSize);

size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t nChars = 0;

    /* Check for the command to flush all handles */
    if (handle == -1)
    {
        return 0;
    }

    /* Check for stdout and stderr      (only necessary if FILE descriptors are enabled.) */

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    while (nChars < bufSize)
    {
        SendChar(buf[nChars]);
        ++nChars;
    }

    return nChars;
}

size_t __read(int handle, unsigned char* buf, size_t bufSize)
{
    size_t nChars = 0;
    /* Check for stdin      (only necessary if FILE descriptors are enabled) */
    if(handle != 0)
    {
        return -1;
    }

    while (nChars < bufSize)
    {
        unsigned char c;
        c = (unsigned char)GetChar();
        if (c == 0U)
        {
            break;
        }
        buf[nChars] = c;
        ++nChars;
    }
    return nChars;
}
#endif  /* ndef DEBUG_ENABLE_SEMIHOST */
#endif  /* defined(__ICCARM__) */

#if (defined(__ARMCC_VERSION) || defined(__ICCARM__))
extern int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);

/**
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 8.2<br>
 * <b>Justification:</b> Parameters lr, msp, psp are already named in this prototype;
 *                       the preceding conditional __WEAK / __attribute__((weak))
 *                       qualifier confuses the MISRA addon's simplified token stream
 *                       into misreading the parameter list as unnamed. This is a
 *                       tool-parsing artifact of the weak-attribute macro, not a
 *                       missing parameter name.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.6<br>
 * <b>Justification:</b> printf() is used solely for HardFault diagnostic output.
 *                       This retarget module intentionally routes standard I/O to
 *                       the UART/semihosting debug channel; these calls are confined
 *                       to fault reporting and are not used by application control logic.
 */
/* cppcheck-suppress misra-c2012-8.2 */
__WEAK uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp);
#endif

#if defined(DEBUG_ENABLE_SEMIHOST)
/* The static buffer is used to speed up the semihost    */
static char g_buf[16];
static char g_buf_len = 0;
static volatile int32_t g_ICE_Conneced = 1;

void _sys_exit(int return_code)__attribute__((noreturn));

uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp = NULL;
    uint32_t inst = 0UL;

    /* Check the used stack */
    if ((lr & 0x40UL) != 0UL)
    {
        /* Secure stack used */
        if ((lr & 4UL) != 0UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }

    }
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
    else
    {
        /* Non-secure stack used */
        if ((lr & 4UL) != 0UL)
        {
            sp = (uint32_t *)__TZ_get_PSP_NS();
        }
        else
        {
            sp = (uint32_t *)__TZ_get_MSP_NS();
        }

    }
#endif

    /* Get the instruction caused the hardfault */
    if (sp != 0UL)
    {
        inst = M16(sp[6]);

        if (inst == 0xBEABUL)
        {
            /*
                If the instruction is 0xBEAB, it means it is caused by BKPT without ICE connected.
                We still return for output/input message to UART.
            */
            g_ICE_Conneced = 0; // Set a flag for ICE offline
            sp[6] += 2UL; // return to next instruction
            return lr;  // Keep lr in R0
        }

        /* It is casued by hardfault (Not semihost). Just process the hard fault here. */
        /* TODO: Implement your hardfault handle code here */

        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("  HardFault!\n\n");
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("r0  = 0x%x\n", sp[0]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("r1  = 0x%x\n", sp[1]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("r2  = 0x%x\n", sp[2]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("r3  = 0x%x\n", sp[3]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("r12 = 0x%x\n", sp[4]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("lr  = 0x%x\n", sp[5]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("pc  = 0x%x\n", sp[6]);
        /* cppcheck-suppress misra-c2012-21.6 */
        (void)printf("psr = 0x%x\n", sp[7]);
    }

    for (;;)
    {
        /* stay here */
    }

}

/**
 *
 * @brief      The function to process semihosted command
 * @param[in]  n32In_R0  : semihost register 0
 * @param[in]  n32In_R1  : semihost register 1
 * @param[out] pn32Out_R0: semihost register 0
 * @retval     0: No ICE debug
 * @retval     1: ICE debug
 *
 */

int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R1; /* [MISRA2.7] */

    if (g_ICE_Conneced != 0L)
    {
        if (pn32Out_R0 != NULL)
        {
            *pn32Out_R0 = n32In_R0;
        }

        return 1L;
    }
    return 0L;
}

#else   /* ndef (DEBUG_ENABLE_SEMIHOST) */

int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);

/**
 * @brief    This function is called by Hardfault handler.
 *
 * @param    None
 *
 * @returns  None
 *
 * @details  This function is called by Hardfault handler and check if it is caused by __BKPT or not.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.6<br>
 * <b>Justification:</b> printf() is used solely for HardFault diagnostic output.
 *                       This retarget module intentionally routes standard I/O to
 *                       the UART/semihosting debug channel; these calls are confined
 *                       to fault reporting and are not used by application control logic.
 */
__WEAK uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
		const uint32_t *sp;
		uint32_t u32SpValid = 0UL;
        uint32_t inst;
        uint32_t addr;
        uint32_t taddr;
        uint32_t tdata;
        int32_t secure;
        uint32_t rm;
        uint32_t rn;
        uint32_t rt;
        uint32_t imm5;
        uint32_t imm8;

		/* It is casued by hardfault. Just process the hard fault */
		/* TODO: Implement your hardfault handle code here */


		/* Check the used stack */
		secure = ((lr & 0x40UL) != 0UL) ? 1L : 0L;
		if (secure != 0L)
		{
				/* Secure stack used */
				if ((lr & 4UL) != 0UL)
				{
						sp = (const uint32_t *)psp;
					u32SpValid = 1UL;
				}
				else
				{
						sp = (const uint32_t *)msp;
					u32SpValid = 1UL;
				}

		}
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3)
		else
		{
				/* Non-secure stack used */
				if ((lr & 4UL) != 0UL)
                {
                        sp = (const uint32_t *)(__TZ_get_PSP_NS());
                        u32SpValid = 1UL;
                }
                else
                {
                        sp = (const uint32_t *)(__TZ_get_MSP_NS());
                        u32SpValid = 1UL;
                }

		}
#endif

		/*
				r0  = sp[0]
				r1  = sp[1]
				r2  = sp[2]
				r3  = sp[3]
				r12 = sp[4]
				lr  = sp[5]
				pc  = sp[6]
				psr = sp[7]
		*/

        /* Get the instruction caused the hardfault */
        if (u32SpValid != 0UL)
        {
            /* cppcheck-suppress misra-c2012-21.6 */
            (void)printf("HardFault @ 0x%08x\n", sp[6]);
            addr = sp[6];
            inst = M16(addr);

		/* cppcheck-suppress misra-c2012-21.6 */
		(void)printf("HardFault Analysis:\n");

		/* cppcheck-suppress misra-c2012-21.6 */
		(void)printf("Instruction code = %x\n", inst);

		if (inst == 0xBEABUL)
		{
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("Execute BKPT without ICE connected\n");
		}
		else if ((inst >> 12U) == 5UL)
		{
				/* 0101xx Load/store (register offset) on page C2-327 of armv8m ref */
				rm = (inst >> 6U) & 0x7UL;
				rn = (inst >> 3U) & 0x7UL;
				rt = inst & 0x7UL;

				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("LDR/STR rt=%x rm=%x rn=%x\n", rt, rm, rn);
				taddr = sp[rn] + sp[rm];
				tdata = sp[rt];
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
							 ((inst & BIT11) != 0UL) ? "LDR" : "STR", tdata, taddr);

		}
		else if ((inst >> 13U) == 3UL)
		{
				/* 011xxx    Load/store word/byte (immediate offset) on page C2-327 of armv8m ref */
				imm5 = (inst >> 6U) & 0x1FUL;
				rn = (inst >> 3U) & 0x7UL;
				rt = inst & 0x7UL;

				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("LDR/STR rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
				taddr = sp[rn] + imm5;
				tdata = sp[rt];
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
							 ((inst & BIT11) != 0UL) ? "LDR" : "STR", tdata, taddr);
		}
		else if ((inst >> 12U) == 8UL)
		{
				/* 1000xx    Load/store halfword (immediate offset) on page C2-328 */
				imm5 = (inst >> 6U) & 0x1FUL;
				rn = (inst >> 3U) & 0x7UL;
				rt = inst & 0x7UL;

				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("LDRH/STRH rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
				taddr = sp[rn] + imm5;
				tdata = sp[rt];
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
							 ((inst & BIT11) != 0UL) ? "LDR" : "STR", tdata, taddr);

		}
		else if ((inst >> 12U) == 9UL)
		{
				/* 1001xx    Load/store (SP-relative) on page C2-328 */
				imm8 = inst & 0xFFUL;
				rt = (inst >> 8U) & 0x7UL;

				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("LDRH/STRH rt=%x imm8=%x\n", rt, imm8);
				taddr = sp[6] + imm8;
				tdata = sp[rt];
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
							 ((inst & BIT11) != 0UL) ? "LDR" : "STR", tdata, taddr);
		}
		else
		{
				/* cppcheck-suppress misra-c2012-21.6 */
				(void)printf("Unexpected instruction\n");
		}

        }

        for (;;)
        {
            /* stay here */
        }
}

/* [MISRA2.7] Parameters are intentionally unused in semihost return stub. */
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R0;
    (void)n32In_R1;
    (void)pn32Out_R0;
    return 0L;
}

#endif  /* defined(DEBUG_ENABLE_SEMIHOST) */


#ifndef DISABLE_UART
/**
 * @brief       Routine to send a char
 *
 * @param[in]   ch Character to send to debug port.
 *
 * @returns     Send value from UART debug port
 *
 * @details     Send a target char to UART debug port .
 */
#ifndef NONBLOCK_PRINTF
void SendChar_ToUART(int ch)
{
    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
    {
        /* wait */
    }

    if (ch == (int)'\n')
    {
        DEBUG_PORT->DAT = (uint32_t)'\r';
        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
    {
        /* wait */
    }
    }
    DEBUG_PORT->DAT = (uint32_t)ch;
}
#else
/* Non-block implement of send char */
#define BUF_SIZE    2048
static void SendChar_ToUART(int ch)
{
    static uint8_t u8Buf[BUF_SIZE] = {0};
    static int32_t i32Head = 0;
    static int32_t i32Tail = 0;
    int32_t i32Tmp;

    /* Only flush the data in buffer to UART when ch == 0 */
    if(ch)
    {
        /* Push char */
        if(ch == '\n')
        {
            i32Tmp = i32Head+1;
            if (i32Tmp > BUF_SIZE)
            {
                i32Tmp = 0;
            }
            if(i32Tmp != i32Tail)
            {
                u8Buf[i32Head] = '\r';
                i32Head = i32Tmp;
            }
        }

        i32Tmp = i32Head+1;
        if (i32Tmp > BUF_SIZE)
        {
            i32Tmp = 0;
        }
        if(i32Tmp != i32Tail)
        {
            u8Buf[i32Head] = ch;
            i32Head = i32Tmp;
        }
    }
    else
    {
        if (i32Tail == i32Head)
        {
            return;
        }
    }

    /* pop char */
    do
    {
        i32Tmp = i32Tail + 1;
        if(i32Tmp > BUF_SIZE)
        {
            i32Tmp = 0;
        }

        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) == 0UL)
        {
            DEBUG_PORT->DAT = u8Buf[i32Tail];
            i32Tail = i32Tmp;
        }
        else
        {
            break; /* FIFO full */
        }
    }while(i32Tail != i32Head);
}
#endif   /* else for NONBLOCK_PRINTF */
#endif   /* if not def DISABLE_UART */

/**
 * @brief       Routine to send a char
 *
 * @param[in]   ch Character to send to debug port.
 *
 * @returns     Send value from UART debug port or semihost
 *
 * @details     Send a target char to UART debug port or semihost.
 */
void SendChar(int ch)
{
#if defined(DEBUG_ENABLE_SEMIHOST)
    g_buf[g_buf_len++] = ch;
    g_buf[g_buf_len] = '\0';
    if(g_buf_len + 1 >= sizeof(g_buf) || ch == '\n' || ch == '\0')
    {
        /* Send the char */
        if(g_ICE_Conneced)
        {
            if(SH_DoCommand(0x04, (int)g_buf, NULL) != 0)
            {
                g_buf_len = 0;
                return;
            }
        }
        else
        {
# if (DEBUG_ENABLE_SEMIHOST == 2) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=2
            int i;
            for (i = 0; i < (int)g_buf_len; i++)
            {
                SendChar_ToUART(g_buf[i]);
            }
            g_buf_len = 0;
# endif
        }
    }
#else

#ifndef DISABLE_UART
    SendChar_ToUART(ch);
#endif

#endif
}

/**
 * @brief    Routine to get a char
 *
 * @param    None
 *
 * @returns  Get value from UART debug port or semihost
 *
 * @details  Wait UART debug port or semihost to input a char.
 */
char GetChar(void)
{
#ifdef DEBUG_ENABLE_SEMIHOST
    int nRet;
# if defined (__ICCARM__)
    while(SH_DoCommand(0x7, 0, &nRet) != 0)
    {
        if (nRet != 0)
        {
            return (char)nRet;
        }
    }
# else
    while(SH_DoCommand(0x101, 0, &nRet) != 0)
    {
        if(nRet != 0)
        {
            SH_DoCommand(0x07, 0, &nRet);
            return (char)nRet;
        }
    }
# endif

# if (DEBUG_ENABLE_SEMIHOST == 2) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=2
    /* Use debug port when ICE is not connected at semihost mode */
    while(!g_ICE_Conneced)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0UL)
        {
            return (DEBUG_PORT->DAT);
        }
    }
# endif

    return (0);
#else

#ifndef DISABLE_UART
    while(1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0UL)
        {
            return (DEBUG_PORT->DAT);
        }
    }
#else
    return 0;
#endif

#endif
}

/**
 * @brief    Check any char input from UART
 *
 * @param    None
 *
 * @retval   1: No any char input
 * @retval   0: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */

int kbhit(void)
{
#ifndef DISABLE_UART
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0UL) ? 1 : 0;
#else
    return 1;
#endif
}
/**
 * @brief    Check if debug message finished
 *
 * @param    None
 *
 * @retval   1: Message is finished
 * @retval   0: Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */

int IsDebugFifoEmpty(void)
{
#ifndef DISABLE_UART
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0UL) ? 1 : 0;
#else
    return 1;
#endif
}

/**
 * @brief       C library retargetting
 *
 * @param[in]   ch Character to send to debug port.
 *
 * @returns     None
 *
 * @details     Check if message finished (FIFO empty of debug port)
 */

void _ttywrch(int ch)
{
    SendChar(ch);
    return;
}


/**
 * @brief      Write character to stream
 *
 * @param[in]  ch       Character to be written. The character is passed as its int promotion.
 * @param[in]  stream   Pointer to a FILE object that identifies the stream where the character is to be written.
 *
 * @returns    If there are no errors, the same character that has been written is returned.
 *             If an error occurs, EOF is returned and the error indicator is set (see ferror).
 *
 * @details    Writes a character to the stream and advances the position indicator.\n
 *             The character is written at the current position of the stream as indicated \n
 *             by the internal position indicator, which is then advanced one character.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/fputc/.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> fputc is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's low-level character output
 *                       to the UART debug port / semihosting channel. Renaming it
 *                       would break the C library's retargeting mechanism, which
 *                       depends on this exact reserved name being provided.
 */
/* cppcheck-suppress misra-c2012-21.2 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
#if (defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6040000)) || (defined(__ICCARM__) && (__VER__ >= 8000000))
    /* Keep the C library retarget FILE member referenced for ARMCC/IAR compatibility. */
    (void)__stdout.handle;
#endif
    SendChar(ch);
    return ch;
}


#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
#if !defined(OS_USE_SEMIHOSTING)
int _write(int fd, char *ptr, int len)
{
    int i = len;

    (void)fd; /* [MISRA2.7] */

    while (i--)
    {
        if (*ptr == '\n')
        {
            while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
            {
                /* wait */ /* [MISRA15.6] */
            }
            DEBUG_PORT->DAT = (uint32_t)'\r';
        }

        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
        {
            /* wait */ /* [MISRA15.6] */
        }
        DEBUG_PORT->DAT = (uint32_t)(*ptr);
        ++ptr;
    }
    return len;
}

int _read(int fd, char *ptr, int len)
{
    (void)fd;  /* [MISRA2.7] */
    (void)len; /* [MISRA2.7] */

    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0UL)
    {
        /* wait */
    }
    *ptr = DEBUG_PORT->DAT;
    return 1;


}
#endif

#else

/**
 * @brief      Get character from UART debug port or semihosting input
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream on which the operation is to be performed.
 *
 * @returns    The character read from UART debug port or semihosting
 *
 * @details    For get message from debug port or semihosting.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> fgetc is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's low-level character input
 *                       to the UART debug port / semihosting channel. Renaming it
 *                       would break the C library's retargeting mechanism, which
 *                       depends on this exact reserved name being provided.
 */
/* cppcheck-suppress misra-c2012-21.2 */
int fgetc(FILE *stream)
{
    (void)stream;
    return ((int)GetChar());
}

/**
 * @brief      Check error indicator
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream.
 *
 * @returns    If the error indicator associated with the stream was set, the function returns a nonzero value.
 *             Otherwise, it returns a zero value.
 *
 * @details    Checks if the error indicator associated with stream is set, returning a value different
 *             from zero if it is. This indicator is generally set by a previous operation on the stream that failed.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/ferror/.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> ferror is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's stream error-indicator query
 *                       to this BSP's debug/semihosting channel. Renaming it would
 *                       break the C library's retargeting mechanism, which depends
 *                       on this exact reserved name being provided.
 */
/* cppcheck-suppress misra-c2012-21.2 */
int ferror(FILE *stream)
{
    (void)stream;
    return EOF;
}

#endif


#ifdef DEBUG_ENABLE_SEMIHOST

# ifdef __ICCARM__
void __exit(int return_code)
{
    /* Check if link with ICE */
    if(SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
            /* wait */
        }
    }
label:
    goto label;  /* endless loop */
}
# else
void _sys_exit(int return_code)
{
    (void)return_code;
    /* Check if link with ICE */
    if(SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
            /* wait */
        }
    }
label:
    goto label;  /* endless loop */
}
# endif  // ifdef __ICCARM__

#endif  /* ifdef DEBUG_ENABLE_SEMIHOST */
