/**************************************************************************//**
 * @file     retarget.c
 * @version  V0.10
 * @brief    M253 series Debug Port and Semihost Setting Source File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>

#if defined(__ICCARM__)
    #define NVT_RETARGET_COMPILER_IAR          1
    #define NVT_RETARGET_IAR_VERSION           __VER__
#else
    #define NVT_RETARGET_COMPILER_IAR          0
    #define NVT_RETARGET_IAR_VERSION           0
#endif

#if defined(__ARMCC_VERSION)
    #define NVT_RETARGET_COMPILER_ARMCC        1
    #define NVT_RETARGET_ARMCC_VERSION         __ARMCC_VERSION
#else
    #define NVT_RETARGET_COMPILER_ARMCC        0
    #define NVT_RETARGET_ARMCC_VERSION         0
#endif

#if defined(__GNUC__)
    #define NVT_RETARGET_COMPILER_GCC          1
#else
    #define NVT_RETARGET_COMPILER_GCC          0
#endif

#if defined(__MICROLIB)
    #define NVT_RETARGET_MICROLIB_ENABLED      1
#else
    #define NVT_RETARGET_MICROLIB_ENABLED      0
#endif

#if defined(__OPTIMIZE__)
    #define NVT_RETARGET_OPTIMIZE_LEVEL        __OPTIMIZE__
#else
    #define NVT_RETARGET_OPTIMIZE_LEVEL        1
#endif

#if defined(__ARM_FEATURE_CMSE)
    #define NVT_RETARGET_ARM_FEATURE_CMSE      __ARM_FEATURE_CMSE
#else
    #define NVT_RETARGET_ARM_FEATURE_CMSE      0
#endif

#if defined(DEBUG_ENABLE_SEMIHOST)
    #define NVT_RETARGET_DEBUG_SEMIHOST_LEVEL  DEBUG_ENABLE_SEMIHOST
    #define NVT_RETARGET_SEMIHOST_ENABLED      1
#else
    #define NVT_RETARGET_DEBUG_SEMIHOST_LEVEL  0
    #define NVT_RETARGET_SEMIHOST_ENABLED      0
#endif

#if defined(OS_USE_SEMIHOSTING)
    #define NVT_RETARGET_OS_USE_SEMIHOSTING    1
#else
    #define NVT_RETARGET_OS_USE_SEMIHOSTING    0
#endif

#if defined(STDIN_ECHO)
    #define NVT_RETARGET_STDIN_ECHO            STDIN_ECHO
#else
    #define NVT_RETARGET_STDIN_ECHO            0
#endif

#if NVT_RETARGET_COMPILER_IAR && (NVT_RETARGET_IAR_VERSION >= 9000000)
    #define NVT_RETARGET_USE_IAR_LOWLEVELIO    1
    #include <LowLevelIOInterface.h>
#else
    #define NVT_RETARGET_USE_IAR_LOWLEVELIO    0
#endif

#if NVT_RETARGET_COMPILER_ARMCC || NVT_RETARGET_COMPILER_IAR
    #define NVT_RETARGET_USE_SEMIHOST_RUNTIME  1
#else
    #define NVT_RETARGET_USE_SEMIHOST_RUNTIME  0
#endif

#if NVT_RETARGET_COMPILER_GCC && !NVT_RETARGET_COMPILER_ARMCC
    #define NVT_RETARGET_USE_GCC_SYSCALLS      1
#else
    #define NVT_RETARGET_USE_GCC_SYSCALLS      0
#endif

#if NVT_RETARGET_COMPILER_ARMCC || (NVT_RETARGET_COMPILER_IAR && !NVT_RETARGET_USE_IAR_LOWLEVELIO)
    #define NVT_RETARGET_USE_STDIO_HOOKS       1
#else
    #define NVT_RETARGET_USE_STDIO_HOOKS       0
#endif

#if (NVT_RETARGET_ARM_FEATURE_CMSE == 3)
    #define NVT_RETARGET_USE_CMSE              1
#else
    #define NVT_RETARGET_USE_CMSE              0
#endif

#if (NVT_RETARGET_DEBUG_SEMIHOST_LEVEL == 1)
    #define NVT_RETARGET_SEMIHOST_UART_FALLBACK 1
#else
    #define NVT_RETARGET_SEMIHOST_UART_FALLBACK 0
#endif

#if (NVT_RETARGET_STDIN_ECHO != 0)
    #define NVT_RETARGET_STDIN_ECHO_ENABLED    1
#else
    #define NVT_RETARGET_STDIN_ECHO_ENABLED    0
#endif

#if NVT_RETARGET_COMPILER_IAR
    #define NVT_RETARGET_WEAK                  __WEAK
    #pragma diag_suppress=Pm150
#else
    #define NVT_RETARGET_WEAK                  __attribute__((weak))
#endif

#include "NuMicro.h"

#if defined(__CC_ARM)
    #if (NVT_RETARGET_ARMCC_VERSION >= 400000)
        /* Insist on keeping widthprec, to avoid X propagation by benign code in C-lib */
        #pragma import _printf_widthprec
    #endif
#endif

#ifndef DEBUG_PORT
    #define DEBUG_PORT   UART4
#endif

#define BUF_SIZE     512

#if ((NVT_RETARGET_ARMCC_VERSION > 0) && (NVT_RETARGET_ARMCC_VERSION < 6040000)) || \
    (NVT_RETARGET_COMPILER_IAR && (NVT_RETARGET_IAR_VERSION >= 8000000) && (NVT_RETARGET_IAR_VERSION < 9000000))
    #define NVT_RETARGET_NEEDS_FILE_HANDLE    1
#else
    #define NVT_RETARGET_NEEDS_FILE_HANDLE    0
#endif

#if (NVT_RETARGET_ARMCC_VERSION > 0) && !NVT_RETARGET_COMPILER_IAR && \
    !NVT_RETARGET_MICROLIB_ENABLED && (NVT_RETARGET_OPTIMIZE_LEVEL == 0) && \
    (NVT_RETARGET_ARMCC_VERSION < 6150000)
    #define NVT_RETARGET_ARMCC_USE_NO_ARGV    1
#else
    #define NVT_RETARGET_ARMCC_USE_NO_ARGV    0
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if NVT_RETARGET_NEEDS_FILE_HANDLE
struct __FILE
{
    int handle; /* Add whatever you need here */
};
#elif NVT_RETARGET_ARMCC_USE_NO_ARGV
    __ASM(".global __ARM_use_no_argv\n\t" "__ARM_use_no_argv:\n\t");
#endif /* NVT_RETARGET_NEEDS_FILE_HANDLE */

#if NVT_RETARGET_NEEDS_FILE_HANDLE
extern FILE __stdout;
extern FILE __stdin;

FILE __stdout = {0};
FILE __stdin = {0};
#endif

#if NVT_RETARGET_USE_SEMIHOST_RUNTIME
    extern int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);
    /* cppcheck-suppress misra-c2012-8.2 */
    extern uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp);
#endif

int kbhit(void);
int IsDebugFifoEmpty(void);
void _ttywrch(int ch);

#if NVT_RETARGET_USE_GCC_SYSCALLS
    #if !NVT_RETARGET_OS_USE_SEMIHOSTING
        int _read(int fd, char *ptr, int len);
    #endif
    int _write(int fd, char *ptr, int len);
#endif

char GetChar(void);
void SendChar_ToUART(int ch);
void SendChar(int ch);
#if NVT_RETARGET_SEMIHOST_ENABLED
    static volatile int32_t g_ICE_Connected = 1;
#endif
enum { r0, r1, r2, r3, r12, lr, pc, psr};


/**
 * @brief       Helper function to dump register while hard fault occurred
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @details     This function is implement to print r0, r1, r2, r3, r12, lr, pc, psr
 */
static void DumpStack(uint32_t stack[])
{
    (void)stack;
    /*
        printf("r0 =0x%x\n", stack[r0]);
        printf("r1 =0x%x\n", stack[r1]);
        printf("r2 =0x%x\n", stack[r2]);
        printf("r3 =0x%x\n", stack[r3]);
        printf("r12=0x%x\n", stack[r12]);
        printf("lr =0x%x\n", stack[lr]);
        printf("pc =0x%x\n", stack[pc]);
        printf("psr=0x%x\n", stack[psr]);
    */
}

static void NVT_Retarget_Halt(void)
{
    for (;;)
    {
    }
}

#if NVT_RETARGET_SEMIHOST_ENABLED

/* The static buffer is used to speed up the semihost */
static char g_buf[16];
static uint8_t g_buf_len = 0;

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
    (void)n32In_R1;

    if (g_ICE_Connected)
    {
        if (pn32Out_R0)
        {
            *pn32Out_R0 = n32In_R0;
        }

        return 1;
    }

    return 0;
}

#else /* NVT_RETARGET_SEMIHOST_ENABLED */

#if NVT_RETARGET_USE_GCC_SYSCALLS

/**
 * @brief    This HardFault handler is implemented to show r0, r1, r2, r3, r12, lr, pc, psr
 *
 * @details  This function is implement to print r0, r1, r2, r3, r12, lr, pc, psr.
 *
 */
__attribute__((weak)) void HardFault_Handler(void)
{
    __ASM(
        "MOV     R0, LR  \n"
        "MRS     R1, MSP \n"
        "MRS     R2, PSP \n"
        "LDR     R3, =ProcessHardFault \n"
        "BLX     R3 \n"
        "BX      R0 \n"
    );
}

#else

int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R0;
    (void)n32In_R1;
    (void)pn32Out_R0;
    return 0;
}

#endif

#endif /* NVT_RETARGET_SEMIHOST_ENABLED */


NVT_RETARGET_WEAK uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp;
    uint32_t inst;

    /* It is caused by hardfault. Just process the hard fault */
    /* TODO: Implement your hardfault handle code here */

    /* Check the used stack */
#if NVT_RETARGET_USE_CMSE

    if (lr & 0x40UL)
    {
#endif

        /* Secure stack used */
        if (lr & 4UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }

#if NVT_RETARGET_USE_CMSE
    }
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
    inst = M16(sp[6]);

    if (inst == 0xBEABUL)
    {
        /*
            If the instruction is 0xBEAB, it means it is caused by BKPT without ICE connected.
            We still return for output/input message to UART.
        */
#if NVT_RETARGET_SEMIHOST_ENABLED
        g_ICE_Connected = 0; // Set a flag for ICE offline
#endif
        sp[6] += 2UL;          // Return to next instruction
        return lr;           // Keep lr in R0
    }

    {
        static const char acHardFaultMsg[] = "  HardFault!\n\n";
        uint32_t u32Idx;

        for (u32Idx = 0U; u32Idx < (sizeof(acHardFaultMsg) - 1U); u32Idx++)
        {
            SendChar(acHardFaultMsg[u32Idx]);
        }
    }

    DumpStack(sp);

    // Halt here
    NVT_Retarget_Halt();

    return 0;
}


/**
 * @brief    Routine to send a char
 *
 * @param[in] ch  A character data writes to debug port
 *
 * @details  Send a target char to UART debug port .
 */
#ifndef NONBLOCK_PRINTF
void SendChar_ToUART(int ch)
{
    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0U)
    {
    }

    if ((char)ch == '\n')
    {
        DEBUG_PORT->DAT = '\r';

        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0U)
        {
        }
    }

    DEBUG_PORT->DAT = (uint32_t)ch;
}

#else

/* Non-block implement of send char */
void SendChar_ToUART(int ch)
{
    static uint8_t u8Buf[BUF_SIZE] = {0};
    static int32_t i32Head = 0;
    static int32_t i32Tail = 0;
    int32_t i32Tmp;

    /* Only flush the data in buffer to UART when ch == 0 */
    if (ch)
    {
        // Push char

        if (ch == '\n')
        {
            i32Tmp = i32Head + 1;

            if (i32Tmp > BUF_SIZE)
            {
                i32Tmp = 0;
            }

            if (i32Tmp != i32Tail)
            {
                u8Buf[i32Head] = '\r';
                i32Head = i32Tmp;
            }
        }

        i32Tmp = i32Head + 1;

        if (i32Tmp > BUF_SIZE)
        {
            i32Tmp = 0;
        }

        if (i32Tmp != i32Tail)
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

    // Pop char
    do
    {
        i32Tmp = i32Tail + 1;

        if (i32Tmp > BUF_SIZE)
        {
            i32Tmp = 0;
        }

        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) == 0)
        {
            DEBUG_PORT->DAT = u8Buf[i32Tail];
            i32Tail = i32Tmp;
        }
        else
        {
            break; // FIFO full
        }
    } while (i32Tail != i32Head);
}

#endif /* else for NONBLOCK_PRINTF */


/**
 * @brief    Routine to send a char
 *
 * @param[in] ch : A character data writes to debug port
 *
 * @returns  None
 *
 * @details  Send a target char to UART debug port or semihost.
 */

NVT_RETARGET_WEAK void SendChar(int ch)
{
#if NVT_RETARGET_SEMIHOST_ENABLED
    g_buf[g_buf_len++] = ch;
    g_buf[g_buf_len] = '\0';

    if (g_buf_len + 1 >= sizeof(g_buf) || ch == '\n' || ch == '\0')
    {
        /* Send the char */
        if (g_ICE_Connected)
        {

            if (SH_DoCommand(0x04, (int)g_buf, NULL) != 0)
            {
                g_buf_len = 0;

                return;
            }
        }
        else
        {
#if NVT_RETARGET_SEMIHOST_UART_FALLBACK
            int i;

            for (i = 0; i < g_buf_len; i++)
            {
                SendChar_ToUART(g_buf[i]);
            }

#endif
            g_buf_len = 0;
        }
    }

#else
    SendChar_ToUART(ch);
#endif /* NVT_RETARGET_SEMIHOST_ENABLED */
}


/**
 * @brief    Routine to get a char
 *
 *
 * @returns  Get value from UART debug port or semihost
 *
 * @details  Wait UART debug port or semihost to input a char.
 */
char GetChar(void)
{
#if NVT_RETARGET_SEMIHOST_ENABLED

    if (g_ICE_Connected)
    {
#if NVT_RETARGET_COMPILER_IAR
        int nRet;

        while (SH_DoCommand(0x7, 0, &nRet) != 0)
        {
            if (nRet != 0)
            {
                return (char)nRet;
            }
        }

#else
        int nRet;

        while (SH_DoCommand(0x101, 0, &nRet) != 0)
        {
            if (nRet != 0)
            {
                SH_DoCommand(0x07, 0, &nRet);
                return (char)nRet;
            }
        }

#endif

    }
    else
    {

#if NVT_RETARGET_SEMIHOST_UART_FALLBACK

        /* Use debug port when ICE is not connected at semihost mode */
        while (!g_ICE_Connected)
        {
            if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
            {
                return ((char)DEBUG_PORT->DAT);
            }
        }

#endif
    }

    return (0);

#else

    for (;;)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }

#endif
}


/**
 * @brief    Check any char input from UART
 *
 *
 * @retval   1: No any char input
 * @retval   0: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */
int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U);
}


/**
 * @brief  Check if debug message finished
 *
 * @return   1 Message is finished.
 *           0 Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}


/**
 * @brief    C library retargetting
 *
 * @param[in]  ch  Write a character data
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
void _ttywrch(int ch)
{
#if NVT_RETARGET_NEEDS_FILE_HANDLE
    (void)__stdout.handle;
    (void)__stdin.handle;
#endif

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
 *
 */
#if NVT_RETARGET_USE_IAR_LOWLEVELIO
size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    size_t nChars = 0;

    if (buffer == 0)
    {
        /*
        * This means that we should flush internal buffers.  Since we
        * don't we just return.  (Remember, "handle" == -1 means that all
        * handles should be flushed.)
        */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
    * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size != 0; --size)
    {
        SendChar(*buffer++);
        ++nChars;
    }

    return nChars;
}

#elif NVT_RETARGET_USE_STDIO_HOOKS
/* cppcheck-suppress misra-c2012-8.4 */
/* cppcheck-suppress misra-c2012-21.2 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    SendChar(ch);
    return ch;
}

#endif

#if NVT_RETARGET_USE_GCC_SYSCALLS

#if NVT_RETARGET_OS_USE_SEMIHOSTING

#else

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    int i = len;

    while (i--)
    {
        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0U)
        {
        }

        if (*ptr == '\n')
        {
            DEBUG_PORT->DAT = '\r';

            while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0U)
            {
            }
        }

        DEBUG_PORT->DAT = *ptr++;
    }

    return len;
}


int _read(int fd, char *ptr, int len)
{
    (void)fd;
    (void)len;

    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0U)
    {
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
 */
#if NVT_RETARGET_USE_IAR_LOWLEVELIO
size_t __read(int handle, unsigned char *buffer, size_t size)
{
    /* Remove the #if #endif pair to enable the implementation */
    int nChars = 0;

    /* This template only reads from "standard in", for all other file
     * handles it returns failure. */
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size > 0; --size)
    {
        int c = GetChar();

        if (c < 0)
        {
            break;
        }

#if NVT_RETARGET_STDIN_ECHO_ENABLED
        SendChar(c);
#endif

        *buffer++ = c;
        ++nChars;
    }

    return nChars;
}

long __lseek(int handle, long offset, int whence)
{
    (void)handle;
    (void)offset;
    (void)whence;
    return -1;
}

#elif NVT_RETARGET_USE_STDIO_HOOKS
/* cppcheck-suppress misra-c2012-8.4 */
/* cppcheck-suppress misra-c2012-21.2 */
int fgetc(FILE *stream)
{
    (void)stream;
    return ((int)GetChar());
}

#endif

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
 */
#if NVT_RETARGET_USE_STDIO_HOOKS
/* cppcheck-suppress misra-c2012-8.4 */
/* cppcheck-suppress misra-c2012-21.2 */
int ferror(FILE *stream)
{
    (void)stream;
    return EOF;
}
#endif

#endif


#if NVT_RETARGET_SEMIHOST_ENABLED
#if NVT_RETARGET_COMPILER_IAR
void __exit(int return_code)
{
    (void)return_code;

    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
        }
    }

    NVT_Retarget_Halt();
}

#else

void _sys_exit(int return_code)
{
    (void)return_code;

    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
        }
    }

    NVT_Retarget_Halt();
}

#endif
#endif
