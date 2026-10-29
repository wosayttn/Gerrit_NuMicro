/**************************************************************************//**
 * @file     queue.c
 * @version  V1.00
 * @brief    It is used to be as an event queue.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "queue.h"


/* Ring-buffer indices shared between interrupt and foreground contexts. */
volatile uint32_t front = 0, rear = 0;

/** @brief Check whether the CIR event queue is full. */
int32_t isFull(void)
{
    int32_t i32Ret;

    __set_PRIMASK(1u);

    if( ((rear == MAX_QUEUE) && (front == 0)) || (rear == (front-1)) )
        i32Ret = 1;
    else
        i32Ret = 0;

    __set_PRIMASK(0u);

    return i32Ret;
}

/** @brief Check whether the CIR event queue contains no pending events. */
int32_t isEmpty(void)
{
    int32_t i32Ret = 0;

    __set_PRIMASK(1u);

    if(front == rear)
        i32Ret = 1;
    __set_PRIMASK(0u);

    return i32Ret;
}

/** @brief Append one CIR event to the ring buffer. */
void Push(uint32_t* queue, uint32_t item)
{
    if (isFull())
    {
        printf("Queue is full!\n");
        return;
    }

    __set_PRIMASK(1u);

    queue[rear] = item;
    if(++rear >= MAX_QUEUE)
        rear = 0;

    __set_PRIMASK(0u);
}

/** @brief Remove and return the oldest CIR event from the ring buffer. */
uint32_t Pop(uint32_t* queue)
{
    uint32_t u32Data;
    if (isEmpty())
    {
        printf("Queue is empty!\n");
        return 0;
    }

    __set_PRIMASK(1u);

    u32Data = queue[front];
    if(++front >= MAX_QUEUE)
        front = 0;

    __set_PRIMASK(0u);

    return u32Data;
}
