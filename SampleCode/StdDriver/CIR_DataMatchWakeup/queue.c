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

/*
 * front points to the next queue entry to be read.
 * rear points to the next queue entry to be written.
 * Both variables are shared between interrupt and main-code contexts, so queue index updates are protected by
 * PRIMASK to avoid being interrupted in the middle of a queue operation.
 */
volatile uint32_t front = 0, rear = 0;

int32_t isFull(void)
{
    int32_t i32Ret;

    /* Temporarily disable interrupts while reading the shared queue indexes. */
    __set_PRIMASK(1u);

    if(((rear == MAX_QUEUE) && (front == 0)) || (rear == (front - 1)))
        i32Ret = 1;
    else
        i32Ret = 0;

    /* Restore interrupt handling after the shared-index check is complete. */
    __set_PRIMASK(0u);

    return i32Ret;
}

int32_t isEmpty(void)
{
    int32_t i32Ret = 0;

    /* The queue is empty when both indexes point to the same position. */
    __set_PRIMASK(1u);

    if(front == rear)
        i32Ret = 1;

    __set_PRIMASK(0u);

    return i32Ret;
}

void Push(uint32_t* queue, uint32_t item)
{
    /* Do not overwrite unread CIR events when the software queue is full. */
    if(isFull())
    {
        printf("Queue is full!\n");
        return;
    }

    __set_PRIMASK(1u);

    /* Store the new event at the current rear position, then advance the write index. */
    queue[rear] = item;
    if(++rear >= MAX_QUEUE)
        rear = 0;

    __set_PRIMASK(0u);
}

uint32_t Pop(uint32_t* queue)
{
    uint32_t u32Data;

    /* Return zero when no CIR event is waiting in the software queue. */
    if(isEmpty())
    {
        printf("Queue is empty!\n");
        return 0;
    }

    __set_PRIMASK(1u);

    /* Read the oldest event, then advance the read index to the next queue entry. */
    u32Data = queue[front];
    if(++front >= MAX_QUEUE)
        front = 0;

    __set_PRIMASK(0u);

    return u32Data;
}
