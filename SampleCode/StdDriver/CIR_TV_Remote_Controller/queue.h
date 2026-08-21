/**************************************************************************//**
 * @file     queue.h
 * @version  V0.10
 * @brief    M471 series CIR Sample Code Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif


#define MAX_QUEUE 20

int32_t isFull(void);
int32_t isEmpty(void);
void Push(uint32_t* queue, uint32_t item);
uint32_t Pop(uint32_t* queue);

#ifdef __cplusplus
}
#endif

#endif  /* QUEUE_H */

