/*
 * Copyright (c) 2022 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ethosu_cpu_cache.h"

#include "ethosu_driver.h"          /* Arm Ethos-U driver header */
#include "log_macros.h"             /* Logging macros */

#include "NuMicro.h"

void ethosu_flush_dcache(uint32_t *p, size_t bytes)
{
    UNUSED(p);
    UNUSED(bytes);
}

void ethosu_invalidate_dcache(uint32_t *p, size_t bytes)
{
    if (SCB->CCR & SCB_CCR_DC_Msk)
    {
        if (p)
        {
            SCB_InvalidateDCache_by_Addr((void *) p, (int32_t) bytes);
        }
        else
        {
            SCB_InvalidateDCache();
        }
    }
}
