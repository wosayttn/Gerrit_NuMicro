/**************************************************************************//**
 * @file     mpu_config_M3351_template.h
 * @version  V1.00
 * @brief    Initial Setup for MPU region for NuMicro M3351
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __MPU_CONFIG_M3351_TEMPLATE_H__
#define __MPU_CONFIG_M3351_TEMPLATE_H__

/*
 * Copy from Library\StdDriver\inc\mpu_config_M3351_template.h
 * Rename to mpu_config_M3351.h
 * Add mpu_config_M3351.h folder to include path
 */

#define MPU_REGIONS_MAX     8U                 /* Max. number of MPU regions */
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <h>Memory Protection Unit (MPU) Regions
// <h>Pre-defined MPU attribute
*/
/*
// <q.0> 0: eMPU_ATTR_DEV_nGnRnE
// <i> Device memory type non Gathering, non Re-ordering, non Early Write Acknowledgement
// <q.1> 1: eMPU_ATTR_DEV_nGnRE
// <i> Device memory type non Gathering, non Re-ordering, Early Write Acknowledgement
// <q.2> 2: eMPU_ATTR_DEV_nGRE
// <i> Device memory type non Gathering, Re-ordering, Early Write Acknowledgement
// <q.3> 3: eMPU_ATTR_DEV_GRE
// <i> Device memory type Gathering, Re-ordering, Early Write Acknowledgement
// <q.4> 4: eMPU_ATTR_NON_CACHEABLE
// <i> Attribute for non-cacheable normal memory
// <q.5> 5: eMPU_ATTR_CACHEABLE_WTRA
// <i> Attribute for cacheable normal memory with Non-transient, Write-Through, Read-allocate, Not Write-allocate
// <q.6> 6: eMPU_ATTR_CACHEABLE_WBWARA
// <i> Attribute for cacheable normal memory with Non-transient, Write-Back, Read-allocate, Write-allocate
*/
#define MPU_INIT_MEM_ATTRS  0x7FU
/*
// </h>
*/
/*
//   <e> MPU Region 0 (Lowest priority)
//   <i> Setup MPU Region 0 Attributes
//   <i> Default Set to EBI Memory Space
*/
#define MPU_INIT_REGION0    1U
/*
//     <o>Base Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE0      0x60000000U       /* Base address of MPU region 0 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE0      0x00300000U       /* Size of MPU region 0 */
#define MPU_INIT_LIMIT0     ((uint32_t)MPU_INIT_BASE0 + (uint32_t)MPU_INIT_SIZE0 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR0       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR0   0xCU
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION0 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE0 < 32U) || ((MPU_INIT_SIZE0 % 32U) != 0U) || ((MPU_INIT_BASE0 % 32U) != 0U))
        #error "MPU region 0 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR0) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 0 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 1
//   <i> Setup MPU Region 1 Attributes
//   <i> Default Set to Aliased SRAM + HyperRAM Memory Space
*/
#define MPU_INIT_REGION1    0U
/*
//     <o>Base Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE1      0x00000000U       /* Base address of MPU region 1 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE1      0x00100000U       /* Size of MPU region 1 */
#define MPU_INIT_LIMIT1     ((uint32_t)MPU_INIT_BASE1 + (uint32_t)MPU_INIT_SIZE1 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR1       0x6U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR1   0x4U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION1 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE1 < 32U) || ((MPU_INIT_SIZE1 % 32U) != 0U) || ((MPU_INIT_BASE1 % 32U) != 0U))
        #error "MPU region 1 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR1) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 1 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 2
//   <i> Setup MPU Region 2 Attributes
*/
#define MPU_INIT_REGION2    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE2      0x00000000U       /* Base address of MPU region 2 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE2      0x00100000U       /* Size of MPU region 2 */
#define MPU_INIT_LIMIT2     ((uint32_t)MPU_INIT_BASE2 + (uint32_t)MPU_INIT_SIZE2 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR2       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR2   0xCU
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION2 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE2 < 32U) || ((MPU_INIT_SIZE2 % 32U) != 0U) || ((MPU_INIT_BASE2 % 32U) != 0U))
        #error "MPU region 2 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR2) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 2 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 3
//   <i> Setup MPU Region 3 Attributes
*/
#define MPU_INIT_REGION3    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE3      0x00000000U       /* Base address of MPU region 3 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE3      0x00010000U       /* Size of MPU region 3 */
#define MPU_INIT_LIMIT3     ((uint32_t)MPU_INIT_BASE3 + (uint32_t)MPU_INIT_SIZE3 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR3       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR3   0x0U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION3 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE3 < 32U) || ((MPU_INIT_SIZE3 % 32U) != 0U) || ((MPU_INIT_BASE3 % 32U) != 0U))
        #error "MPU region 3 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR3) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 3 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 4
//   <i> Setup MPU Region 4 Attributes
*/
#define MPU_INIT_REGION4    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE4      0x00000000U       /* Base address of MPU region 4 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE4      0x00010000U       /* Size of MPU region 4 */
#define MPU_INIT_LIMIT4     ((uint32_t)MPU_INIT_BASE4 + (uint32_t)MPU_INIT_SIZE4 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR4       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR4   0x0U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION4 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE4 < 32U) || ((MPU_INIT_SIZE4 % 32U) != 0U) || ((MPU_INIT_BASE4 % 32U) != 0U))
        #error "MPU region 4 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR4) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 4 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 5
//   <i> Setup MPU Region 5 Attributes
*/
#define MPU_INIT_REGION5    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE5      0x00000000U       /* Base address of MPU region 5 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE5      0x00010000U       /* Size of MPU region 5 */
#define MPU_INIT_LIMIT5     ((uint32_t)MPU_INIT_BASE5 + (uint32_t)MPU_INIT_SIZE5 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR5       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR5   0x0U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION5 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE5 < 32U) || ((MPU_INIT_SIZE5 % 32U) != 0U) || ((MPU_INIT_BASE5 % 32U) != 0U))
        #error "MPU region 5 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR5) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 5 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 6
//   <i> Setup MPU Region 6 Attributes
*/
#define MPU_INIT_REGION6    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE6      0x00000000U       /* Base address of MPU region 6 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE6      0x00010000U       /* Size of MPU region 6 */
#define MPU_INIT_LIMIT6     ((uint32_t)MPU_INIT_BASE6 + (uint32_t)MPU_INIT_SIZE6 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR6       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR6   0x0U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION6 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE6 < 32U) || ((MPU_INIT_SIZE6 % 32U) != 0U) || ((MPU_INIT_BASE6 % 32U) != 0U))
        #error "MPU region 6 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR6) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 6 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 7 (Highest priority)
//   <i> Setup MPU Region 7 Attributes
*/
#define MPU_INIT_REGION7    0U
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE7      0x00000000U       /* Base address of MPU region 7 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE7      0x00010000U       /* Size of MPU region 7 */
#define MPU_INIT_LIMIT7     ((uint32_t)MPU_INIT_BASE7 + (uint32_t)MPU_INIT_SIZE7 - 1U)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR7       0x0U
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR7   0x0U
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION7 == 1U) /* Check region config */
    #if ((MPU_INIT_SIZE7 < 32U) || ((MPU_INIT_SIZE7 % 32U) != 0U) || ((MPU_INIT_BASE7 % 32U) != 0U))
        #error "MPU region 7 has invalid base address or size config !"
    #endif

    #if ((1U << MPU_MEM_ATTR7) & (MPU_INIT_MEM_ATTRS) == 0U)
        #error "MPU region 7 has invalid memory attribute !"
    #endif
#endif
/*
// </h>
*/

typedef enum
{
    eMPU_ATTR_DEV_nGnRnE,
    eMPU_ATTR_DEV_nGnRE,
    eMPU_ATTR_DEV_nGRE,
    eMPU_ATTR_DEV_GRE,
    eMPU_ATTR_NON_CACHEABLE,
    eMPU_ATTR_CACHEABLE_WTRA,
    eMPU_ATTR_CACHEABLE_WBWARA,
} E_MPU_ATTR;

typedef enum
{
    eMPU_RBAR_SH,
    eMPU_RBAR_RO,
    eMPU_RBAR_NP,
    eMPU_RBAR_XN,
} E_MPU_RBAR;

#define MPU_INIT_REGIONS            ((MPU_INIT_REGION0 << 0U) | (MPU_INIT_REGION1 << 1U) | (MPU_INIT_REGION2 << 2U) | (MPU_INIT_REGION3 << 3U) | \
                                     (MPU_INIT_REGION4 << 4U) | (MPU_INIT_REGION5 << 5U) | (MPU_INIT_REGION6 << 6U) | (MPU_INIT_REGION7 << 7U))
#define MPU_INIT_REGION(i)          ((uint32_t)(MPU_INIT_REGION##i))
#define MPU_INIT_FIXED(i)           ((uint32_t)(MPU_INIT_FIXED##i))
#define MPU_INIT_BASE(i)            ((uint32_t)(MPU_INIT_BASE##i))
#define MPU_INIT_SIZE(i)            ((uint32_t)(MPU_INIT_SIZE##i))
#define MPU_INIT_LIMIT(i)           ((uint32_t)(MPU_INIT_LIMIT##i))
#define MPU_MEM_ATTR(i)             ((uint32_t)(MPU_MEM_ATTR##i))
#define MPU_MEM_ACC_ATTR(i, index)  (((uint32_t)(MPU_MEM_ACC_ATTR##i) >> (uint32_t)(index)) & 0x1U)
#define MPU_INIT_RBAR(i, u32Base)   ((uint32_t)(ARM_MPU_RBAR(u32Base, ARM_MPU_SH_NON, MPU_MEM_ACC_ATTR(i, eMPU_RBAR_RO), MPU_MEM_ACC_ATTR(i, eMPU_RBAR_NP), MPU_MEM_ACC_ATTR(i, eMPU_RBAR_XN))))

#endif  // __MPU_CONFIG_M3351_TEMPLATE_H__
