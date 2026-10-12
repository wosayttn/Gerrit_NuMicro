/*
 * RNDIS Dashboard fixed backing storage for lwIP mem_malloc().
 *
 * This file is deliberately unguarded: lwIP includes it repeatedly through
 * memp_std.h with different LWIP_MALLOC_MEMPOOL macro definitions.
 */
LWIP_MALLOC_MEMPOOL_START
LWIP_MALLOC_MEMPOOL(4, 768)
LWIP_MALLOC_MEMPOOL_END