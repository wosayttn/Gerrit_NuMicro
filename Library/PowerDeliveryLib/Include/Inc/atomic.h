/* Copyright 2012 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Atomic operations for ARMv7 */

#ifndef __CROS_EC_ATOMIC_H
#define __CROS_EC_ATOMIC_H

//#include "atomic_t.h"
#include "common.h"

#include <stdbool.h>


typedef long atomic_t;
typedef long atomic_val_t;

#if defined(__ICCARM__)
typedef unsigned int irq_state_t;

static inline irq_state_t get_PRIMASK(void)
{
    uint32_t r;
    __asm volatile ("MRS %0, PRIMASK" : "=r"(r));
    return r;
}

static inline void set_PRIMASK(uint32_t v)
{
    __asm volatile ("MSR PRIMASK, %0" :: "r"(v));
}

static inline void disable_irq(void)
{
    __asm volatile ("CPSID i");
}

static inline irq_state_t irq_save(void)
{
    irq_state_t s = get_PRIMASK();
    disable_irq();
    return s;
}

static inline void irq_restore(irq_state_t s)
{
    set_PRIMASK(s);
}
#endif

static inline atomic_val_t atomic_clear_bits(atomic_t *addr, atomic_val_t bits)
{
#if defined(__ICCARM__)
    atomic_val_t old;
    __DMB();
    irq_state_t key = irq_save();
    old = *addr;
    *addr = old & ~bits;
    irq_restore(key);
    __DMB();
    return old;
#else
    return __atomic_fetch_and(addr, ~bits, __ATOMIC_SEQ_CST);
#endif
}

static inline atomic_val_t atomic_or(atomic_t *addr, atomic_val_t bits)
{
#if defined(__ICCARM__)
		__DMB();
    irq_state_t key = irq_save();
    atomic_val_t old = *addr;
    *addr = old | bits;
    irq_restore(key);
		__DMB();
    return old; 
#else    
    return __atomic_fetch_or(addr, bits, __ATOMIC_SEQ_CST);
#endif    
}

static inline atomic_val_t atomic_add(atomic_t *addr, atomic_val_t value)
{
#if defined(__ICCARM__)	
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t old = *addr;
		*addr = old + value;
		irq_restore(key);
		__DMB();
		return old;
#else	
    return __atomic_fetch_add(addr, value, __ATOMIC_SEQ_CST);
#endif	
}

static inline atomic_val_t atomic_sub(atomic_t *addr, atomic_val_t value)
{
#if defined(__ICCARM__)	
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t old = *addr;
		*addr = old - value;
		irq_restore(key);
		__DMB();
		return old;
#else	
    return __atomic_fetch_sub(addr, value, __ATOMIC_SEQ_CST);
#endif	
}

static inline atomic_val_t atomic_clear(atomic_t *addr)
{
#if defined(__ICCARM__)	
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t old = *addr;
		*addr = 0;
		irq_restore(key);
		__DMB();
		return old;
#else	
    return __atomic_exchange_n(addr, 0, __ATOMIC_SEQ_CST);
#endif	
}

static inline atomic_val_t atomic_and(atomic_t *addr, atomic_val_t bits)
{
#if defined(__ICCARM__)		
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t old = *addr;
		*addr = old & bits;
		irq_restore(key);
		__DMB();
	  return old;
#else	
    return __atomic_fetch_and(addr, bits, __ATOMIC_SEQ_CST);
#endif	
}

static inline bool atomic_compare_exchange(atomic_t *addr,
        atomic_val_t *expected,
        atomic_val_t desired)
{
#if defined(__ICCARM__)		
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t cur = *addr;
		if (cur == *expected) {
				*addr = desired;
				irq_restore(key);
				__DMB();
				return true;
		} else {
				*expected = cur; // ????????
				irq_restore(key);
				__DMB();
				return false;
		}
#else		
    return __atomic_compare_exchange_n(addr, expected, desired, false,
                                       __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
#endif		
}

static inline atomic_val_t atomic_exchange(atomic_t *addr, atomic_val_t value)
{
#if defined(__ICCARM__)	
		__DMB();
		irq_state_t key = irq_save();
		atomic_val_t old = *addr;
		*addr = value;
		irq_restore(key);
		__DMB();
		return old;
#else	
    return __atomic_exchange_n(addr, value, __ATOMIC_SEQ_CST);
#endif	
}

static inline atomic_val_t atomic_load(atomic_t *addr)
{
#if defined(__ICCARM__)	
		__DMB();
		atomic_val_t v = *addr;
		__DMB();
		return v;
#else		
    return __atomic_load_n(addr, __ATOMIC_SEQ_CST);
#endif	
}

#endif /* __CROS_EC_ATOMIC_H */
