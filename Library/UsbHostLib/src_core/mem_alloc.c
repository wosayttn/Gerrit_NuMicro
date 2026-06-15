/**************************************************************************//**
 * @file     mem_alloc.c
 * @version  V1.00
 * @brief    USB Host library memory allocation functions.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"

/// @cond HIDDEN_SYMBOLS

//#define MEM_DEBUG

#ifdef MEM_DEBUG
    #define mem_debug       (void)usbh_printf
#else
    #define mem_debug(...)
#endif


#ifdef __ICCARM__
    #pragma data_alignment = 64
    static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE];
#else
    static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE] __attribute__((aligned(64)));
#endif

#ifdef __ICCARM__
    #pragma data_alignment = 64
    static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE];
#else
    static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE] __attribute__((aligned(64)));
#endif

static uint8_t  _hw_unit_used[HW_MEM_UNIT_NUM];
static uint8_t  _dma_unit_used[DMA_MEM_UNIT_NUM];

static volatile int  _hw_mem_pool_used;
static volatile int  _dma_men_pool_used;

UDEV_T *g_udev_list;

static uint8_t  _dev_addr_pool[128];
static volatile int  _device_addr;


/*--------------------------------------------------------------------------*/
/*   Memory alloc/free recording                                            */
/*--------------------------------------------------------------------------*/

void usbh_memory_init(void)
{
    if ((int)sizeof(TD_T) > (int)HW_MEM_UNIT_SIZE)
    {
        USB_error("TD_T - MEM_POOL_UNIT_SIZE too small!\n");

        while (1)
        {
            ;
        }
    }

    if ((int)sizeof(ED_T) > (int)HW_MEM_UNIT_SIZE)
    {
        USB_error("ED_T - MEM_POOL_UNIT_SIZE too small!\n");

        while (1)
        {
            ;
        }
    }

    (void)memset(_hw_unit_used, 0, sizeof(_hw_unit_used));
    _hw_mem_pool_used = 0;

    (void)memset(_dma_unit_used, 0, sizeof(_dma_unit_used));
    _dma_men_pool_used = 0;

    g_udev_list = USBNULL;

    (void)memset(_dev_addr_pool, 0, sizeof(_dev_addr_pool));
    _device_addr = 1;
}

uint32_t  usbh_memory_used(void)
{
    USB_debug("USB H/W memory: %d/%d, DMA memory: %d/%d\n", _hw_mem_pool_used, HW_MEM_UNIT_NUM,
              _dma_men_pool_used, DMA_MEM_UNIT_NUM);
    return _dma_men_pool_used;
}

void *usbh_alloc_mem(int size)
{
    int  i;
    int  start;
    int  found;
    int  wanted;
    void  *p;

    start = -1;
    found = 0;
    wanted = ((size + DMA_MEM_UNIT_SIZE - 1) / DMA_MEM_UNIT_SIZE);

    for (i = 0; i < (DMA_MEM_UNIT_NUM - wanted + 1); i++)
    {
        if (_dma_unit_used[i] == 0U)
        {
            if (found == 0)
            {
                start = i;
            }

            found++;

            if (found >= wanted)
            {
                break;
            }
        }
        else
        {
            found = 0;
        }
    }

    if (found < wanted)
    {
        USB_error("%s failed to allocate %d KB!!! (%d / %d)\n", __func__,
                  size / 1024, _dma_men_pool_used, DMA_MEM_UNIT_NUM);
        return USBNULL;
    }

    /* Go allocate it */
    for (i = start; i < (start + wanted); i++)
    {
        _dma_unit_used[i] = 1;
    }

    _dma_men_pool_used += wanted;


    (void)memset(&_dma_mem_pool[start], 0, DMA_MEM_UNIT_SIZE * wanted);
    p = (void *)&_dma_mem_pool[start];
    return p;
}

int usbh_free_mem(const void *p, int size)
{
    int i;
    int start;
    int wanted;
    uint32_t paddr;
    uint32_t base;

    paddr = (uint32_t)(p);
    base = (uint32_t)(&_dma_mem_pool[0]);

    if ((paddr < base) || (paddr > (base + (((uint32_t)DMA_MEM_UNIT_NUM - 1U) * ((uint32_t)DMA_MEM_UNIT_SIZE)))))
    {
        USB_error("%s - invalid DMA address 0x%x!\n", __func__, (uint32_t)paddr);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    start = (paddr - base) / ((uint32_t)DMA_MEM_UNIT_SIZE);

    if ((uint32_t)&_dma_mem_pool[start] != paddr)
    {
        USB_error("%s paddr not block aligned: 0x%x\n", __func__, (uint32_t)paddr);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    wanted = (size + DMA_MEM_UNIT_SIZE - 1) / DMA_MEM_UNIT_SIZE;

    if ((paddr + (((uint32_t)wanted) * (uint32_t)DMA_MEM_UNIT_SIZE)) > (base + ((uint32_t)DMA_MEM_UNIT_NUM * (uint32_t)DMA_MEM_UNIT_SIZE)))
    {
        USB_error("%s - invalid DMA address 0x%x, size %d!\n", __func__, (uint32_t)paddr, size);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    for (i = start; i < (start + wanted); i++)
    {
        if (!_dma_unit_used[i])
        {
            USB_error("%s warning - try to free an unused block %d!\n", __func__, i);
        }

        _dma_unit_used[i] = 0;
    }

    _dma_men_pool_used -= wanted;

    return USBH_OK;
}

/*--------------------------------------------------------------------------*/
/*   USB device allocate/free                                               */
/*--------------------------------------------------------------------------*/

UDEV_T *alloc_device(void)
{
    UDEV_T  *udev;

    udev = usbh_alloc_mem(sizeof(*udev));

    if (udev == USBNULL)
    {
        USB_error("alloc_device failed!\n");
        return USBNULL;
    }

    (void)memset(udev, 0, sizeof(*udev));
    udev->cur_conf = -1;                    /* must! used to identify the first SET CONFIGURATION */
    udev->next = g_udev_list;               /* chain to global device list */
    g_udev_list = udev;
    return udev;
}

void free_device(UDEV_T *udev)
{
    UDEV_T  *d;

    if (udev == USBNULL)
    {
        return;
    }

    if (udev->cfd_buff != USBNULL)
    {
        (void)usbh_free_mem(udev->cfd_buff, MAX_DESC_BUFF_SIZE);
    }

    /*
     *  Remove it from the global device list
     */
    if (g_udev_list == udev)
    {
        g_udev_list = g_udev_list->next;
    }
    else
    {
        d = g_udev_list;

        while (d != USBNULL)
        {
            if (d->next == udev)
            {
                d->next = udev->next;
                break;
            }

            d = d->next;
        }
    }

    (void)usbh_free_mem(udev, sizeof(*udev));
}

int  alloc_dev_address(void)
{
    _device_addr++;

    if (_device_addr >= 128)
    {
        _device_addr = 1;
    }

    while (1)
    {
        if (_dev_addr_pool[_device_addr] == 0U)
        {
            _dev_addr_pool[_device_addr] = 1;
            return _device_addr;
        }

        _device_addr++;

        if (_device_addr >= 128)
        {
            _device_addr = 1;
        }
    }
}

void  free_dev_address(int dev_addr)
{
    if (dev_addr < 128)
    {
        _dev_addr_pool[dev_addr] = 0;
    }
}

/*--------------------------------------------------------------------------*/
/*   UTR (USB Transfer Request) allocate/free                               */
/*--------------------------------------------------------------------------*/

UTR_T *alloc_utr(UDEV_T *udev)
{
    UTR_T  *utr;

    utr = usbh_alloc_mem(sizeof(*utr));

    if (utr == USBNULL)
    {
        USB_error("alloc_utr failed!\n");
        return USBNULL;
    }

    (void)memset(utr, 0, sizeof(*utr));
    utr->udev = udev;
    mem_debug("[ALLOC] [UTR] - 0x%x\n", (int)utr);
    return utr;
}

void free_utr(const UTR_T *utr)
{
    if (utr == USBNULL)
    {
        return;
    }

    mem_debug("[FREE] [UTR] - 0x%x\n", (int)utr);
    (void)usbh_free_mem(utr, sizeof(*utr));
}

/*--------------------------------------------------------------------------*/
/*   OHCI ED allocate/free                                                  */
/*--------------------------------------------------------------------------*/

ED_T *alloc_ohci_ED(void)
{
    int    i;
    ED_T   *ed;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if (_hw_unit_used[i] == 0U)
        {
            _hw_unit_used[i] = 1;
            _hw_mem_pool_used++;

            ed = (ED_T *)&_hw_mem_pool[i];
            (void)memset(ed, 0, sizeof(*ed));
            mem_debug("[ALLOC] [ED] - 0x%x\n", (int)ed);
            return ed;
        }
    }

    USB_error("alloc_ohci_ED failed!\n");
    return USBNULL;
}

void free_ohci_ED(const ED_T *ed)
{
    int      i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)ed)
        {
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            mem_debug("[FREE]  [ED] - 0x%x\n", (int)ed);
            return;
        }
    }

    USB_debug("free_ohci_ED - not found! (ignored in case of multiple UTR)\n");
}

/*--------------------------------------------------------------------------*/
/*   OHCI TD allocate/free                                                  */
/*--------------------------------------------------------------------------*/
TD_T *alloc_ohci_TD(UTR_T *utr)
{
    int    i;
    TD_T   *td;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if (_hw_unit_used[i] == 0U)
        {
            _hw_unit_used[i] = 1;
            _hw_mem_pool_used++;

            td = (TD_T *)&_hw_mem_pool[i];

            (void)memset(td, 0, sizeof(*td));
            td->utr = utr;
            mem_debug("[ALLOC] [TD] - 0x%x\n", (int)td);
            return td;
        }
    }

    USB_error("alloc_ohci_TD failed!\n");
    return USBNULL;
}

void free_ohci_TD(const TD_T *td)
{
    int   i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)td)
        {
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            mem_debug("[FREE]  [TD] - 0x%x\n", (int)td);
            return;
        }
    }

    USB_error("free_ohci_TD - not found!\n");
}
/// @endcond HIDDEN_SYMBOLS
