#include "sfud.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <c1> Enable SPIM DMA Read
// <i> Enable SPIM DMA Read
//#define ENABLE_SPIM_DMA_READ
// </c>
// *** <<< end of configuration section >>> ***
#pragma once

enum
{
    SFUD_WINBOND_DEV_IDX0 = 0,
    SFUD_WINBOND_DEV_IDX1,
    SFUD_WINBOND_DEV_IDX2,
		SFUD_WINBOND_DEV_IDX3,
		SFUD_WINBOND_DEV_IDX4,
};

#define SFUD_FLASH_DEVICE_TABLE                                             \
{                                                                           \
    [SFUD_WINBOND_DEV_IDX0] = {.name = "W25Q256JV0", .spi.name = "SPI0"},   \
    [SFUD_WINBOND_DEV_IDX1] = {.name = "W25Q256JV1", .spi.name = "QSPI0"},	\
		[SFUD_WINBOND_DEV_IDX2] = {.name = "W25Q256JV2", .spi.name = "USPI0"},	\
		[SFUD_WINBOND_DEV_IDX3] = {.name = "W25Q256JV3", .spi.name = "SPIM0"},	\
		[SFUD_WINBOND_DEV_IDX4] = {.name = "W25Q256JV4", .spi.name = "LPSPI0"},	\
}
