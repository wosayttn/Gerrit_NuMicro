
#ifndef EEPROM_EMU_H
#define EEPROM_EMU_H

#include <stdint.h>

typedef uint32_t flash_status_t;

/* Status */
#define FLASH_STATUS_OK              (0x00000000U)
#define FLASH_STATUS_INVALID_PARAM   (0x00000001U)
#define FLASH_STATUS_OUT_OF_RANGE    (0x00000002U)
#define FLASH_STATUS_HW_ERROR        (0x00000003U)
#define APROM_BASE_ADDR              (0x00000000U)
#define APROM_MAX_SIZE               (FMC_APROM_SIZE - APROM_BASE_ADDR)
#define APROM_END_ADDR               (APROM_BASE_ADDR + APROM_MAX_SIZE - 1U)
#define FLASH_WORD_SIZE              (4U)

/* API */
flash_status_t EEPROM_Emu_Init(void);
flash_status_t EEPROM_Emu_DeInit(void);
flash_status_t EEPROM_Emu_ReadByte(uint32_t addr, uint8_t *pData);
flash_status_t EEPROM_Emu_WriteByte(uint32_t addr, uint8_t data);
flash_status_t EEPROM_Emu_ReadArray(uint32_t addr, uint8_t *pData, uint32_t len);
flash_status_t EEPROM_Emu_WriteArray(uint32_t addr, const uint8_t *pData, uint32_t len);
flash_status_t EEPROM_Emu_Erase(uint32_t addr, uint32_t len);

#endif
