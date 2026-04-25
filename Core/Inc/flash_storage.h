#ifndef FLASH_STORAGE_H

#define FLASH_STORAGE_H



#include "main.h"

#include <string.h>



/*

 * Flash Storage for STM32F767ZI

 *

 * Uses Sector 7 (last 256KB) at 0x080C0000

 * Program code lives in Sectors 0-6, so this is safe.

 *

 * IMPORTANT:

 *   - Flash erase takes 1-4 seconds — CPU stalls, no control loop!

 *   - Only call Flash_Write when robot is STOPPED

 *   - Flash has ~10,000 erase cycles before wear-out

 */



#define FLASH_STORAGE_SECTOR    FLASH_SECTOR_7

#define FLASH_STORAGE_ADDR      0x080C0000UL

#define FLASH_STORAGE_SIZE      (256 * 1024)  // 256KB



// Magic number to check if flash has valid data

#define FLASH_MAGIC             0xDEADBEEF



// Header stored at the start of the flash sector

typedef struct {

    uint32_t magic;        // FLASH_MAGIC if valid

    uint32_t num_samples;  // how many log entries

    float kp;              // gains at time of save

    float kd;

    float ki;

    uint16_t pot_center;   // calibration value

    uint16_t reserved;

} FlashHeader;



/*

 * Write log data to flash.

 * Call ONLY when robot is stopped (motors off).

 * Takes 1-4 seconds. CPU stalls during erase.

 *

 * data:       pointer to float array (log_theta)

 * num_floats: number of floats to store

 * kp,kd,ki:   current gains

 * pot_center: calibration value

 */

static inline HAL_StatusTypeDef Flash_WriteLog(

    volatile float *data, uint32_t num_floats,

    float kp, float kd, float ki, uint16_t pot_center)

{

    HAL_StatusTypeDef status;



    // 1. Unlock flash

    HAL_FLASH_Unlock();



    // 2. Erase sector 7 (this takes 1-4 seconds!)

    FLASH_EraseInitTypeDef erase;

    uint32_t error;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;

    erase.Sector = FLASH_STORAGE_SECTOR;

    erase.NbSectors = 1;

    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7-3.6V



    status = HAL_FLASHEx_Erase(&erase, &error);

    if (status != HAL_OK) {

        HAL_FLASH_Lock();

        return status;

    }



    // 3. Write header

    FlashHeader header;

    header.magic = FLASH_MAGIC;

    header.num_samples = num_floats;

    header.kp = kp;

    header.kd = kd;

    header.ki = ki;

    header.pot_center = pot_center;

    header.reserved = 0;



    uint32_t addr = FLASH_STORAGE_ADDR;

    uint32_t *hdr_ptr = (uint32_t *)&header;

    for (uint32_t i = 0; i < sizeof(FlashHeader) / 4; i++) {

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, hdr_ptr[i]);

        if (status != HAL_OK) {

            HAL_FLASH_Lock();

            return status;

        }

        addr += 4;

    }



    // 4. Write data (float array)

    for (uint32_t i = 0; i < num_floats; i++) {

        uint32_t word;

        memcpy(&word, (void *)&data[i], 4);  // float → uint32 without cast issues

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);

        if (status != HAL_OK) {

            HAL_FLASH_Lock();

            return status;

        }

        addr += 4;

    }



    // 5. Lock flash

    HAL_FLASH_Lock();

    return HAL_OK;

}



/*

 * Check if flash has valid data

 */

static inline uint8_t Flash_HasData(void)

{

    FlashHeader *header = (FlashHeader *)FLASH_STORAGE_ADDR;

    return (header->magic == FLASH_MAGIC) ? 1 : 0;

}



/*

 * Read header from flash

 */

static inline FlashHeader Flash_ReadHeader(void)

{

    return *(FlashHeader *)FLASH_STORAGE_ADDR;

}



/*

 * Read log data from flash into a buffer

 * Returns number of floats read

 */

static inline uint32_t Flash_ReadLog(float *dest, uint32_t max_floats)

{

    if (!Flash_HasData()) return 0;



    FlashHeader *header = (FlashHeader *)FLASH_STORAGE_ADDR;

    uint32_t count = header->num_samples;

    if (count > max_floats) count = max_floats;



    float *src = (float *)(FLASH_STORAGE_ADDR + sizeof(FlashHeader));

    memcpy(dest, src, count * sizeof(float));



    return count;

}



#endif /* FLASH_STORAGE_H */
