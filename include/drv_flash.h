/*
File:        drv_flash.h
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      include
Info:        Flash driver abstraction — erase / write / read API and registration
Dependency:  None

This file is part of Re-BOOT Firmware Project.

Re-BOOT Firmware is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version
3 of the License, or (at your option) any later version.

Re-BOOT Firmware is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Re-BOOT Firmware. If not, see <https://www.gnu.org/licenses/>.
*/

/**
 * @file flash_drv.h
 * @brief Hardware-independent internal flash driver abstraction.
 *
 * This module provides a portable flash interface used by the bootloader
 * to erase and program application firmware into internal MCU flash.
 *
 * The application (board support layer) must implement the three
 * hardware-specific operations and register them once at startup via
 * @ref bl_flash_driver_register() before calling @ref bootloader_exe().
 *
 * @par Typical usage
 * @code
 * // 1.  Implement MCU-specific flash operations
 * static int32_t mcu_flash_erase(uint32_t address, uint32_t size);
 * static int32_t mcu_flash_write(uint32_t address,
 *                                const uint8_t *data, uint32_t size);
 * static int32_t mcu_flash_read (uint32_t address,
 *                                uint8_t *buf,   uint32_t size);
 *
 * // 2.  Populate the ops struct
 * static const flash_ops_t flash_ops =
 * {
 *     .erase = mcu_flash_erase,
 *     .write = mcu_flash_write,
 *     .read  = mcu_flash_read,
 * };
 *
 * // 3.  Register before entering the bootloader
 * int main(void)
 * {
 *     bl_flash_driver_register(&flash_ops);
 *     bootloader_exe();          // never returns
 * }
 * @endcode
 *
 * @par Return-value convention
 * All flash operations return:
 *   -  0  on success
 *   - -1  on any hardware error (timeout, ECC fault, etc.)
 */

#ifndef __INCLUDE_FLASH_DRV_H__
#define __INCLUDE_FLASH_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"

/**
 * @defgroup FLASH_DRIVER Flash Driver Abstraction
 * @brief Hardware-independent flash erase / write / read interface.
 * @{
 */

/* ====================================================================
   Flash operations structure
   ==================================================================== */

/**
 * @brief Flash hardware operation callbacks.
 *
 * The application fills this structure with MCU-specific function
 * pointers and passes it to @ref bl_flash_driver_register().
 */
typedef struct
{
    /**
     * @brief Erase one or more flash sectors.
     *
     * Must erase the smallest flash region that fully contains the
     * range [ @p address, @p address + @p size ).  On most MCUs this
     * means erasing the sector(s) that overlap the given range.
     *
     * After a successful erase every byte in the range reads as 0xFF.
     *
     * @param address  Starting address of the region to erase.
     *                 Must be sector-aligned on the target MCU.
     * @param size     Number of bytes to erase.
     *                 Must be a multiple of the sector size.
     *
     * @return  0  success — region reads as 0xFF.
     * @return -1  hardware error (e.g. write-protect, timeout).
     */
    int32_t (*erase)(uint32_t address, uint32_t size);

    /**
     * @brief Program bytes into previously erased flash.
     *
     * @p address and @p size must meet any alignment requirements of
     * the target MCU (e.g. word-aligned writes on Cortex-M).  The
     * caller (bootloader) guarantees the target range has been erased.
     *
     * @param address  Destination flash address.
     * @param data     Pointer to source data buffer.
     * @param size     Number of bytes to write.
     *
     * @return  0  success.
     * @return -1  hardware error.
     */
    int32_t (*write)(uint32_t address, const uint8_t *data, uint32_t size);

    /**
     * @brief Read bytes from flash.
     *
     * Used after programming to verify written data.
     *
     * @param address  Source flash address.
     * @param buf      Destination buffer.
     * @param size     Number of bytes to read.
     *
     * @return  0  success.
     * @return -1  hardware error.
     */
    int32_t (*read)(uint32_t address, uint8_t *buf, uint32_t size);

} flash_ops_t;


/* ====================================================================
   Public API
   ==================================================================== */

/**
 * @brief Register the hardware flash driver.
 *
 * Must be called exactly once before any flash operation.
 * The @p ops struct must remain valid (static storage) for the
 * lifetime of the bootloader.
 *
 * @param ops  Pointer to the populated flash operations struct.
 */
void bl_flash_driver_register(const flash_ops_t *ops);

/**
 * @brief Erase a flash region.
 *
 * Forwards to the registered @c flash_ops_t::erase callback.
 *
 * @param address  Sector-aligned start address.
 * @param size     Number of bytes (multiple of sector size).
 *
 * @return  0  success.
 * @return -1  driver not registered, or hardware error.
 */
int32_t flash_erase(uint32_t address, uint32_t size);

/**
 * @brief Write data into flash.
 *
 * The target region must have been erased first.
 * Forwards to the registered @c flash_ops_t::write callback.
 *
 * @param address  Destination flash address.
 * @param data     Pointer to source data.
 * @param size     Number of bytes to write.
 *
 * @return  0  success.
 * @return -1  driver not registered, or hardware error.
 */
int32_t flash_write(uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from flash.
 *
 * Forwards to the registered @c flash_ops_t::read callback.
 *
 * @param address  Source flash address.
 * @param buf      Destination buffer.
 * @param size     Number of bytes to read.
 *
 * @return  0  success.
 * @return -1  driver not registered, or hardware error.
 */
int32_t flash_read(uint32_t address, uint8_t *buf, uint32_t size);

/** @} */ /* end of FLASH_DRIVER group */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_FLASH_DRV_H__ */
