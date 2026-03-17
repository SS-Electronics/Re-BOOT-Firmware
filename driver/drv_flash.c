/*
File:        drv_flash.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      driver
Info:        Flash driver abstraction — erase / write / read forwarding layer
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
 * @file flash_drv.c
 * @brief Hardware-independent flash driver abstraction.
 *
 * Forwards every flash operation (erase / write / read) to the
 * MCU-specific implementation registered by the application via
 * @ref bl_flash_driver_register().
 *
 * @par Design
 * The bootloader never calls MCU HAL functions directly.  Instead it
 * calls flash_erase(), flash_write(), and flash_read() defined here.
 * This keeps the bootloader core fully portable — only the thin board-
 * support layer needs to change when retargeting to a new MCU.
 *
 * @par Registration sequence (in main / board init)
 * @code
 * static const flash_ops_t flash_ops =
 * {
 *     .erase = stm32_flash_erase,
 *     .write = stm32_flash_write,
 *     .read  = stm32_flash_read,
 * };
 *
 * bl_flash_driver_register(&flash_ops);
 * bootloader_exe();
 * @endcode
 */

#include "drv_flash.h"

/* ====================================================================
   Module-level state
   ==================================================================== */

/** Registered hardware-specific flash implementation */
static const flash_ops_t *flash_operations = NULL;


/* ====================================================================
   Driver registration
   ==================================================================== */

/**
 * @brief Register the hardware flash driver.
 *
 * Must be called once before any flash operation.  Safe to call from
 * main() before the RTOS scheduler starts or before bootloader_exe().
 *
 * @param ops  Pointer to a statically allocated @ref flash_ops_t
 *             populated with MCU-specific function pointers.
 */
void bl_flash_driver_register(const flash_ops_t *ops)
{
    flash_operations = ops;
}


/* ====================================================================
   Public API — thin forwarding wrappers
   ==================================================================== */

/**
 * @brief Erase a flash region.
 *
 * Erases the region [ @p address, @p address + @p size ) by calling
 * the registered @c flash_ops_t::erase callback.
 *
 * The caller must ensure:
 *  - @p address is sector-aligned for the target MCU.
 *  - @p size is a non-zero multiple of the sector size.
 *
 * @param address  Sector-aligned start address.
 * @param size     Number of bytes to erase (multiple of sector size).
 *
 * @return  0  success — region now reads as 0xFF.
 * @return -1  driver not registered, erase pointer NULL, or
 *             hardware error reported by the MCU implementation.
 */
int32_t flash_erase(uint32_t address, uint32_t size)
{
    if ((flash_operations == NULL) || (flash_operations->erase == NULL))
        return -1;

    return flash_operations->erase(address, size);
}


/**
 * @brief Write data into a previously erased flash region.
 *
 * Programs @p size bytes from @p data into flash starting at
 * @p address by calling the registered @c flash_ops_t::write callback.
 *
 * The target region must have been erased (reads 0xFF) before calling
 * this function.  Writing to non-erased flash produces undefined data
 * on most MCUs.
 *
 * @param address  Destination flash address.
 * @param data     Pointer to the source data buffer.
 * @param size     Number of bytes to write.
 *
 * @return  0  success.
 * @return -1  driver not registered, write pointer NULL, @p data is
 *             NULL, @p size is zero, or hardware error.
 */
int32_t flash_write(uint32_t address, const uint8_t *data, uint32_t size)
{
    if ((flash_operations == NULL) || (flash_operations->write == NULL))
        return -1;

    if ((data == NULL) || (size == 0u))
        return -1;

    return flash_operations->write(address, data, size);
}


/**
 * @brief Read data from flash.
 *
 * Reads @p size bytes from flash starting at @p address into @p buf
 * by calling the registered @c flash_ops_t::read callback.
 *
 * Used by the bootloader after programming to verify the written
 * sector matches the received data.
 *
 * @param address  Source flash address.
 * @param buf      Pointer to the destination buffer.
 * @param size     Number of bytes to read.
 *
 * @return  0  success.
 * @return -1  driver not registered, read pointer NULL, @p buf is
 *             NULL, @p size is zero, or hardware error.
 */
int32_t flash_read(uint32_t address, uint8_t *buf, uint32_t size)
{
    if ((flash_operations == NULL) || (flash_operations->read == NULL))
        return -1;

    if ((buf == NULL) || (size == 0u))
        return -1;

    return flash_operations->read(address, buf, size);
}

