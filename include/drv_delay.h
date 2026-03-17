/*
File:        drv_delay.h
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Driver
Info:        Delay driver abstraction — hardware-independent ms delay
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
 * @file drv_delay.h
 * @brief Hardware-independent millisecond delay driver abstraction.
 *
 * The bootloader core never calls HAL_Delay() directly.  Instead it
 * calls delay_ms() defined here, which forwards to the MCU-specific
 * implementation registered by the application via
 * @ref bl_delay_driver_register().
 *
 * @par Typical usage (in main.c)
 * @code
 * // HAL_Delay already matches the signature — pass it directly
 * bl_delay_driver_register(HAL_Delay);
 * bl_flash_driver_register(&flash_ops);
 * bl_uart_driver_register(&uart_ops);
 * bootloader_exe();
 * @endcode
 */

#ifndef __INCLUDE_DRV_DELAY_H__
#define __INCLUDE_DRV_DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"

/**
 * @brief Function pointer type for a millisecond delay implementation.
 *
 * The registered function must block for at least @p ms milliseconds
 * before returning.  HAL_Delay() on STM32 satisfies this contract.
 *
 * @param ms  Number of milliseconds to wait.
 */
typedef void (*delay_ms_fn_t)(uint32_t ms);

/**
 * @brief Register the hardware delay implementation.
 *
 * Must be called once before any delay_ms() call.
 * On STM32 pass HAL_Delay directly:
 * @code
 *   bl_delay_driver_register(HAL_Delay);
 * @endcode
 *
 * @param fn  Pointer to an ms-resolution blocking delay function.
 */
void bl_delay_driver_register(delay_ms_fn_t fn);

/**
 * @brief Block for @p ms milliseconds.
 *
 * Forwards to the registered implementation.  If no implementation has
 * been registered the call returns immediately (no-op).
 *
 * @param ms  Number of milliseconds to wait.
 */
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_DELAY_H__ */
