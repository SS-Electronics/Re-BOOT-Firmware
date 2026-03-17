/*
File:        drv_jump.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Driver
Info:        Application-jump driver abstraction — hardware-independent
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
 * @file drv_jump.c
 * @brief Hardware-independent application-jump driver.
 *
 * Forwards the bl_app_jump() call to the MCU-specific implementation
 * registered by the application via @ref bl_jump_driver_register().
 *
 * This module contains no MCU headers.  All CMSIS register access
 * (SCB, NVIC, SysTick, __set_MSP, __disable_irq) lives exclusively
 * in the board-support layer (main.c), keeping bootloader.c and this
 * driver fully portable.
 */

#include "drv_jump.h"

/* ====================================================================
   Module-level state
   ==================================================================== */

/** Registered MCU-specific application-jump implementation */
static app_jump_fn_t jump_fn = NULL;


/* ====================================================================
   Driver registration
   ==================================================================== */

/**
 * @brief Register the MCU-specific application-jump implementation.
 *
 * @param fn  Pointer to the MCU-specific jump function.
 */
void bl_jump_driver_register(app_jump_fn_t fn)
{
    jump_fn = fn;
}


/* ====================================================================
   Public API
   ==================================================================== */

/**
 * @brief Transfer execution to the application.
 *
 * Calls the registered jump implementation.  Never returns if a valid
 * implementation has been registered.
 *
 * Safe no-op if @ref bl_jump_driver_register() was not called — the
 * bootloader simply continues its loop rather than branching to an
 * unknown address.
 */
void bl_app_jump(void)
{
    if (jump_fn != NULL)
    {
        jump_fn();
    }
}
