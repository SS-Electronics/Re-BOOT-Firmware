/*
File:        drv_delay.c
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
 * @file drv_delay.c
 * @brief Hardware-independent millisecond delay driver.
 *
 * Forwards every delay_ms() call to the MCU-specific implementation
 * registered by the application via @ref bl_delay_driver_register().
 *
 * The bootloader core has zero direct HAL dependencies for timing:
 * only this thin forwarding layer touches the HAL, and only
 * indirectly through the function pointer stored here.
 */

#include "drv_delay.h"

/* ====================================================================
   Module-level state
   ==================================================================== */

/** Registered hardware-specific delay implementation */
static delay_ms_fn_t delay_fn = NULL;


/* ====================================================================
   Driver registration
   ==================================================================== */

/**
 * @brief Register the hardware ms-delay implementation.
 *
 * @param fn  Pointer to a blocking millisecond delay function.
 *            On STM32 pass HAL_Delay directly.
 */
void bl_delay_driver_register(delay_ms_fn_t fn)
{
    delay_fn = fn;
}


/* ====================================================================
   Public API
   ==================================================================== */

/**
 * @brief Block for @p ms milliseconds.
 *
 * Calls the registered implementation.  Returns immediately if no
 * implementation has been registered (safe no-op).
 *
 * @param ms  Number of milliseconds to wait.
 */
void delay_ms(uint32_t ms)
{
    if (delay_fn != NULL)
    {
        delay_fn(ms);
    }
}
