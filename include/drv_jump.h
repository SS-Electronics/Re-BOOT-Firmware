/*
File:        drv_jump.h
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
 * @file drv_jump.h
 * @brief Hardware-independent application-jump driver abstraction.
 *
 * The bootloader core never touches CMSIS registers (SCB, NVIC, SysTick)
 * or compiler intrinsics (__set_MSP, __disable_irq) directly.  Instead
 * it calls bl_app_jump() defined here, which forwards to the MCU-specific
 * implementation registered by the application via
 * @ref bl_jump_driver_register().
 *
 * This removes the last MCU-specific header (stm32f4xx.h) from
 * bootloader.c, making the bootloader core fully portable.
 *
 * @par Typical usage (in main.c)
 * @code
 * // 1. Implement the MCU-specific jump sequence
 * static void mcu_jump_to_application(void)
 * {
 *     __disable_irq();
 *     SysTick->CTRL = 0;
 *     // ... clear NVIC, set VTOR, load MSP, branch
 * }
 *
 * // 2. Register before entering the bootloader
 * int main(void)
 * {
 *     bl_jump_driver_register(mcu_jump_to_application);
 *     bl_flash_driver_register(&flash_ops);
 *     bl_uart_driver_register(&uart_ops);
 *     bl_delay_driver_register(HAL_Delay);
 *     bootloader_exe();
 * }
 * @endcode
 */

#ifndef __INCLUDE_DRV_JUMP_H__
#define __INCLUDE_DRV_JUMP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"

/**
 * @brief Function pointer type for the MCU-specific application-jump routine.
 *
 * The registered function must:
 *  - Disable all interrupts.
 *  - Stop and clear SysTick.
 *  - Clear NVIC enable and pending registers.
 *  - Relocate the vector table (SCB->VTOR) to APP_START_ADDRESS.
 *  - Load the application's initial stack pointer.
 *  - Re-enable interrupts.
 *  - Branch to the application's Reset_Handler.
 *  - Never return.
 */
typedef void (*app_jump_fn_t)(void);

/**
 * @brief Register the MCU-specific application-jump implementation.
 *
 * Must be called once before bootloader_exe().
 *
 * @param fn  Pointer to the MCU-specific jump function.
 */
void bl_jump_driver_register(app_jump_fn_t fn);

/**
 * @brief Transfer execution to the application.
 *
 * Calls the registered MCU-specific jump implementation.
 * This function never returns under normal operation.
 *
 * If no implementation has been registered the call is a no-op
 * (safe fallback — bootloader stays in its loop).
 */
void bl_app_jump(void);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_JUMP_H__ */
