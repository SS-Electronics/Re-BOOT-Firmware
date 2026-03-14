/*
File:        drv_uart.h
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Driver
Info:        UART Driver Abstraction
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

#ifndef __INCLUDE_DRV_UART_H__
#define __INCLUDE_DRV_UART_H__


/**
 * @file drv_uart.h
 * @brief Generic UART driver abstraction interface
 *
 * This module provides a hardware independent UART interface used by
 * higher layers such as the transport layer or bootloader command parser.
 *
 * The actual UART hardware implementation is provided by the application
 * (typically in main.c or a HAL driver) and registered using
 * uart_driver_register().
 *
 * This allows the firmware to remain independent from the underlying
 * MCU UART implementation.
 *
 * Example usage:
 *
 * @code
 * uart_ops_t ops =
 * {
 *     .init     = mcu_uart_init,
 *     .close    = mcu_uart_close,
 *     .transmit = mcu_uart_tx,
 *     .receive  = mcu_uart_rx
 * };
 *
 * uart_driver_register(&ops);
 *
 * uart_init();
 *
 * uart_transmit(data, len);
 * @endcode
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"

/**
 * @defgroup UART_DRIVER UART Driver Abstraction
 * @brief Hardware independent UART driver interface
 *
 * This module defines the UART abstraction used by the transport layer.
 * The real UART hardware driver must implement the operations defined
 * in uart_ops_t and register them using uart_driver_register().
 *
 * @{
 */


/**
 * @brief UART operations abstraction
 *
 * This structure contains function pointers to the
 * hardware-specific UART implementation.
 *
 * The application must populate this structure and
 * register it using uart_driver_register().
 */
typedef struct
{
    /**
     * @brief Initialize UART hardware
     *
     * This function initializes the UART peripheral and prepares
     * it for communication.
     */
    void (*init)(void);

    /**
     * @brief Close UART driver
     *
     * This function deinitializes the UART peripheral and
     * releases any allocated resources.
     */
    void (*close)(void);

    /**
     * @brief Transmit data over UART
     *
     * @param data Pointer to transmit buffer
     * @param len  Number of bytes to transmit
     *
     * @return Number of bytes transmitted
     */
    int32_t (*transmit)(const uint8_t *data,
                        uint16_t len);

    /**
     * @brief Receive data from UART
     *
     * @param data Pointer to receive buffer
     * @param maxlen Maximum number of bytes to read
     *
     * @return Number of bytes received
     */
    int32_t (*receive)(uint8_t *data,
                       uint16_t maxlen);

} uart_ops_t;


/**
 * @brief Register UART driver operations
 *
 * This function registers the UART hardware implementation
 * with the abstraction layer.
 *
 * The provided operations will be used by the UART wrapper APIs.
 *
 * @param ops Pointer to UART operations structure
 *
 * @return None
 */
void uart_driver_register(const uart_ops_t *ops);


/**
 * @brief Initialize UART driver
 *
 * Calls the registered UART init function.
 *
 * @return
 *  - 0 : Success
 *  - negative value : Error
 */
int32_t uart_init(void);


/**
 * @brief Close UART driver
 *
 * Calls the registered UART close function.
 */
void uart_close(void);


/**
 * @brief Transmit data using UART
 *
 * Sends raw data through the registered UART driver.
 *
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to transmit
 *
 * @return
 *  - Number of bytes transmitted
 *  - negative value on error
 */
int uart_transmit(const uint8_t *data,
                  uint16_t len);


/**
 * @brief Receive data using UART
 *
 * Reads data from the registered UART driver.
 *
 * @param data Pointer to receive buffer
 * @param maxlen Maximum bytes to receive
 *
 * @return
 *  - Number of bytes received
 *  - negative value on error
 */
int uart_receive(uint8_t *data,
                 uint16_t maxlen);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_UART_H__ */
