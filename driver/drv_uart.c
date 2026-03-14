/*
File:        drv_uart.c
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

/**
 * @file drv_uart.c
 * @brief UART driver abstraction implementation
 *
 * This module implements the UART abstraction wrapper that forwards
 * API calls to the hardware-specific UART implementation registered
 * by the application.
 */

#include "drv_uart.h"

#if (BOOT_INTERFACE == BL_INTERFACE_UART)

/**
 * @brief Pointer to registered UART operations
 */
static const uart_ops_t *uart_operations = NULL;


/**
 * @brief Register UART operations
 *
 * Stores the UART hardware implementation provided by the application.
 *
 * @param ops Pointer to UART operations structure
 */
void uart_driver_register(const uart_ops_t *ops)
{
    uart_operations = ops;
}


/**
 * @brief Initialize UART
 *
 * Calls the hardware-specific initialization routine.
 *
 * @return
 *  - 0 on success
 *  - -1 if driver not registered
 */
int32_t uart_init(void)
{
    if ((uart_operations == NULL) || (uart_operations->init == NULL))
    {
        return -1;
    }

    uart_operations->init();

    return 0;
}


/**
 * @brief Close UART driver
 *
 * Calls the hardware-specific close routine.
 */
void uart_close(void)
{
    if ((uart_operations != NULL) && (uart_operations->close != NULL))
    {
        uart_operations->close();
    }
}


/**
 * @brief Transmit UART data
 *
 * Sends raw data through the registered UART driver.
 *
 * @param data Pointer to transmit buffer
 * @param len Number of bytes to transmit
 *
 * @return
 *  - Number of bytes transmitted
 *  - -1 if driver not registered
 */
int uart_transmit(const uint8_t *data, uint16_t len)
{
    if ((uart_operations == NULL) || (uart_operations->transmit == NULL))
    {
        return -1;
    }

    return uart_operations->transmit(data, len);
}


/**
 * @brief Receive UART data
 *
 * Reads data from the registered UART driver.
 *
 * @param data Pointer to receive buffer
 * @param maxlen Maximum number of bytes to receive
 *
 * @return
 *  - Number of bytes received
 *  - -1 if driver not registered
 */
int uart_receive(uint8_t *data, uint16_t maxlen)
{
    if ((uart_operations == NULL) || (uart_operations->receive == NULL))
    {
        return -1;
    }

    return uart_operations->receive(data, maxlen);
}

#endif
