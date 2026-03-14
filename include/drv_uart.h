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
 * @author
 * @brief Generic UART driver abstraction interface
 * @version 1.0
 * @date 2026
 *
 * @details
 * This module provides a hardware independent UART abstraction layer
 * used by upper firmware layers such as:
 *
 * - Transport layer
 * - Bootloader protocol
 * - Command parser
 *
 * The actual UART peripheral implementation is provided by the
 * application (typically inside `main.c` or an MCU HAL driver).
 *
 * The application must populate a @ref uart_ops_t structure with
 * hardware specific functions and register it using
 * @ref bl_uart_driver_register().
 *
 * This design allows the firmware to remain **portable across MCUs**
 * while keeping hardware access separated from protocol logic.
 *
 * Typical workflow:
 *
 * @code
 * static int32_t mcu_uart_tx(const uint8_t *data, uint16_t len);
 * static int32_t mcu_uart_rx(uint8_t *data, uint16_t len);
 *
 * uart_ops_t uart_ops =
 * {
 *     .init     = mcu_uart_init,
 *     .close    = mcu_uart_close,
 *     .transmit = mcu_uart_tx,
 *     .receive  = mcu_uart_rx
 * };
 *
 * int main(void)
 * {
 *     bl_uart_driver_register(&uart_ops);
 *
 *     uart_init();
 *
 *     uart_transmit(buffer, length);
 * }
 * @endcode
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"
#include "reboot_config.h"

/**
 * @defgroup UART_DRIVER UART Driver Abstraction
 * @brief Hardware independent UART interface
 *
 * This module defines the UART abstraction used by the transport layer
 * and bootloader communication stack.
 *
 * The actual hardware driver must implement the functions defined in
 * @ref uart_ops_t and register them using @ref bl_uart_driver_register().
 *
 * @{
 */

#if (BOOT_INTERFACE == BL_INTERFACE_UART)
/**
 * @brief UART data source type
 *
 * Defines the origin of received UART data.
 */
typedef enum
{
    UART_PERIPHERAL = 0,   /**< Data received directly from UART peripheral */
    UART_BUFFER           /**< Data retrieved from internal driver buffer */
} uart_data_src_t;


/**
 * @brief UART RX byte callback prototype
 *
 * Callback used to notify upper layers when a single byte is received.
 *
 * @param byte Received byte
 */
typedef void (*drv_rx_byte_cb_t)(uint8_t byte);


/**
 * @brief UART RX data callback prototype
 *
 * Callback used when a block of data is received.
 *
 * @param data Pointer to received data buffer
 * @param len  Number of received bytes
 */
typedef void (*drv_rx_data_cb_t)(uint8_t *data, uint16_t len);


/**
 * @brief UART operations abstraction structure
 *
 * This structure contains function pointers to the
 * hardware-specific UART implementation.
 *
 * The application must populate this structure and register it
 * using @ref bl_uart_driver_register().
 *
 * @note
 * All functions must be implemented by the platform layer.
 */
typedef struct
{
    /**
     * @brief Initialize UART hardware
     *
     * Configures the UART peripheral and prepares it
     * for communication.
     */
    void (*init)(void);


    /**
     * @brief Deinitialize UART hardware
     *
     * Releases UART resources and disables the peripheral.
     */
    void (*close)(void);


    /**
     * @brief Transmit data over UART
     *
     * @param data Pointer to transmit buffer
     * @param len  Number of bytes to transmit
     *
     * @return
     *  - Number of bytes transmitted
     *  - Negative value on error
     */
    int32_t (*transmit)(const uint8_t *data,
                        uint16_t len);


    /**
     * @brief Receive data from UART
     *
     * Reads data from the UART peripheral or internal driver buffer.
     *
     * @param data   Pointer to receive buffer
     * @param maxlen Maximum number of bytes to read
     *
     * @return
     *  - Number of bytes received
     *  - Negative value on error
     */
    int32_t (*receive)(uint8_t *data,
                       uint16_t maxlen);

} uart_ops_t;


/**
 * @brief Register UART driver implementation
 *
 * Registers the hardware specific UART operations with the
 * abstraction layer.
 *
 * @param ops Pointer to UART operations structure
 *
 * @note
 * This function must be called before using any UART API.
 */
void bl_uart_driver_register(const uart_ops_t *ops);


/**
 * @brief Initialize UART driver
 *
 * Calls the registered hardware initialization function.
 *
 * @return
 *  - 0 on success
 *  - Negative value on failure
 */
int32_t uart_init(void);


/**
 * @brief Close UART driver
 *
 * Calls the registered hardware close function.
 */
void uart_close(void);


/**
 * @brief Transmit data using UART
 *
 * Sends raw data through the registered UART driver.
 *
 * @param data Pointer to transmit buffer
 * @param len  Number of bytes to transmit
 *
 * @return
 *  - Number of bytes transmitted
 *  - Negative value on error
 */
int uart_transmit(const uint8_t *data,
                  uint16_t len);


/**
 * @brief Receive data using UART
 *
 * Reads raw data from the UART driver.
 *
 * @param data   Pointer to receive buffer
 * @param maxlen Maximum number of bytes to receive
 *
 * @return
 *  - Number of bytes received
 *  - Negative value on error
 */
int uart_receive(uint8_t *data,
                 uint16_t maxlen,
                 uart_data_src_t source);


/**
 * @brief UART RX byte handler
 *
 * This function is typically called from a UART RX interrupt
 * service routine (ISR). It stores the received byte into the
 * internal UART ring buffer.
 *
 * @param byte Received byte
 */
void drv_cb_uart_rx_byte(uint8_t byte);


/**
 * @brief Get number of bytes available in UART RX buffer
 *
 * @return Number of unread bytes in the UART buffer
 */
uint16_t drv_cb_uart_bytes_available(void);


/** @} */ /* end of UART_DRIVER group */

#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_UART_H__ */
