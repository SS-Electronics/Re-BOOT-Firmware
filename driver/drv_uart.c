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

/* Registered UART hardware implementation */
static const uart_ops_t *uart_operations = NULL;

/* RX ring buffer */
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;


/**
 * @brief Register UART driver
 */
void bl_uart_driver_register(const uart_ops_t *ops)
{
    uart_operations = ops;
}


/**
 * @brief Initialize UART
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
 * @brief Close UART
 */
void uart_close(void)
{
    if ((uart_operations != NULL) && (uart_operations->close != NULL))
    {
        uart_operations->close();
    }
}


/**
 * @brief UART transmit
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
 * @brief UART receive
 */
int uart_receive(uint8_t *data,
                 uint16_t maxlen,
                 uart_data_src_t source)
{
    uint16_t count = 0;

    if (data == NULL)
    {
        return -1;
    }

    switch(source)
    {
        case UART_PERIPHERAL:

            if ((uart_operations == NULL) || (uart_operations->receive == NULL))
            {
                return -1;
            }

            return uart_operations->receive(data, maxlen);


        case UART_BUFFER:

            while ((uart_rx_tail != uart_rx_head) && (count < maxlen))
            {
                data[count++] = uart_rx_buffer[uart_rx_tail];

                uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
            }

            return count;


        default:
            return -1;
    }
}


/**
 * @brief UART RX byte callback
 *
 * Called from UART ISR.
 */
void drv_cb_uart_rx_byte(uint8_t byte)
{
    uint16_t next = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;

    /* Prevent overflow */
    if (next != uart_rx_tail)
    {
        uart_rx_buffer[uart_rx_head] = byte;
        uart_rx_head = next;
    }
}


/**
 * @brief Bytes available in RX buffer
 */
uint16_t drv_cb_uart_bytes_available(void)
{
    if (uart_rx_head >= uart_rx_tail)
    {
        return uart_rx_head - uart_rx_tail;
    }

    return UART_RX_BUFFER_SIZE - uart_rx_tail + uart_rx_head;
}

#endif
