/*
File:        drv_can.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Driver
Info:        CAN Driver Abstraction
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
 * @file drv_can.c
 * @brief CAN driver abstraction with RX buffer
 *
 * This module forwards CAN API calls to the hardware-specific
 * implementation and provides a software RX buffer.
 */

#include "drv_can.h"

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)

/* Registered CAN hardware implementation */
static const can_ops_t *can_operations = NULL;

/* RX ring buffer */
static can_frame_t can_rx_buffer[CAN_RX_BUFFER_SIZE];

static volatile uint16_t can_rx_head = 0;
static volatile uint16_t can_rx_tail = 0;


/**
 * @brief Register CAN driver
 */
void bl_can_driver_register(const can_ops_t *ops)
{
    can_operations = ops;
}


/**
 * @brief Initialize CAN driver
 */
int32_t can_init(void)
{
    if ((can_operations == NULL) || (can_operations->init == NULL))
    {
        return -1;
    }

    can_operations->init();

    return 0;
}


/**
 * @brief Close CAN driver
 */
void can_close(void)
{
    if ((can_operations != NULL) && (can_operations->close != NULL))
    {
        can_operations->close();
    }
}


/**
 * @brief CAN transmit
 */
int can_transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    if ((can_operations == NULL) || (can_operations->transmit == NULL))
    {
        return -1;
    }

    return can_operations->transmit(id, data, len);
}


/**
 * @brief CAN receive
 *
 * @param frame      Pointer to store received frame
 * @param source     UART_PERIPHERAL or UART_BUFFER equivalent for CAN
 */
int can_receive(can_frame_t *frame, can_data_src_t source)
{
    if (frame == NULL)
    {
        return -1;
    }

    uint16_t count = 0;

    switch(source)
    {
        case CAN_PERIPHERAL:

            if ((can_operations == NULL) || (can_operations->receive == NULL))
            {
                return -1;
            }

            return can_operations->receive(frame);

        case CAN_BUFFER:

            if (can_rx_tail == can_rx_head)
            {
                return 0; // no frames available
            }

            *frame = can_rx_buffer[can_rx_tail];
            can_rx_tail = (can_rx_tail + 1) % CAN_RX_BUFFER_SIZE;

            return 1;

        default:
            return -1;
    }
}


/**
 * @brief CAN RX frame callback
 *
 * Called from CAN ISR.
 */
void drv_cb_can_rx_frame(const can_frame_t *frame)
{
    uint16_t next = (can_rx_head + 1) % CAN_RX_BUFFER_SIZE;

    /* Prevent overflow */
    if (next != can_rx_tail)
    {
        can_rx_buffer[can_rx_head] = *frame;
        can_rx_head = next;
    }
}


/**
 * @brief Frames available in RX buffer
 */
uint16_t drv_cb_can_frames_available(void)
{
    if (can_rx_head >= can_rx_tail)
    {
        return can_rx_head - can_rx_tail;
    }

    return CAN_RX_BUFFER_SIZE - can_rx_tail + can_rx_head;
}

#endif
