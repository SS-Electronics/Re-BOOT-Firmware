/*
File:        transport.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      comm
Info:        Communication Abstraction
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
 * @file transport.c
 * @brief Transport layer abstraction for UART / CAN / TCP
 *
 * This module provides a hardware-agnostic transport layer for
 * sending and receiving packets. It supports:
 * - UART: frame format [:][cmd 1B][length 2B][data N][#]
 * - CAN: frame format [cmd 1B][length 2B][data N][#] inside can_frame_t
 * - TCP: raw packet transmission
 *
 * Provides:
 * - Interface-specific send/receive
 * - Generic transport_send_packet() and transport_receive_packet()
 */

#include "transport.h"
#include <string.h>

#if (BOOT_INTERFACE == BL_INTERFACE_UART)
#include "drv_uart.h"

/**
 * @brief UART RX parser state
 */
typedef struct
{
    uint8_t buffer[COMM_MAX_DATA + 5]; /**< Buffer to store incoming UART frame */
    uint16_t index;                    /**< Current write index */
} uart_rx_state_t;

/** @brief UART RX parser instance */
static uart_rx_state_t uart_state = {0};

/**
 * @brief Parse UART byte stream into comm_packet_t
 *
 * UART frame format: [:][cmd 1B][len 2B][data N][#]
 *
 * @param[out] packet Pointer to comm_packet_t to store parsed frame
 * @return
 *  - >0 : Number of data bytes received
 *  - 0  : No complete frame yet
 *  - <0 : Error code
 *         -1 : Null packet pointer
 *         -2 : Frame too short
 *         -3 : Invalid length
 */
int uart_receive_packet(comm_packet_t *packet)
{
    if (!packet) return -1;

    uint8_t byte;

    while (uart_receive(&byte, 1, UART_BUFFER) == 1)
    {
        /* Wait for start byte */
        if (uart_state.index == 0 && byte != ':') continue;

        if (uart_state.index >= sizeof(uart_state.buffer))
        {
            uart_state.index = 0; /* buffer overflow, reset */
        }

        uart_state.buffer[uart_state.index++] = byte;

        /* Check frame terminator */
        if (byte == '#')
        {
            if (uart_state.index < 5) /* : + cmd + len_hi + len_lo + # */
            {
                uart_state.index = 0;
                return -2;
            }

            packet->command = uart_state.buffer[1];
            packet->length  = (uart_state.buffer[2] << 8) | uart_state.buffer[3];

            if (packet->length > COMM_MAX_DATA || packet->length != uart_state.index - 5)
            {
                uart_state.index = 0;
                return -3;
            }

            memcpy(packet->data, &uart_state.buffer[4], packet->length);
            uart_state.index = 0;
            return packet->length;
        }
    }

    return 0; /* incomplete frame */
}

/**
 * @brief Send comm_packet_t over UART
 *
 * UART frame format: [:][cmd 1B][len 2B][data N][#]
 *
 * @param[in] packet Pointer to comm_packet_t to send
 * @return
 *  - Number of bytes transmitted
 *  - <0 : Error
 */
int uart_send_packet(const comm_packet_t *packet)
{
    if (!packet || packet->length > COMM_MAX_DATA) return -1;

    uint8_t buffer[COMM_MAX_DATA + 5]; /* : + cmd + len_hi + len_lo + data + # */

    buffer[0] = ':';
    buffer[1] = (uint8_t)(packet->command & 0xFF);
    buffer[2] = (uint8_t)((packet->length >> 8) & 0xFF);
    buffer[3] = (uint8_t)(packet->length & 0xFF);
    memcpy(&buffer[4], packet->data, packet->length);
    buffer[4 + packet->length] = '#';

    return uart_transmit(buffer, 5 + packet->length);
}

#endif /* BL_INTERFACE_UART */

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)
#include "drv_can.h"

/**
 * @brief Convert CAN frame to comm_packet_t
 *
 * CAN frame format: [cmd 1B][len 2B][data N][#]
 *
 * @param[in] frame Pointer to received CAN frame
 * @param[out] packet Pointer to comm_packet_t to store parsed frame
 * @return
 *  - Number of data bytes copied
 *  - <0 : Error
 */
int can_frame_to_packet(const can_frame_t *frame, comm_packet_t *packet)
{
    if (!frame || !packet || frame->len < 4) return -1;

    packet->command = frame->data[0];
    packet->length  = (frame->data[1] << 8) | frame->data[2];

    if (packet->length > COMM_MAX_DATA || packet->length != frame->len - 4) return -2;

    memcpy(packet->data, &frame->data[3], packet->length);
    return packet->length;
}

/**
 * @brief Convert comm_packet_t to CAN frame
 *
 * @param[in] packet Pointer to comm_packet_t to send
 * @param[out] frame Pointer to CAN frame to populate
 * @param[in] id CAN identifier
 * @return 0 on success, <0 on error
 */
int packet_to_can_frame(const comm_packet_t *packet, can_frame_t *frame, uint32_t id)
{
    if (!packet || !frame || packet->length > 5) return -1;

    frame->id  = id;
    frame->len = 3 + packet->length + 1; /* cmd+len+data+# */

    frame->data[0] = (uint8_t)(packet->command & 0xFF);
    frame->data[1] = (uint8_t)((packet->length >> 8) & 0xFF);
    frame->data[2] = (uint8_t)(packet->length & 0xFF);

    memcpy(&frame->data[3], packet->data, packet->length);
    frame->data[3 + packet->length] = '#';

    return 0;
}

/**
 * @brief Send comm_packet_t over CAN
 *
 * @param[in] packet Pointer to comm_packet_t to send
 * @param[in] id CAN identifier
 * @return Number of bytes transmitted or <0 on error
 */
int can_send_packet(const comm_packet_t *packet, uint32_t id)
{
    can_frame_t frame;
    if (packet_to_can_frame(packet, &frame, id) != 0) return -1;

    return can_transmit(frame.id, frame.data, frame.len);
}

/**
 * @brief Receive comm_packet_t from CAN buffer
 *
 * @param[out] packet Pointer to comm_packet_t to store received frame
 * @return Number of bytes received or <0 on error
 */
int can_receive_packet(comm_packet_t *packet)
{
    can_frame_t frame;
    int ret = can_receive(&frame, CAN_BUFFER);
    if (ret <= 0) return ret;

    return can_frame_to_packet(&frame, packet);
}

#endif /* BL_INTERFACE_CAN */

/**
 * @brief Receive a comm_packet_t from selected transport interface
 *
 * @param[out] packet Pointer to store received packet
 * @return Number of bytes received, 0 if incomplete, <0 on error
 */
int transport_receive_packet(comm_packet_t *packet)
{
#if (BOOT_INTERFACE == BL_INTERFACE_UART)
    return uart_receive_packet(packet);

#elif (BOOT_INTERFACE == BL_INTERFACE_CAN)
    return can_receive_packet(packet);

#elif (BOOT_INTERFACE == BL_INTERFACE_TCP)
    uint8_t buffer[sizeof(comm_packet_t)];
    int len = tcp_receive(buffer, sizeof(buffer));
    if (len <= 0) return -1;

    if (len < sizeof(uint32_t) + sizeof(uint16_t)) return -2;

    memcpy(packet, buffer, sizeof(comm_packet_t));
    return packet->length;

#else
#error "Invalid BOOT_INTERFACE"
#endif
}

/**
 * @brief Send a comm_packet_t via selected transport interface
 *
 * @param[in] packet Pointer to comm_packet_t to send
 * @return Number of bytes sent or <0 on error
 */
int transport_send_packet(comm_packet_t *packet)
{
#if (BOOT_INTERFACE == BL_INTERFACE_UART)
    return uart_send_packet(packet);

#elif (BOOT_INTERFACE == BL_INTERFACE_CAN)
    uint32_t id = 0x123; /* default CAN ID */
    return can_send_packet(packet, id);

#elif (BOOT_INTERFACE == BL_INTERFACE_TCP)
    uint16_t total = sizeof(packet->command) + sizeof(packet->length) + packet->length;
    uint8_t buffer[sizeof(comm_packet_t)];
    memcpy(buffer, packet, total);
    return tcp_transmit(buffer, total);

#else
#error "Invalid BOOT_INTERFACE"
#endif
}
