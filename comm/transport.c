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



static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}


#if (BOOT_INTERFACE == BL_INTERFACE_UART)

#include "drv_uart.h"

/* RX parser state */
typedef struct
{
    uint8_t  buffer[COMM_MAX_DATA + 6];
    uint16_t index;
    uint16_t expected_length;
} uart_parser_t;

static uart_parser_t parser;;


int transport_receive_packet(comm_packet_t *packet)
{
    if (!packet)
        return -1;

    uint8_t byte;

    while (uart_receive(&byte, 1, UART_BUFFER) == 1)
    {
        /* Wait for start byte */
        if (parser.index == 0)
        {
            if (byte != ':')
                continue;

            parser.buffer[parser.index++] = byte;
            continue;
        }

        parser.buffer[parser.index++] = byte;

        /* overflow protection */
        if (parser.index >= sizeof(parser.buffer))
        {
            parser.index = 0;
            parser.expected_length = 0;
            continue;
        }

        /* read length */
        if (parser.index == 4)
        {
            parser.expected_length =
                ((uint16_t)parser.buffer[2] << 8) |
                 parser.buffer[3];

            /* invalid length → resync */
            if (parser.expected_length > COMM_MAX_DATA)
            {
                parser.index = 0;
                parser.expected_length = 0;
                continue;
            }
        }

        /* complete frame */
        if (parser.index == parser.expected_length + 6)
        {
            packet->command = parser.buffer[1];
            packet->length  = parser.expected_length;

            memcpy(packet->data,
                   &parser.buffer[4],
                   packet->length);

            uint16_t crc_rx =
                ((uint16_t)parser.buffer[4 + packet->length] << 8) |
                 parser.buffer[5 + packet->length];

            uint16_t crc_calc =
                crc16_ccitt(&parser.buffer[1], packet->length + 3);

            parser.index = 0;
            parser.expected_length = 0;

            if (crc_rx != crc_calc)
                continue;

            return packet->length;
        }
    }

    return 0;
}


int transport_send_packet(comm_packet_t *packet)
{
    if (!packet || packet->length > COMM_MAX_DATA)
        return -1;

    uint8_t buffer[COMM_MAX_DATA + 6];

    buffer[0] = ':';
    buffer[1] = packet->command;
    buffer[2] = (packet->length >> 8) & 0xFF;
    buffer[3] = packet->length & 0xFF;

    memcpy(&buffer[4], packet->data, packet->length);

    uint16_t crc = crc16_ccitt(&buffer[1], packet->length + 3);

    buffer[4 + packet->length] = (crc >> 8) & 0xFF;
    buffer[5 + packet->length] = crc & 0xFF;

    return uart_transmit(buffer, packet->length + 6);
}

#endif



#if (BOOT_INTERFACE == BL_INTERFACE_CAN)

#include "drv_can.h"

int transport_receive_packet(comm_packet_t *packet)
{
    can_frame_t frame;

    int ret = can_receive(&frame);
    if (ret <= 0)
        return ret;

    if (frame.len < 5)
        return -1;

    packet->command = frame.data[0];
    packet->length =
        (frame.data[1] << 8) |
         frame.data[2];

    if (packet->length > COMM_MAX_DATA ||
        packet->length != frame.len - 5)
        return -2;

    memcpy(packet->data,
           &frame.data[3],
           packet->length);

    uint16_t crc_rx =
        ((uint16_t)frame.data[3 + packet->length] << 8) |
         frame.data[4 + packet->length];

    uint16_t crc_calc =
        crc16_ccitt(&frame.data[0], packet->length + 3);

    if (crc_rx != crc_calc)
        return -3;

    return packet->length;
}


int transport_send_packet(comm_packet_t *packet)
{
    if (!packet || packet->length > 5)
        return -1;

    can_frame_t frame;

    frame.id = 0x123;

    frame.data[0] = packet->command;
    frame.data[1] = (packet->length >> 8) & 0xFF;
    frame.data[2] = packet->length & 0xFF;

    memcpy(&frame.data[3],
           packet->data,
           packet->length);

    uint16_t crc =
        crc16_ccitt(&frame.data[0], packet->length + 3);

    frame.data[3 + packet->length] = (crc >> 8) & 0xFF;
    frame.data[4 + packet->length] = crc & 0xFF;

    frame.len = packet->length + 5;

    return can_transmit(frame.id,
                        frame.data,
                        frame.len);
}

#endif
