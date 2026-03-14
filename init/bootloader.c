/*
File:        bootloader.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Init
Info:        Entry Point of the firmware
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

#include "booloader.h"


/* Example bootloader command IDs */

/* Simulated flash buffer for writing */
static uint8_t flash_buffer[FLASH_SECTOR_SIZE] = {0};
static uint32_t flash_address = 0;    /* current flash address for write */
static uint16_t buffer_index = 0;     /* current index in flash_buffer */

/* Response packet */
static comm_packet_t response_packet;

/* Mock functions for flash operations */
void flash_write(uint32_t address, uint8_t *data, uint16_t len)
{
    /* In real code: write to MCU flash */
//    printf("Flash write: addr=0x%08X len=%d\n", address, len);
}

void jump_to_application(void)
{
    /* In real code: set PC to app start */
//    printf("Jumping to application...\n");
}

/**
 * @brief Process received command
 */
static void process_command(comm_packet_t *packet)
{
    response_packet.command = packet->command;
    response_packet.length  = 1; /* default 1 byte ACK/NACK */

    switch (packet->command)
    {
        case CMD_RESET_REQ:
            /* Reset command: send target info */
//            printf("RESET command received\n");
            response_packet.data[0] = 0;
            /* Example: send MCU/bootloader version or ID if needed */
        break;

//        case CMD_ADDRESS:
//            /* Update flash address */
//            if (packet->length < 4)
//            {
//                response_packet.data[0] = NACK;
//                break;
//            }
//            flash_address = (packet->data[0] << 24) |
//                            (packet->data[1] << 16) |
//                            (packet->data[2] << 8)  |
//                            (packet->data[3]);
//            buffer_index = 0;
//            printf("ADDRESS command: flash_address=0x%08X\n", flash_address);
//            response_packet.data[0] = ACK;
//            break;
//
//        case CMD_DATA:
//            /* Add data to buffer */
//            if (buffer_index + packet->length > MAX_BUFFER_SIZE)
//            {
//                response_packet.data[0] = NACK; /* buffer overflow */
//                break;
//            }
//            memcpy(&flash_buffer[buffer_index], packet->data, packet->length);
//            buffer_index += packet->length;
//            printf("DATA command: received %d bytes, buffer_index=%d\n",
//                    packet->length, buffer_index);
//            response_packet.data[0] = ACK;
//            break;
//
//        case CMD_FLASH_WRITE:
//            /* Write flash sector */
//            if (buffer_index == 0)
//            {
//                response_packet.data[0] = NACK; /* nothing to write */
//                break;
//            }
//            flash_write(flash_address, flash_buffer, buffer_index);
//            flash_address += buffer_index;
//            buffer_index = 0; /* clear buffer */
//            response_packet.data[0] = ACK;
//            break;
//
//        case CMD_APP_START:
//            /* Jump to application */
//            response_packet.data[0] = ACK;
//            transport_send_packet(&response_packet);
//            printf("APP_START command received, jumping to app\n");
//            jump_to_application();
//            break;
//
//        default:
//            response_packet.data[0] = NACK; /* unknown command */
//            break;
    }

    /* Send ACK/NACK for all except APP_START (already sent) */
    if (packet->command != 0)
    {
        transport_send_packet(&response_packet);
    }
}

/**
 * @brief Main bootloader FSM loop
 */
void bootloader_exe(void)
{
    comm_packet_t packet;

    uart_init();


    while (1)
    {
        int ret = transport_receive_packet(&packet);

        if (ret > 0)
        {
            /* Got a valid command, process it */
            process_command(&packet);

            /* Clear packet for next receive */
            memset(&packet, 0, sizeof(packet));
        }
    }
}
