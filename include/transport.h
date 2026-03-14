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

#ifndef __INCLUDE_TRANSPORT_H__
#define __INCLUDE_TRANSPORT_H__

#include "bl_types.h"
#include "reboot_config.h"

/**
 * @brief Receive packet from transport interface
 *
 * @param packet Pointer to packet structure
 *
 * @return number of bytes received or negative error
 */
int transport_receive_packet(comm_packet_t *packet);


/**
 * @brief Send packet through transport interface
 *
 * @param packet Pointer to packet structure
 *
 * @return number of bytes transmitted
 */
int transport_send_packet(comm_packet_t *packet);


#endif /* __INCLUDE_TRANSPORT_H__ */
