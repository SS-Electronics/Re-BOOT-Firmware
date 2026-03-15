/*
File:        bl_types.h
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Include
Info:        Generic types and includes across project
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
#ifndef __INCLUDE_BL_TYPES_H__
#define __INCLUDE_BL_TYPES_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdlib.h>


#define COMM_MAX_DATA 64

/**
 * @brief Generic communication packet
 */
typedef struct
{
    uint8_t  command;
    uint16_t length;
    uint8_t  data[COMM_MAX_DATA];
} comm_packet_t;



#endif /* __INCLUDE_BL_TYPES_H__ */
