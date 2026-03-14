/*
File:        bl_protocol_config.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Moudle:      app_config.c
Info:        Protocol related configuration
Dependency:  None

This file is part of Re-BOOT Project.

Re-BOOT is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version
3 of the License, or (at your option) any later version.

Re-BOOT is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Re-BOOT. If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef __BL_PROTOCOL_CONFIG_H__
#define __BL_PROTOCOL_CONFIG_H__

/**
 * @brief Protocol Commands
 */
#define CMD_RESET_REQ          0x10
#define CMD_PIPELINE_DATA      0x11
#define CMD_ADDR_UPDATE        0x12
#define CMD_PIPELINE_VERIFY    0x13
#define CMD_START_APP          0x14

#define RESP_TARGET_INFO       0x30
#define RESP_SEG_ACK           0x31
#define RESP_SEG_NACK          0x32
#define RESP_CRC_ACK           0x33
#define RESP_CRC_NACK          0x34
#define RESP_SECTOR_WR_ACK     0x35
#define RESP_SECTOR_WR_NACK    0x36
#define RESP_APP_JUMP_ACK      0x37
#define RESP_APP_JUMP_NACK     0x38
#define RESP_PIPE_INFO         0x39
#define RESP_PIPELINE_CRC      0x40

#endif /* __BL_PROTOCOL_CONFIG_H__ */

