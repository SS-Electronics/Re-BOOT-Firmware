/*
File:        bootloader.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      app_config.c
Info:        Bootloader entry point
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


#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

#include "bl_types.h"
#include "global.h"

#include "drv_uart.h"
#include "drv_can.h"
#include "drv_flash.h"
#include "drv_delay.h"
#include "drv_jump.h"

#include "transport.h"

#include "bl_protocol_config.h"
#include "reboot_config.h"




void bootloader_exe(void);



#endif /* __BOOTLOADER_H__ */
