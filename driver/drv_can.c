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
 * @brief CAN driver abstraction implementation
 *
 * This module forwards CAN API calls to the hardware-specific
 * implementation registered by the application.
 */

#include "drv_can.h"

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)

/**
 * @brief Pointer to registered CAN operations
 */
static const can_ops_t *can_operations = NULL;


/**
 * @brief Register CAN operations
 */
void can_driver_register(const can_ops_t *ops)
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
 * @brief Transmit CAN frame
 */
int can_transmit(uint32_t id,
                 const uint8_t *data,
                 uint8_t len)
{
    if ((can_operations == NULL) || (can_operations->transmit == NULL))
    {
        return -1;
    }

    return can_operations->transmit(id, data, len);
}


/**
 * @brief Receive CAN frame
 */
int can_receive(uint32_t *id,
                uint8_t *data,
                uint8_t maxlen)
{
    if ((can_operations == NULL) || (can_operations->receive == NULL))
    {
        return -1;
    }

    return can_operations->receive(id, data, maxlen);
}

#endif
