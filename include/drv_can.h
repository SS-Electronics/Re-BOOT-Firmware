/*
File:        drv_can.h
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
 * @file drv_can.h
 * @brief Generic CAN driver abstraction interface
 *
 * This module provides a hardware independent CAN interface used by
 * higher layers such as the transport layer or bootloader protocol.
 *
 * The actual CAN hardware implementation must be provided by the
 * application and registered using can_driver_register().
 */

#ifndef __INCLUDE_DRV_CAN_H__
#define __INCLUDE_DRV_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"


/**
 * @defgroup CAN_DRIVER CAN Driver Abstraction
 * @brief Hardware independent CAN driver interface
 *
 * @{
 */


/**
 * @brief CAN operations abstraction
 *
 * Structure containing hardware-specific CAN driver functions.
 * The application must populate this structure and register it
 * using can_driver_register().
 */
typedef struct
{
    /**
     * @brief Initialize CAN hardware
     */
    void (*init)(void);

    /**
     * @brief Close CAN driver
     */
    void (*close)(void);

    /**
     * @brief Transmit CAN frame
     *
     * @param id CAN identifier
     * @param data Pointer to transmit buffer
     * @param len Number of bytes to transmit
     *
     * @return Number of bytes transmitted
     */
    int32_t (*transmit)(uint32_t id,
                        const uint8_t *data,
                        uint8_t len);

    /**
     * @brief Receive CAN frame
     *
     * @param id Pointer to store CAN identifier
     * @param data Receive buffer
     * @param maxlen Maximum data length
     *
     * @return Number of bytes received
     */
    int32_t (*receive)(uint32_t *id,
                       uint8_t *data,
                       uint8_t maxlen);

} can_ops_t;


/**
 * @brief Register CAN driver operations
 *
 * @param ops Pointer to CAN operations structure
 */
void can_driver_register(const can_ops_t *ops);


/**
 * @brief Initialize CAN driver
 *
 * @return
 *  - 0 success
 *  - negative value error
 */
int32_t can_init(void);


/**
 * @brief Close CAN driver
 */
void can_close(void);


/**
 * @brief Transmit CAN frame
 *
 * @param id CAN identifier
 * @param data Pointer to data buffer
 * @param len Data length
 *
 * @return Number of bytes transmitted
 */
int can_transmit(uint32_t id,
                 const uint8_t *data,
                 uint8_t len);


/**
 * @brief Receive CAN frame
 *
 * @param id Pointer to store CAN identifier
 * @param data Receive buffer
 * @param maxlen Maximum data length
 *
 * @return Number of bytes received
 */
int can_receive(uint32_t *id,
                uint8_t *data,
                uint8_t maxlen);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_CAN_H__ */

