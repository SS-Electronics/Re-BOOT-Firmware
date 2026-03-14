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
 * @brief Hardware-independent CAN driver abstraction with RX buffer
 *
 * This module provides a generic CAN interface for higher layers
 * such as transport layers or bootloader protocols. It supports
 * both direct hardware reception and a software RX buffer.
 *
 * The application must implement the hardware-specific functions
 * and register them using `bl_can_driver_register()`.
 */


#ifndef __INCLUDE_DRV_CAN_H__
#define __INCLUDE_DRV_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bl_types.h"
#include "reboot_config.h"

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)
/**
 * @defgroup CAN_DRIVER CAN Driver Abstraction
 * @brief Hardware-independent CAN driver interface
 *
 * This module defines a generic interface to interact with
 * CAN hardware. The application provides the actual CAN
 * implementation via a `can_ops_t` structure.
 *
 * It supports a software RX buffer with ISR callback.
 *
 * @{
 */

/**
 * @brief CAN data source for receive operations
 */
typedef enum
{
    CAN_PERIPHERAL, /**< Read directly from hardware peripheral */
    CAN_BUFFER      /**< Read from software RX buffer */
} can_data_src_t;

/**
 * @brief CAN frame structure
 */
typedef struct
{
    uint32_t id;       /**< CAN identifier */
    uint8_t  data[8];  /**< Data bytes (max 8 for standard CAN) */
    uint8_t  len;      /**< Number of valid bytes */
} can_frame_t;

/**
 * @brief CAN hardware operations
 *
 * Contains pointers to hardware-specific CAN functions.
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
     * @brief Transmit a CAN frame
     *
     * @param id   CAN identifier
     * @param data Pointer to data buffer
     * @param len  Number of bytes to transmit (max 8 for standard CAN)
     * @return Number of bytes transmitted, or negative on error
     */
    int32_t (*transmit)(uint32_t id,
                        const uint8_t *data,
                        uint8_t len);

    /**
     * @brief Receive a CAN frame from hardware
     *
     * @param frame Pointer to store received frame
     * @return 1 on success, 0 if no frame, negative on error
     */
    int32_t (*receive)(can_frame_t *frame);

} can_ops_t;

/**
 * @brief Register CAN driver operations
 *
 * @param ops Pointer to hardware-specific CAN operations
 *
 * @note Must be called before using other CAN APIs
 */
void bl_can_driver_register(const can_ops_t *ops);

/**
 * @brief Initialize the CAN driver
 *
 * @return 0 on success, negative value on error
 */
int32_t can_init(void);

/**
 * @brief Close the CAN driver
 */
void can_close(void);

/**
 * @brief Transmit a CAN frame
 *
 * @param id   CAN identifier
 * @param data Pointer to data buffer
 * @param len  Number of bytes to transmit
 * @return Number of bytes transmitted, or negative on error
 */
int can_transmit(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Receive a CAN frame
 *
 * @param frame  Pointer to store received frame
 * @param source Source of data: CAN_PERIPHERAL or CAN_BUFFER
 * @return Number of frames received (1 if success, 0 if buffer empty), negative on error
 */
int can_receive(can_frame_t *frame, can_data_src_t source);

/**
 * @brief CAN RX frame callback
 *
 * Called from ISR to push a frame into software RX buffer.
 *
 * @param frame Pointer to received frame
 */
void drv_cb_can_rx_frame(const can_frame_t *frame);

/**
 * @brief Get number of frames available in software RX buffer
 *
 * @return Number of frames in buffer
 */
uint16_t drv_cb_can_frames_available(void);

/** @} */ // end of CAN_DRIVER group

#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DRV_CAN_H__ */


