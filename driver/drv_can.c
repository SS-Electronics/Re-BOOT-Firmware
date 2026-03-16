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
 * @brief CAN driver abstraction with ISR-fed RX ring buffer.
 *
 * @par Re-BOOT CAN frame convention
 *
 *  ┌────────────┬─────┬──────────────────┐
 *  │  CAN_ID    │ DLC │  DATA[0..DLC-1]  │
 *  └────────────┴─────┴──────────────────┘
 *   = CMD          = length  = payload bytes (0–8)
 *
 * The CAN identifier carries the Re-BOOT command code directly.
 * DLC is the exact payload byte count.
 * The data field contains raw payload bytes — no CMD byte, no LEN
 * bytes, and no software CRC (CAN hardware provides its own 15-bit CRC).
 *
 * @par Data flow
 * @code
 *   MCU CAN RX ISR
 *       └─► drv_cb_can_rx_frame(&frame)
 *               stores { .id, .len, .data[8] } → ring buffer [head]
 *               advances head
 *
 *   transport_receive_packet()  (main context)
 *       └─► can_receive(&frame, CAN_BUFFER)
 *               copies ring buffer [tail] → frame
 *               advances tail
 *               returns 1 (frame ready) or 0 (buffer empty)
 * @endcode
 */

#include <string.h>
#include "drv_can.h"

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)

/* ====================================================================
   Module-level state
   ==================================================================== */

/** Registered hardware-specific CAN implementation */
static const can_ops_t *can_operations = NULL;

/**
 * @brief ISR-fed software RX ring buffer.
 *
 * Each slot is a full can_frame_t:
 *   .id       — 32-bit CAN identifier  (Re-BOOT: command code)
 *   .len      — DLC (0–8)              (Re-BOOT: payload byte count)
 *   .data[8]  — raw payload bytes
 *
 * can_rx_head — next write slot, updated by ISR (drv_cb_can_rx_frame)
 * can_rx_tail — next read  slot, updated by main (can_receive)
 */
static can_frame_t       can_rx_buffer[CAN_RX_BUFFER_SIZE];
static volatile uint16_t can_rx_head = 0u;
static volatile uint16_t can_rx_tail = 0u;


/* ====================================================================
   Driver registration & lifecycle
   ==================================================================== */

/**
 * @brief Register a hardware CAN implementation.
 *
 * Must be called once before any other CAN API.
 *
 * @param ops  Pointer to MCU-specific CAN operation struct.
 */
void bl_can_driver_register(const can_ops_t *ops)
{
    can_operations = ops;
}


/**
 * @brief Initialise the CAN peripheral.
 *
 * @return  0  success.
 * @return -1  no driver registered, or init pointer is NULL.
 */
int32_t can_init(void)
{
    if ((can_operations == NULL) || (can_operations->init == NULL))
        return -1;

    can_operations->init();
    return 0;
}


/**
 * @brief De-initialise / close the CAN peripheral.
 */
void can_close(void)
{
    if ((can_operations != NULL) && (can_operations->close != NULL))
        can_operations->close();
}


/* ====================================================================
   Transmit
   ==================================================================== */

/**
 * @brief Transmit a CAN frame via the registered hardware driver.
 *
 * @param id    CAN identifier (Re-BOOT: command code).
 * @param data  Payload bytes  (Re-BOOT: raw data, no overhead).
 * @param len   DLC — number of bytes to send (0–8).
 *
 * @return Number of bytes transmitted, or negative on error.
 */
int can_transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    if ((can_operations == NULL) || (can_operations->transmit == NULL))
        return -1;

    return can_operations->transmit(id, data, len);
}


/* ====================================================================
   Receive
   ==================================================================== */

/**
 * @brief Receive one CAN frame.
 *
 * @par CAN_BUFFER  (normal / ISR path — use this in the bootloader)
 * Pops the oldest frame from the ring buffer filled by
 * drv_cb_can_rx_frame().  Returns 1 with the frame, or 0 when
 * the buffer is empty so the caller can yield without blocking.
 *
 * @par CAN_PERIPHERAL  (polled path — targets without an ISR)
 * Reads directly from hardware via can_ops_t::receive.
 *
 * The populated frame carries:
 *   frame->id       — CAN_ID  → mapped to CMD   by transport layer
 *   frame->len      — DLC     → mapped to length by transport layer
 *   frame->data[]   — bytes   → mapped to data[] by transport layer
 *
 * @param frame   Output.  Must not be NULL.
 * @param source  CAN_BUFFER or CAN_PERIPHERAL.
 *
 * @return  1  Frame received and written to *frame.
 * @return  0  No frame available.
 * @return -1  Invalid argument or driver not registered.
 */
int can_receive(can_frame_t *frame, can_data_src_t source)
{
    if (frame == NULL)
        return -1;

    switch (source)
    {
        /* ----------------------------------------------------------
           Polled hardware path
        ---------------------------------------------------------- */
        case CAN_PERIPHERAL:
            if ((can_operations == NULL) || (can_operations->receive == NULL))
                return -1;

            return can_operations->receive(frame);

        /* ----------------------------------------------------------
           Ring-buffer path  (ISR-driven, used by transport layer)
        ---------------------------------------------------------- */
        case CAN_BUFFER:

            if (can_rx_tail == can_rx_head)
                return 0;   /* buffer empty */

            /* Copy the complete frame: id, len (DLC), all 8 data bytes */
            frame->id  = can_rx_buffer[can_rx_tail].id;
            frame->len = can_rx_buffer[can_rx_tail].len;
            memcpy(frame->data, can_rx_buffer[can_rx_tail].data, 8u);

            /* Advance tail (wraps at CAN_RX_BUFFER_SIZE) */
            can_rx_tail = (uint16_t)((can_rx_tail + 1u) % CAN_RX_BUFFER_SIZE);

            return 1;

        default:
            return -1;
    }
}


/* ====================================================================
   ISR callback — called from CAN RX interrupt
   ==================================================================== */

/**
 * @brief Push one received CAN frame into the software RX ring buffer.
 *
 * *** Wire this function to your MCU's CAN RX interrupt handler. ***
 *
 * Stores the complete frame into the next free ring-buffer slot:
 *
 *   can_rx_buffer[head].id       ← frame->id   (CAN identifier = CMD)
 *   can_rx_buffer[head].len      ← frame->len  (DLC = payload length)
 *   can_rx_buffer[head].data[8]  ← frame->data (all 8 bytes copied)
 *
 * The head pointer is advanced LAST so that a reader (can_receive)
 * running in main context never observes a partially written slot.
 *
 * Frames are silently dropped when the buffer is full (oldest-kept
 * policy).  Increase CAN_RX_BUFFER_SIZE in reboot_config.h if
 * overflow occurs at high bus loads.
 *
 * @param frame  Frame delivered by hardware ISR.  Must not be NULL.
 *               Contents are copied — safe to reuse immediately.
 */
void drv_cb_can_rx_frame(const can_frame_t *frame)
{
    if (frame == NULL)
        return;

    uint16_t next = (uint16_t)((can_rx_head + 1u) % CAN_RX_BUFFER_SIZE);

    /* Buffer full — drop incoming frame, preserve unread data */
    if (next == can_rx_tail)
        return;

    /* Store CAN_ID (= CMD), DLC (= length), and full 8-byte data field */
    can_rx_buffer[can_rx_head].id  = frame->id;
    can_rx_buffer[can_rx_head].len = frame->len;
    memcpy(can_rx_buffer[can_rx_head].data, frame->data, 8u);

    /* Advance head last — acts as the publish/commit point */
    can_rx_head = next;
}


/**
 * @brief Return the number of unread frames in the RX ring buffer.
 *
 * @return 0 … CAN_RX_BUFFER_SIZE − 1.
 */
uint16_t drv_cb_can_frames_available(void)
{
    if (can_rx_head >= can_rx_tail)
        return (uint16_t)(can_rx_head - can_rx_tail);

    return (uint16_t)(CAN_RX_BUFFER_SIZE - can_rx_tail + can_rx_head);
}

#endif /* BOOT_INTERFACE == BL_INTERFACE_CAN */
