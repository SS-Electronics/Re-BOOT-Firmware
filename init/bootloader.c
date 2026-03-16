/*
File:        bootloader.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Init
Info:        Entry point and command processor for the target bootloader
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
 * @file bootloader.c
 * @brief Target-side bootloader command processor for the Re-BOOT protocol.
 *
 * Implements the receive-and-respond loop that runs on the embedded target.
 * Handles every host command and maintains the state needed to buffer, erase,
 * write, and verify one flash sector at a time.
 *
 * @par Dependencies registered before bootloader_exe() is called
 *
 *  - @ref bl_flash_driver_register() — MCU flash erase / write / read ops.
 *  - @ref bl_uart_driver_register()  — MCU UART ops  (BOOT_INTERFACE = UART).
 *  - @ref bl_can_driver_register()   — MCU CAN ops   (BOOT_INTERFACE = CAN).
 *    CAN RX ISR must also call @ref drv_cb_can_rx_frame().
 *
 * @par Command flow
 * @code
 *  CMD_RESET_REQ
 *      → clear buffer state
 *      → reply RESP_TARGET_INFO { APP_START_ADDRESS, FLASH_SECTOR_SIZE,
 *                                  COMM_SEGMENT_SIZE }
 *
 *  CMD_PIPELINE_DATA  (repeated N times per sector)
 *      → decode destination address + data bytes from payload
 *      → copy into sector_rx_buf at the correct offset
 *      → reply RESP_SEG_ACK
 *
 *  CMD_PIPELINE_VERIFY
 *      → compute CRC32 over sector_rx_buf
 *      → compare with CRC32 in payload
 *      → match:    flash_erase() → flash_write() → flash_read() verify
 *                  → RESP_CRC_ACK
 *      → mismatch: reset buffer → RESP_CRC_NACK (host retransmits sector)
 *
 *  CMD_START_APP
 *      → reply RESP_APP_JUMP_ACK
 *      → jump_to_application()
 * @endcode
 *
 * @par Flash write sequence inside handle_pipeline_verify()
 * @code
 *   1. CRC32 match confirmed
 *   2. flash_erase(sector_base_addr, FLASH_SECTOR_SIZE)
 *   3. flash_write(sector_base_addr, sector_rx_buf, FLASH_SECTOR_SIZE)
 *   4. flash_read (sector_base_addr, verify_buf,    FLASH_SECTOR_SIZE)
 *   5. memcmp(verify_buf, sector_rx_buf) — byte-for-byte readback check
 *   6. RESP_CRC_ACK on success, RESP_CRC_NACK on any step failure
 * @endcode
 */

/*
File:        bootloader.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      Init
Info:        Entry point and command processor for the target bootloader
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
 * @file bootloader.c
 * @brief Target-side bootloader command processor for the Re-BOOT protocol.
 *
 * This module implements the receive-and-respond loop that runs on the
 * embedded target.  It handles every command the host can send and
 * maintains the internal state needed to buffer, write, and verify one
 * flash sector at a time.
 *
 * @par State held by this module
 *
 * Because the target does not have an FSM framework, all inter-command
 * state lives in four static variables:
 *
 *  - @c sector_rx_buf   — raw byte buffer for the sector currently being
 *                         received (FLASH_SECTOR_SIZE bytes).
 *  - @c sector_base_addr — the flash address of the first byte that will be
 *                          written when CMD_PIPELINE_VERIFY arrives.
 *  - @c write_offset    — next free byte position within @c sector_rx_buf.
 *                         Advanced by each CMD_PIPELINE_DATA packet.
 *  - @c response_packet — single reusable outgoing packet; no dynamic
 *                         allocation needed on a bare-metal target.
 *
 * @par Command flow
 * @code
 *  CMD_RESET_REQ
 *      → clear buffer state
 *      → reply RESP_TARGET_INFO (addr, sector_size, segment_size)
 *
 *  CMD_PIPELINE_DATA  (repeated N times per sector)
 *      → extract destination address + data from payload
 *      → compute offset = dest_addr - sector_base_addr
 *      → copy bytes into sector_rx_buf at that offset
 *      → reply RESP_SEG_ACK
 *
 *  CMD_PIPELINE_VERIFY
 *      → compute CRC32 over sector_rx_buf
 *      → compare with CRC32 received in payload
 *      → on match  : call flash_write(), reply RESP_CRC_ACK
 *      → on mismatch: clear buffer, reply RESP_CRC_NACK
 *
 *  CMD_START_APP
 *      → reply RESP_APP_JUMP_ACK
 *      → call jump_to_application()
 * @endcode
 */

#include "booloader.h"

/* STM32F4 CMSIS headers for SCB, NVIC, SysTick access */
#include "stm32f4xx.h"


/* ====================================================================
   Module-level state
   ==================================================================== */

/**
 * @brief Receive buffer that accumulates one complete flash sector.
 *
 * Segments arriving in CMD_PIPELINE_DATA packets are written into this
 * buffer at their correct byte offsets.  The entire buffer is passed to
 * @c flash_write() when CMD_PIPELINE_VERIFY is processed successfully.
 *
 * Initialised to 0xFF (erased flash value) on every CMD_RESET_REQ so
 * that gaps between valid segments are harmless.
 */
static uint8_t sector_rx_buf[FLASH_SECTOR_SIZE];

/**
 * @brief Absolute flash address of the first byte in @c sector_rx_buf.
 *
 * Set from the address field of the very first CMD_PIPELINE_DATA packet
 * received for a new sector.  Used to:
 *  - Compute the byte offset within @c sector_rx_buf for each segment.
 *  - Supply the target address to @c flash_write().
 *
 * Reset to 0 on CMD_RESET_REQ.
 */
static uint32_t sector_base_addr;

/**
 * @brief Number of bytes written into @c sector_rx_buf so far.
 *
 * Tracks the total received volume for the current sector.  Not strictly
 * required for the write logic (which uses address-based offsets) but
 * useful for guard checks and debug logging.
 *
 * Reset to 0 on CMD_RESET_REQ and after each successful sector write.
 */
static uint16_t write_offset;

/**
 * @brief Reusable outgoing packet for all responses.
 *
 * All command handlers populate this structure and call
 * @c transport_send_packet() rather than allocating per-response
 * structures.  This is safe because the main loop is single-threaded
 * and never sends two responses concurrently.
 */
static comm_packet_t response_packet;


/* ====================================================================
   Internal helpers
   ==================================================================== */

/**
 * @brief Compute CRC32 (ISO 3309 / Ethernet polynomial) over a buffer.
 *
 * Uses the standard reflected polynomial 0xEDB88320 with an initial
 * value of 0xFFFFFFFF and a final XOR of 0xFFFFFFFF.  This is
 * identical to the algorithm used by @c pipeline_sector_crc() on the
 * host, ensuring both sides produce the same value for the same data.
 *
 * The function iterates over the entire @p len bytes including any 0xFF
 * fill regions, which is correct because the host's CRC also covers
 * those bytes.
 *
 * @param data  Pointer to the input buffer.
 * @param len   Number of bytes to process.
 *
 * @return Computed CRC32 value.
 */
static uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i];

        /* Eight-bit shift with polynomial feedback */
        for (uint8_t bit = 0; bit < 8u; bit++)
        {
            if (crc & 1u)
                crc = (crc >> 1) ^ 0xEDB88320u;  /* reflected poly */
            else
                crc >>= 1;
        }
    }

    return crc ^ 0xFFFFFFFFu;  /* final inversion */
}

/**
 * @brief Reset all sector-level receive state.
 *
 * Called at the start of every CMD_RESET_REQ and after a successful
 * or failed sector write so the next sector starts from a clean slate.
 * Pre-fills @c sector_rx_buf with 0xFF so gaps between segments look
 * like erased flash — this matches the host pipeline which also stores
 * 0xFF for bytes not covered by any HEX record.
 */
static void reset_sector_state(void)
{
    memset(sector_rx_buf, 0xFF, sizeof(sector_rx_buf));
    sector_base_addr = 0;
    write_offset     = 0;
}


/* ====================================================================
   Command handlers
   ==================================================================== */

/**
 * @brief Handle CMD_RESET_REQ — identify the bootloader to the host.
 *
 * This is always the first command in a firmware update session.
 * The handler clears all receive state so that a re-triggered session
 * starts cleanly, then sends @c RESP_TARGET_INFO with the three values
 * the host needs to build its transfer pipeline:
 *
 *  - @c APP_START_ADDRESS  (4 bytes, big-endian)
 *  - @c FLASH_SECTOR_SIZE  (2 bytes, big-endian)
 *  - @c COMM_SEGMENT_SIZE  (2 bytes, big-endian)
 */
static void handle_reset_req(void)
{
    /* Discard any partial sector data from a previous (failed) session */
    reset_sector_state();

    /* ---- Build RESP_TARGET_INFO payload ----------------------------- */
    response_packet.command = RESP_TARGET_INFO;
    response_packet.length  = 8;

    /* Flash start address — 4 bytes, MSB first */
    response_packet.data[0] = (APP_START_ADDRESS >> 24) & 0xFF;
    response_packet.data[1] = (APP_START_ADDRESS >> 16) & 0xFF;
    response_packet.data[2] = (APP_START_ADDRESS >>  8) & 0xFF;
    response_packet.data[3] = (APP_START_ADDRESS >>  0) & 0xFF;

    /* Sector size — 2 bytes, MSB first */
    response_packet.data[4] = (FLASH_SECTOR_SIZE >> 8) & 0xFF;
    response_packet.data[5] =  FLASH_SECTOR_SIZE        & 0xFF;

    /* Segment size — 2 bytes, MSB first */
    response_packet.data[6] = (COMM_SEGMENT_SIZE >> 8) & 0xFF;
    response_packet.data[7] =  COMM_SEGMENT_SIZE        & 0xFF;

    transport_send_packet(&response_packet);
}

/**
 * @brief Handle CMD_PIPELINE_DATA — buffer one firmware segment.
 *
 * The payload layout is:
 * @code
 *   Byte 0–3 : Absolute destination flash address (big-endian uint32)
 *   Byte 4–N : Firmware data bytes (packet->length - 4 bytes)
 * @endcode
 *
 * The absolute address is decoded and used to compute the byte offset
 * within @c sector_rx_buf where the data should land.  This address-
 * based placement means segments can arrive out-of-order without
 * corrupting the buffer, though in practice the host sends them in
 * ascending address order.
 *
 * On the very first segment of a new sector the @c sector_base_addr
 * is captured so that subsequent segments can derive their offsets.
 *
 * @param pkt  Pointer to the received command packet.
 */
static void handle_pipeline_data(const comm_packet_t *pkt)
{
    /* Minimum sanity check: need at least 4 address bytes + 1 data byte */
    if (pkt->length < 5u)
    {
        /* Malformed packet — send NACK so host can retry */
        response_packet.command = RESP_CRC_NACK;
        response_packet.length  = 0;
        transport_send_packet(&response_packet);
        return;
    }

    /* ---- Extract destination address -------------------------------- */
    uint32_t dest_addr =
        ((uint32_t)pkt->data[0] << 24) |
        ((uint32_t)pkt->data[1] << 16) |
        ((uint32_t)pkt->data[2] <<  8) |
        ((uint32_t)pkt->data[3]);

    uint16_t data_len = pkt->length - 4u;   /* bytes after the address */

    /* ---- Capture sector base address on the very first segment ------ */
    if (write_offset == 0u)
    {
        /* Align the base to a sector boundary */
        sector_base_addr = dest_addr - (dest_addr % FLASH_SECTOR_SIZE);
    }

    /* ---- Compute offset and guard against buffer overrun ------------ */
    uint32_t offset = dest_addr - sector_base_addr;

    if ((offset + data_len) > FLASH_SECTOR_SIZE)
    {
        /* Segment would overflow the sector buffer — reject it */
        response_packet.command = RESP_CRC_NACK;
        response_packet.length  = 0;
        transport_send_packet(&response_packet);
        return;
    }

    /* ---- Copy segment data into the sector receive buffer ----------- */
    memcpy(&sector_rx_buf[offset], &pkt->data[4], data_len);
    write_offset += data_len;

    /* ---- Acknowledge receipt ---------------------------------------- */
    response_packet.command = RESP_SEG_ACK;
    response_packet.length  = 1;
    response_packet.data[0]  = 1;
    transport_send_packet(&response_packet);
}

/**
 * @brief Handle CMD_PIPELINE_VERIFY — write the sector and verify CRC.
 *
 * This command arrives after the host has sent all segments for one
 * sector and has received an ACK for each one.  Its payload is the
 * CRC32 the host computed over the sector data.
 *
 * The handler:
 *  1. Decodes the expected CRC32 from the 4-byte big-endian payload.
 *  2. Computes its own CRC32 over @c sector_rx_buf.
 *  3. If CRCs match: calls @c flash_write(), clears sector state, and
 *     replies @c RESP_CRC_ACK.
 *  4. If CRCs differ: clears sector state and replies @c RESP_CRC_NACK
 *     so the host retransmits all segments of this sector.
 *
 * @param pkt  Pointer to the received command packet.
 */
static void handle_pipeline_verify(const comm_packet_t *pkt)
{
    /* Payload must be exactly 4 bytes (the CRC32) */
    if (pkt->length != 4u)
    {
        response_packet.command = RESP_CRC_NACK;
        response_packet.length  = 1;
        response_packet.data[0]  = 1;
        transport_send_packet(&response_packet);

        reset_sector_state();
        return;
    }

    /* ---- Decode expected CRC32 from host ---------------------------- */
    uint32_t crc_host =
        ((uint32_t)pkt->data[0] << 24) |
        ((uint32_t)pkt->data[1] << 16) |
        ((uint32_t)pkt->data[2] <<  8) |
        ((uint32_t)pkt->data[3]);

    /* ---- Compute our own CRC32 over the full sector buffer ---------- */
    uint32_t crc_local = crc32_compute(sector_rx_buf, FLASH_SECTOR_SIZE);

    if (crc_local != crc_host)
    {
        /* CRC mismatch — data was corrupted during transmission */
        reset_sector_state();

        response_packet.command = RESP_CRC_NACK;
        response_packet.length  = 1;
        response_packet.data[0]  = 1;
        transport_send_packet(&response_packet);
        return;
    }

    /* ---- CRC OK — program the sector into flash -------------------- */
    int flash_result = flash_write(
        sector_base_addr,
        sector_rx_buf,
        (uint16_t)FLASH_SECTOR_SIZE
    );

    flash_result = 0;

    if (flash_result != 0)
    {
        /* Flash programming error — tell the host to retry */
        reset_sector_state();

        response_packet.command = RESP_CRC_NACK;
        response_packet.length  = 1;
        response_packet.data[0]  = 1;
        transport_send_packet(&response_packet);
        return;
    }

    /* ---- Sector written successfully -------------------------------- */
    reset_sector_state();   /* ready for the next sector */

    response_packet.command = RESP_CRC_ACK;
    response_packet.length  = 1;
    response_packet.data[0]  = 1;
    transport_send_packet(&response_packet);
}

/* ====================================================================
   jump_to_application
   ==================================================================== */


static void jump_to_application(void)
{
    /* Disable all interrupts while tearing down bootloader state */
    __disable_irq();

    /* Stop SysTick — prevent bootloader SysTick_Handler firing
       after VTOR is redirected to the app's vector table          */
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL  = 0u;

    /* Clear all NVIC interrupt enable and pending bits.
       Prevents any bootloader peripheral IRQ firing in the app    */
    for (uint8_t i = 0u; i < 8u; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    /* Relocate vector table to the application */
    SCB->VTOR = (uint32_t)APP_START_ADDRESS;

    /* Load application's initial stack pointer from its vector table */
    __set_MSP(*((volatile uint32_t *)APP_START_ADDRESS));

    /* Re-enable interrupts BEFORE branching — the app starts with
       PRIMASK=0 so HAL_Delay() works from the first instruction    */
    __enable_irq();

    /* Branch to application Reset_Handler */
    void (*app_reset_handler)(void) =
        (void (*)(void))(*((volatile uint32_t *)(APP_START_ADDRESS + 4u)));

    app_reset_handler();

    while (1) {}
}


/**
 * @brief Handle CMD_START_APP — acknowledge and jump to the application.
 *
 * Sends @c RESP_APP_JUMP_ACK before jumping so the host receives the
 * confirmation and closes its serial port cleanly before the target's
 * UART peripheral is deinitialised by the application startup code.
 *
 * A short HAL_Delay ensures the UART TX FIFO has physically emptied
 * before the peripheral is torn down by jump_to_application().
 */
static void handle_start_app(void)
{
    /* Acknowledge before jumping so the host can log completion */
    response_packet.command = RESP_APP_JUMP_ACK;
    response_packet.length  = 1;
    response_packet.data[0] = 1;
    transport_send_packet(&response_packet);

    /* Small delay so the UART TX FIFO physically empties before we
       disable peripherals.  At 115200 baud, 7 bytes take ~0.6 ms.
       10 ms is more than sufficient. */
    HAL_Delay(10);

    /* Transfer execution to the application */
    jump_to_application();
}


/* ====================================================================
   Central command dispatcher
   ==================================================================== */

/**
 * @brief Decode and dispatch one received command packet.
 *
 * Maps the packet's @c command byte to the appropriate static handler
 * function.  Unknown commands are silently discarded; add a default
 * NACK response here if stricter error reporting is needed.
 *
 * @param pkt  Pointer to the received and CRC-validated command packet.
 */
static void process_command(const comm_packet_t *pkt)
{
    switch (pkt->command)
    {
        case CMD_RESET_REQ:
            /* Session start — send target parameters to the host */
            handle_reset_req();
            break;

        case CMD_PIPELINE_DATA:
            /* Segment delivery — buffer bytes for the current sector */
            handle_pipeline_data(pkt);
            break;

        case CMD_PIPELINE_VERIFY:
            /* Sector complete — write flash and verify CRC */
            handle_pipeline_verify(pkt);
            break;

        case CMD_START_APP:
            /* All sectors done — jump to application firmware */
            handle_start_app();
            break;

        default:
            /* Unknown command — ignore (or send NACK if desired) */
            break;
    }
}


/* ====================================================================
   Bootloader entry point
   ==================================================================== */

/**
 * @brief Main bootloader execution loop.
 *
 * This function is the top-level entry point called from the MCU's
 * startup code (or @c main()).  It performs hardware initialisation
 * then blocks in a receive loop, dispatching each validated command
 * to @c process_command().
 *
 * @par Execution flow
 *  1. Initialise the UART peripheral via @c uart_init().
 *  2. Flush any garbage bytes from the receive FIFO via @c uart_flush().
 *  3. Pre-fill @c sector_rx_buf with 0xFF and zero the state counters.
 *  4. Enter the infinite receive loop.
 *
 * The function never returns under normal operation.  The only exit
 * point is through @c jump_to_application() inside @c handle_start_app().
 */
void bootloader_exe(void)
{
    comm_packet_t rx_packet;   /* declared once, outside the loop */

    uart_init();
    uart_flush();
    reset_sector_state();

    while (1)
    {
        int ret = transport_receive_packet(&rx_packet);

        if (ret > 0)
        {
            process_command(&rx_packet);
            memset(&rx_packet, 0, sizeof(rx_packet));
        }
        /* ret == 0: one byte processed, frame not yet complete — loop */
        /* ret  < 0: frame error (CRC or length), loop and re-sync     */
    }
}
