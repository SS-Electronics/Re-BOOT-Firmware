/*
File:        bootloader.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      init
Info:        Bootloader command processor — receive loop, sector write, app jump
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
 * Implements the receive-and-respond loop running on the embedded target.
 * Handles all host commands and maintains state to buffer, erase, write,
 * and CRC-verify one flash sector at a time.
 *
 * @par Driver dependencies — must be registered before bootloader_exe()
 *
 *  - @ref bl_flash_driver_register() — MCU flash erase / write / read ops.
 *  - @ref bl_uart_driver_register()  — MCU UART ops  (BOOT_INTERFACE = UART).
 *  - @ref bl_can_driver_register()   — MCU CAN ops   (BOOT_INTERFACE = CAN).
 *  - @ref bl_delay_driver_register() — MCU ms-delay (e.g. HAL_Delay).
 *  - @ref bl_jump_driver_register()  — MCU application-jump sequence.
 *
 * @par Command flow
 * @code
 *  CMD_RESET_REQ       → RESP_TARGET_INFO { APP_START_ADDRESS,
 *                                           FLASH_SECTOR_SIZE, COMM_SEGMENT_SIZE }
 *  CMD_PIPELINE_DATA   → buffer segment into sector_rx_buf → RESP_SEG_ACK
 *  CMD_PIPELINE_VERIFY → CRC32 check → flash_erase + flash_write → RESP_CRC_ACK
 *                                                                  / RESP_CRC_NACK
 *  CMD_START_APP       → write valid-flag → RESP_APP_JUMP_ACK → bl_app_jump()
 * @endcode
 *
 * @par Module state
 *  - @c sector_rx_buf    — accumulates one sector of received data (FLASH_SECTOR_SIZE).
 *  - @c sector_base_addr — flash address of the first byte in @c sector_rx_buf.
 *  - @c write_offset     — running byte count for the current sector.
 *  - @c response_packet  — single reusable outgoing packet (no dynamic alloc).
 */

#include "booloader.h"
/* No MCU-specific headers in this file.
 * All hardware operations (flash, uart, delay, application jump) are
 * forwarded through registered driver callbacks.  See drv_flash.h,
 * drv_uart.h, drv_delay.h, and drv_jump.h. */

/**
 * @defgroup APP_VALID_FLAG Application-valid firmware flag
 * @{
 *
 * The byte at (APP_START_ADDRESS - 1) acts as a single-bit firmware-
 * validity sentinel stored in the last byte of the bootloader's reserved
 * flash region (Sector 1 on STM32F411, 0x08004000 – 0x08007FFF).
 *
 * Convention:
 *   0x00 — valid firmware has been flashed; jump to app on next boot
 *   0xFF — no valid firmware (erased flash); stay in bootloader
 *
 * The flag is written to 0x00 by handle_start_app() after a successful
 * firmware update, just before the first application jump.  It is never
 * erased by the bootloader (only a full chip erase would clear it), so
 * subsequent power-cycles see 0x00 and fast-boot directly into the app.
 */
#define APP_VALID_FLAG_ADDR   (APP_START_ADDRESS - 1u)
#define APP_VALID_FLAG_VALUE  (0x00u)
/** @} */


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


/**
 * @brief Handle CMD_START_APP — mark firmware valid, acknowledge, and jump.
 *
 * Sequence:
 *  1. Send RESP_APP_JUMP_ACK so the host can close the session cleanly.
 *  2. Allow the UART TX FIFO to drain (10 ms via registered delay driver —
 *     no HAL dependency here).
 *  3. Write 0x00 to APP_VALID_FLAG_ADDR so that on the next power-cycle
 *     the bootloader detects a valid firmware and fast-boots into the app
 *     after a 500 ms window.
 *  4. Jump to the application.
 */
static void handle_start_app(void)
{
    /* Acknowledge before jumping so the host can log completion */
    response_packet.command = RESP_APP_JUMP_ACK;
    response_packet.length  = 1;
    response_packet.data[0] = 1;
    transport_send_packet(&response_packet);

    /* Allow UART TX FIFO to drain.  At 115200 baud, 7 bytes take ~0.6 ms;
       10 ms is more than sufficient.  Uses registered delay driver —
       no direct HAL call in this file. */
    delay_ms(10u);

    /* Mark firmware as valid.  Writing 0x00 to a byte that reads 0xFF
       (erased flash) requires only a program operation — no sector erase.
       On STM32F4 this single byte lives in Sector 1 (0x08004000–0x08007FFF)
       which is never touched during normal firmware updates. */
    uint8_t valid_flag = APP_VALID_FLAG_VALUE;
    flash_write(APP_VALID_FLAG_ADDR, &valid_flag, 1u);

    /* Transfer execution to the application.
     * MCU-specific teardown (NVIC, SysTick, VTOR, MSP) is performed by
     * the registered jump driver — no CMSIS headers needed here. */
    bl_app_jump();
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
 * Top-level entry point called from @c main() after all drivers have
 * been registered.  Never returns under normal operation.
 *
 * @par Execution flow
 *
 * @code
 *   ┌─ Interface init (BOOT_INTERFACE selects at compile time) ─┐
 *   │  UART : uart_init() + uart_flush()                        │
 *   │  CAN  : can_init()                                        │
 *   └───────────────────────────────────────────────────────────┘
 *            │
 *            ▼
 *   handle_reset_req()  ← proactive RESP_TARGET_INFO broadcast
 *            │             host can start CMD_PIPELINE_DATA immediately
 *            │
 *   Read APP_VALID_FLAG_ADDR
 *   │
 *   ├─ 0x00  (valid firmware)
 *   │        ┌──── 500 ms trigger window ────┐
 *   │        │  poll transport  1 ms / tick  │
 *   │        │  any valid packet received?   │
 *   │        └───────────────────────────────┘
 *   │               │ YES             │ NO (timeout)
 *   │               ▼                 ▼
 *   │        process_command()    bl_app_jump()
 *   │        ↓ fall into loop     (never returns)
 *   │
 *   └─ 0xFF  (no firmware / erased flash)
 *            fall straight into receive loop
 *
 *   Receive loop:
 *     transport_receive_packet() → process_command() → repeat
 *     CMD_START_APP → write valid-flag → bl_app_jump()
 * @endcode
 *
 * @par Proactive announce
 * @c handle_reset_req() is called unconditionally on boot.  It resets
 * all sector state and transmits @c RESP_TARGET_INFO so a host that is
 * already listening receives target parameters without needing to send
 * @c CMD_RESET_REQ first.  If the host does send @c CMD_RESET_REQ later
 * it is handled normally in the receive loop (idempotent).
 *
 * @par Trigger-window detail
 * The ISR ring buffer is live before the flag check so no byte is lost.
 * @c transport_receive_packet() is non-blocking (returns 0 when empty)
 * and is polled every 1 ms.  At 115200 baud a 6-byte frame arrives in
 * < 1 ms — no packet can be missed within the window.
 */
void bootloader_exe(void)
{
    comm_packet_t rx_packet;

    /* ---- Interface initialisation ----------------------------------------
     * Start the selected communication peripheral so its ISR ring buffer
     * begins accumulating bytes / frames before anything else.
     *
     * UART: initialise the peripheral and flush any power-on garbage.
     * CAN : initialise the peripheral (CAN RX ISR feeds the frame buffer).
     * ----------------------------------------------------------------------- */
#if   (BOOT_INTERFACE == BL_INTERFACE_UART)
    uart_init();
    uart_flush();
#elif (BOOT_INTERFACE == BL_INTERFACE_CAN)
    can_init();
#endif

    /* ---- Proactive target announcement -----------------------------------
     * Broadcast RESP_TARGET_INFO immediately on boot.
     * A listening host can skip CMD_RESET_REQ and push CMD_PIPELINE_DATA
     * straight away.  handle_reset_req() also calls reset_sector_state()
     * internally so no separate state initialisation is needed here.
     * ----------------------------------------------------------------------- */
    handle_reset_req();

    /* ---- Firmware-validity check -----------------------------------------
     * Read the sentinel byte at (APP_START_ADDRESS - 1).
     *
     *   0x00 — valid firmware.
     *           Open 500 ms window; poll for any packet from host.
     *           Packet received → process it, enter update loop.
     *           Timeout         → bl_app_jump().
     *
     *   0xFF (or any non-zero) — no valid firmware / erased flash.
     *           Fall straight into the receive loop and wait indefinitely.
     * ----------------------------------------------------------------------- */
    uint8_t app_flag = *(volatile uint8_t *)APP_VALID_FLAG_ADDR;

    if (app_flag == APP_VALID_FLAG_VALUE)
    {
        /* ---- 500 ms trigger window -------------------------------------------
         * Poll transport_receive_packet() every 1 ms (500 ticks = 500 ms).
         * The function is non-blocking — it returns 0 when the ring buffer
         * is empty — so the ISR continues to fill the buffer during each
         * delay_ms(1) call without missing any incoming bytes / frames.
         *
         * Trigger condition: ANY valid packet from the host.
         *   CMD_PIPELINE_DATA — host starts uploading immediately after
         *                       receiving the proactive RESP_TARGET_INFO.
         *   CMD_RESET_REQ     — host follows the normal protocol flow and
         *                       re-requests target info; accepted for compat.
         * ----------------------------------------------------------------------- */
        uint8_t update_requested = 0u;

        for (uint16_t tick = 0u; tick < 500u; tick++)
        {
            int ret = transport_receive_packet(&rx_packet);

            if (ret > 0)
            {
                /* Any valid packet signals the host wants a firmware update */
                update_requested = 1u;
                break;
            }

            delay_ms(1u);   /* 1 ms tick — ISR fills ring buffer here */
        }

        if (!update_requested)
        {
            /* No host activity within 500 ms — boot existing firmware */
            bl_app_jump();
            /* never returns */
        }

        /* Dispatch the packet that triggered the window (CMD_PIPELINE_DATA
           or CMD_RESET_REQ) then fall into the main receive loop below.   */
        process_command(&rx_packet);
        memset(&rx_packet, 0, sizeof(rx_packet));
    }

    /* ----------------------------------------------------------------
     * Main receive loop
     *
     * Runs for both:
     *   a) the no-valid-firmware path (fall-through from above), and
     *   b) after a trigger-window CMD_RESET_REQ was handled above.
     *
     * The only exit is CMD_START_APP → handle_start_app() → bl_app_jump().
     * ---------------------------------------------------------------- */
    while (1)
    {
        int ret = transport_receive_packet(&rx_packet);

        if (ret > 0)
        {
            process_command(&rx_packet);
            memset(&rx_packet, 0, sizeof(rx_packet));
        }
        /* ret == 0: no byte available yet — loop (non-blocking)   */
        /* ret  < 0: frame error (CRC / length) — loop and re-sync */
    }
}
