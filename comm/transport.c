/*
File:        transport.c
Author:      Subhajit Roy
             subhajitroy005@gmail.com

Module:      comm
Info:        Transport layer — packet framing, CRC16, send/receive for UART / CAN
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
 * @file transport.c
 * @brief Hardware-agnostic transport layer for Re-BOOT firmware.
 *
 * This module provides @c transport_send_packet() and
 * @c transport_receive_packet() for both UART and CAN interfaces,
 * selected at compile time via @c BOOT_INTERFACE.
 *
 * @par Wire frame format (UART)
 * Both the host (PC) and the target use the identical frame layout:
 * @code
 *  ┌─────┬─────┬───────┬───────┬──────────────┬───────┬───────┐
 *  │ ':' │ CMD │ LEN_H │ LEN_L │ DATA[0..N-1] │ CRC_H │ CRC_L │
 *  └─────┴─────┴───────┴───────┴──────────────┴───────┴───────┘
 *    1 B   1 B    1 B     1 B       N bytes      1 B     1 B
 * @endcode
 *
 * CRC16-CCITT is computed over @c CMD + @c LEN_H + @c LEN_L + @c DATA.
 * The @c ':' delimiter is @b not included in the CRC.
 *
 * @par Why the CRC bytes must always be present
 * The host parser reads exactly 6 + N bytes per frame.  If the target
 * omits the two CRC bytes, the host consumes the first two bytes of
 * the @e next frame as the CRC, which never matches, and it discards
 * every packet with a -5 error, hanging the update indefinitely.
 *
 * @par UART receiver design — state machine vs. accumulator
 * The original receiver used a raw byte accumulator with a
 * @c while(uart_receive(...) == 1) loop.  That design had two problems:
 *
 *  1. If @c uart_receive blocks, the function never returns between
 *     bytes, so @c bootloader_exe cannot do anything else while waiting
 *     for the next byte.  If @c uart_receive is non-blocking and returns
 *     0 when no byte is ready, the function exits mid-frame with
 *     @c parser.index pointing somewhere in the middle.  On the next call,
 *     the @c index == 0 guard is not taken, so the very first byte of
 *     the next call is appended at the wrong position.
 *
 *  2. A CRC failure caused a @c continue inside the same @c while loop,
 *     which meant the parser silently discarded the frame but kept
 *     the function alive — starving the main bootloader loop.
 *
 * The replacement uses a seven-state machine (states 0–6) with
 * module-level variables so the parser position survives across calls
 * regardless of whether @c uart_receive blocks or not.  A single
 * @c parser_reset() call atomically zeros all four state variables and
 * is called on every exit path (success, length error, CRC error) so
 * the next call always starts scanning cleanly for a @c ':' delimiter.
 */

#include "transport.h"
#include <string.h>


/* ====================================================================
   CRC16-CCITT — shared by both UART and CAN sections
   ==================================================================== */

/**
 * @brief Compute CRC16-CCITT (polynomial 0x1021, init 0xFFFF).
 *
 * This is an exact copy of the function used in the host-side
 * @c transport_layer.c.  Both sides @b must use the same algorithm,
 * the same polynomial, the same initial value, and process bytes in
 * the same order or every packet will fail CRC validation.
 *
 * The CRC is processed MSB-first (non-reflected), which matches the
 * original CCITT specification.
 *
 * @param data  Pointer to the input byte buffer.
 * @param len   Number of bytes to process.
 *
 * @return CRC16-CCITT value.
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8u; j++)
        {
            if (crc & 0x8000u)
                crc = (crc << 1) ^ 0x1021u;
            else
                crc <<= 1;
        }
    }

    return crc;
}


/* ====================================================================
   UART implementation
   ==================================================================== */

#if (BOOT_INTERFACE == BL_INTERFACE_UART)

#include "drv_uart.h"

/* ------------------------------------------------------------------
   UART RX parser state  (module-level — persists across calls)
   ------------------------------------------------------------------

   Declaring these at module scope (rather than as function-local
   statics or inside a struct) means parser_reset() can zero all of
   them unconditionally from any error path.  A function-local static
   can only be reset from inside that function, which makes clean
   error recovery cumbersome.
------------------------------------------------------------------ */

/**
 * @brief Current parser state (0 = idle, scanning for ':').
 *
 * State encoding:
 *  0 — waiting for @c ':' start-of-frame delimiter
 *  1 — reading CMD byte
 *  2 — reading LEN_H byte
 *  3 — reading LEN_L byte
 *  4 — reading DATA bytes (repeated @c rx_length times)
 *  5 — reading CRC_H byte
 *  6 — reading CRC_L byte and validating
 */
static uint8_t  rx_state  = 0;

/**
 * @brief Data byte index within the current frame's payload.
 *
 * Counts bytes written into @c packet->data[] during state 4.
 * Reset to 0 by @c parser_reset() at the start of each new frame.
 */
static uint16_t rx_index  = 0;

/**
 * @brief Payload length decoded from LEN_H:LEN_L in state 3.
 *
 * Used in state 4 to know when the data phase is complete, and
 * again in state 6 to build the CRC verification buffer.
 */
static uint16_t rx_length = 0;

/**
 * @brief Received CRC16 value, accumulated across states 5 and 6.
 *
 * State 5 stores the high byte; state 6 stores the low byte and
 * then performs the comparison against the locally computed CRC.
 */
static uint16_t rx_crc    = 0;


/**
 * @brief Reset all UART RX parser variables to the idle state.
 *
 * Must be called on every successful packet return, every length
 * overflow (-3), and every CRC failure (-5) so that the next call
 * to @c transport_receive_packet() always begins scanning for a
 * fresh @c ':' delimiter with clean state.
 *
 * Calling this function is equivalent to "the parser has finished
 * with the current frame, good or bad, and is ready for the next one."
 */
static void parser_reset(void)
{
    rx_state  = 0;
    rx_index  = 0;
    rx_length = 0;
    rx_crc    = 0;
}


/**
 * @brief Receive one framed packet from the UART byte stream.
 *
 * Reads one byte per call to @c uart_receive() and advances a
 * seven-state machine.  Because the parser state lives in module-level
 * variables, this function can be called repeatedly — even if
 * @c uart_receive returns 0 (no byte ready) in between — and the
 * parse position is preserved correctly across every call.
 *
 * @par Non-blocking vs blocking uart_receive
 * The function works correctly in both modes:
 *  - @b Blocking: Each @c uart_receive call returns exactly one byte.
 *    The function processes it and loops.  It returns only when a
 *    complete frame has been assembled.  The main loop is blocked
 *    during this time, which is acceptable for a simple bootloader.
 *  - @b Non-blocking (VMIN=0, VTIME=1): @c uart_receive may return 0.
 *    In that case @c transport_receive_packet returns 0 immediately,
 *    the main loop can perform other work, and on the next call the
 *    parser resumes from exactly where it left off.
 *
 * @par CRC failure handling
 * On a CRC mismatch the frame is discarded and the function returns
 * -5.  @c parser_reset() is called first so the next call starts
 * scanning for a new @c ':' rather than trying to continue the
 * corrupted frame.  The host protocol is stop-and-wait so a single
 * lost response causes the host to retransmit the last command.
 *
 * @param packet  Output: populated with the received packet on success.
 *
 * @return Payload byte count (>= 0) on success.
 * @retval  0  No byte was available from @c uart_receive this call
 *             (non-blocking mode) — call again on the next loop tick.
 * @retval -1  @p packet is NULL.
 * @retval -3  Payload length field exceeds @c COMM_MAX_DATA — frame
 *             discarded, parser reset.
 * @retval -5  CRC16 mismatch — frame discarded, parser reset.
 */
int transport_receive_packet(comm_packet_t *packet)
{
    if (!packet)
        return -1;

    uint8_t byte;

    /* Read one byte; if nothing is available yet, return 0 so the
       main bootloader loop can do other work (timeout handling etc.). */
    while (uart_receive(&byte, 1, UART_BUFFER) == 1)
    {
        switch (rx_state)
        {
            /* --------------------------------------------------------
               State 0: hunt for the ':' start-of-frame delimiter.
               Every byte that is not ':' is silently discarded — this
               is the self-healing re-synchronisation mechanism that
               keeps the parser robust against line noise and partial
               frames left over from power-on glitches.
            -------------------------------------------------------- */
            case 0:
                if (byte == (uint8_t)':')
                    rx_state = 1;
                break;

            /* --------------------------------------------------------
               State 1: capture the command byte.
            -------------------------------------------------------- */
            case 1:
                packet->command = byte;
                rx_state = 2;
                break;

            /* --------------------------------------------------------
               State 2: capture LEN_H (high byte of payload length).
            -------------------------------------------------------- */
            case 2:
                rx_length = (uint16_t)((uint16_t)byte << 8);
                rx_state  = 3;
                break;

            /* --------------------------------------------------------
               State 3: capture LEN_L (low byte of payload length).
               Validate immediately so we never try to receive more
               bytes than fit in packet->data[].
               Skip state 4 entirely for zero-length frames — jump
               straight to CRC_H (state 5).
            -------------------------------------------------------- */
            case 3:
                rx_length |= (uint16_t)byte;

                if (rx_length > (uint16_t)COMM_MAX_DATA)
                {
                    /* Payload too large — cannot fit in receive buffer */
                    parser_reset();
                    return -3;
                }

                packet->length = rx_length;
                rx_index       = 0;

                /* Zero-length payload → skip data phase */
                rx_state = (rx_length == 0u) ? 5u : 4u;
                break;

            /* --------------------------------------------------------
               State 4: accumulate payload bytes until rx_length bytes
               have been stored, then advance to the CRC phase.
            -------------------------------------------------------- */
            case 4:
                packet->data[rx_index++] = byte;

                if (rx_index == rx_length)
                    rx_state = 5;
                break;

            /* --------------------------------------------------------
               State 5: capture CRC high byte.
            -------------------------------------------------------- */
            case 5:
                rx_crc   = (uint16_t)((uint16_t)byte << 8);
                rx_state = 6;
                break;

            /* --------------------------------------------------------
               State 6: capture CRC low byte, then validate the frame.

               Build the same byte sequence that transport_send_packet()
               fed to crc16_ccitt() when it transmitted this frame:
               CMD + LEN_H + LEN_L + DATA.  If the CRCs match the
               packet is returned to the caller; otherwise it is
               discarded and -5 is returned.

               In both cases parser_reset() is called before the return
               so the next call starts cleanly.
            -------------------------------------------------------- */
            case 6:
            {
                rx_crc |= (uint16_t)byte;

                /* Save the received CRC into a local before resetting parser state.
                   parser_reset() will zero rx_crc, so we must capture it now.
                   This is the same pattern used for pkt_len below. */
                uint16_t received_crc = rx_crc;

                /* Build CRC verification input buffer */
                uint8_t  crc_buf[COMM_MAX_DATA + 3u];
                uint16_t crc_pos = 0u;

                crc_buf[crc_pos++] = packet->command;
                crc_buf[crc_pos++] = (uint8_t)((packet->length >> 8) & 0xFFu);
                crc_buf[crc_pos++] = (uint8_t)( packet->length        & 0xFFu);

                if (packet->length > 0u)
                {
                    memcpy(&crc_buf[crc_pos], packet->data, packet->length);
                    crc_pos += packet->length;
                }

                uint16_t crc_calc = crc16_ccitt(crc_buf, crc_pos);
                uint16_t pkt_len  = packet->length;

                /* Reset parser state NOW — safe because we captured rx_crc above.
                   From this point on, all module-level parser variables are zero
                   and the next call to transport_receive_packet() starts clean
                   regardless of which path we take below. */
                parser_reset();

                if (crc_calc != received_crc)
                {
                    /* CRC mismatch — frame discarded, parser already reset */
                    return -5;
                }

                /* Valid packet — return payload byte count to caller */
                return (int)pkt_len;
            }

            default:
                /* Unreachable under normal operation — reset to be safe */
                parser_reset();
            break;
        }
    }

    /* uart_receive returned 0: no byte was available this call.
       Return 0 so the caller can yield and retry on the next tick. */
    return 0;
}


/**
 * @brief Encode one packet into a wire frame and transmit it over UART.
 *
 * Builds the complete frame in a local buffer:
 * @code
 *   ':' | CMD | LEN_H | LEN_L | DATA[0..N-1] | CRC_H | CRC_L
 * @endcode
 * then calls @c uart_transmit() once with the whole frame so there
 * are no inter-byte gaps that could confuse the host parser.
 *
 * The CRC covers @c CMD + @c LEN_H + @c LEN_L + @c DATA, i.e.
 * @c buffer[1] through @c buffer[3 + N - 1].  The @c ':' at
 * @c buffer[0] is @b not included in the CRC, matching the host.
 *
 * @param packet  Packet to transmit.  Must not be NULL.
 *
 * @return Number of bytes handed to @c uart_transmit() on success,
 *         or -1 if @p packet is NULL or the payload is too large.
 */
int transport_send_packet(comm_packet_t *packet)
{
    if (!packet || packet->length > (uint16_t)COMM_MAX_DATA)
        return -1;

    /* Frame: ':' + CMD + LEN_H + LEN_L + DATA(N) + CRC_H + CRC_L */
    uint8_t  buffer[COMM_MAX_DATA + 6u];

    /* ---- Build frame header --------------------------------------- */
    buffer[0] = ':';
    buffer[1] = packet->command;
    buffer[2] = (uint8_t)((packet->length >> 8) & 0xFFu);  /* LEN_H */
    buffer[3] = (uint8_t)( packet->length        & 0xFFu); /* LEN_L */

    /* ---- Copy payload -------------------------------------------- */
    if (packet->length > 0u)
        memcpy(&buffer[4], packet->data, packet->length);

    /* ---- Compute CRC over CMD + LEN_H + LEN_L + DATA ------------- */
    /* buffer[1] is CMD; the CRC covers (length + 3) bytes.          */
    uint16_t crc = crc16_ccitt(&buffer[1], (uint16_t)(packet->length + 3u));

    /* ---- Append CRC bytes ---------------------------------------- */
    buffer[4u + packet->length] = (uint8_t)((crc >> 8) & 0xFFu);  /* CRC_H */
    buffer[5u + packet->length] = (uint8_t)( crc        & 0xFFu); /* CRC_L */

    /* ---- Transmit the complete frame in one shot ----------------- */
    return uart_transmit(buffer, (uint16_t)(packet->length + 6u));
}

#endif /* BL_INTERFACE_UART */


/* ====================================================================
   CAN implementation
   ==================================================================== */

#if (BOOT_INTERFACE == BL_INTERFACE_CAN)

#include "drv_can.h"

/**
 * @brief Receive one Re-BOOT protocol packet from the CAN bus.
 *
 * Pops the oldest frame from the ISR ring buffer (filled by
 * drv_cb_can_rx_frame() from the CAN RX interrupt) and maps it
 * directly to a comm_packet_t using the CAN native fields:
 *
 * @code
 *  ┌────────────┬─────┬──────────────────┐
 *  │  CAN_ID    │ DLC │  DATA[0..DLC-1]  │
 *  └────────────┴─────┴──────────────────┘
 *   = CMD          = length  = payload bytes
 * @endcode
 *
 * No software CRC is performed.  CAN bus hardware provides a 15-bit
 * hardware CRC on every frame; a corrupted frame is never delivered
 * to the receive FIFO.
 *
 * @param packet  Output: populated on success.  Must not be NULL.
 *
 * @return Payload byte count (DLC) on success.
 * @retval  0  Ring buffer empty — retry next tick.
 * @retval -1  NULL packet, driver error, or DLC > COMM_MAX_DATA.
 */
int transport_receive_packet(comm_packet_t *packet)
{
    if (!packet)
        return -1;

    can_frame_t frame;

    /* Pop one frame from the ISR-fed ring buffer */
    int ret = can_receive(&frame, CAN_BUFFER);

    if (ret <= 0)
    {
        /* 0 = buffer empty, <0 = driver error — caller retries */
        return ret;
    }

    /* ---- Map CAN native fields to comm_packet_t ------------------- */

    /*
     * CAN_ID  → command
     * The CAN identifier carries the Re-BOOT command code directly.
     * This eliminates the CMD byte from the data field and allows
     * hardware acceptance filters to pass only bootloader frames.
     */
    packet->command = (uint8_t)(frame.id & 0xFFu);

    /*
     * DLC  → payload length
     * DLC is always 0–8 for classic CAN, so no range check against
     * a large COMM_MAX_DATA is strictly needed, but we guard anyway
     * to be safe against a misconfigured driver returning DLC > 8.
     */
    if (frame.len > (uint8_t)COMM_MAX_DATA)
        return -1;

    packet->length = (uint16_t)frame.len;

    /*
     * data[0..DLC-1]  → payload bytes
     * Raw bytes, no overhead, no CRC.  DLC = 0 is valid (command with
     * no payload, e.g. CMD_RESET_REQ, CMD_START_APP).
     */
    if (packet->length > 0u)
        memcpy(packet->data, frame.data, packet->length);

    return (int)packet->length;
}


/**
 * @brief Encode and transmit one Re-BOOT protocol packet over CAN.
 *
 * Maps the comm_packet_t directly onto CAN native fields:
 *
 * @code
 *  ┌────────────┬─────┬──────────────────┐
 *  │  CAN_ID    │ DLC │  DATA[0..DLC-1]  │
 *  └────────────┴─────┴──────────────────┘
 *   = CMD          = length  = payload bytes
 * @endcode
 *
 * No software CRC bytes are appended.  CAN hardware generates and
 * validates its own 15-bit CRC transparently.
 *
 * @note Maximum payload per frame is 8 bytes (classic CAN DLC limit).
 *
 * @param packet  Must not be NULL; payload must be 0–8 bytes.
 *
 * @return Number of CAN data bytes transmitted on success, -1 on error.
 */
int transport_send_packet(comm_packet_t *packet)
{
    /* Classic CAN: DLC max 8 */
    if (!packet || packet->length > 8u)
        return -1;

    can_frame_t frame;

    /*
     * CMD  → CAN_ID
     * Use the command code as the CAN identifier so any node on the
     * bus can filter bootloader traffic by ID range.
     */
    frame.id = (uint32_t)packet->command;

    /*
     * length  → DLC
     */
    frame.len = (uint8_t)packet->length;

    /*
     * payload bytes  → data[0..DLC-1]
     * No CMD byte, no LEN bytes, no CRC bytes in the data field.
     */
    if (packet->length > 0u)
        memcpy(frame.data, packet->data, packet->length);

    return can_transmit(frame.id, frame.data, frame.len);
}

#endif /* BL_INTERFACE_CAN */
