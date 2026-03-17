# Re-BOOT-Firmware

**Target-side firmware library for the Re-BOOT bootloader protocol.**

Re-BOOT-Firmware is the embedded C library that runs on the microcontroller target. It implements the receive-and-respond loop, sector-by-sector flash programming, CRC verification, and application jump — all through a fully portable, HAL-independent driver abstraction. The host counterpart is the [Re-BOOT](https://github.com/subhajitroy005/Re-BOOT) PC tool.

---

## Table of Contents

- [Overview](#overview)
- [Flash Memory Layout (STM32F411)](#flash-memory-layout-stm32f411)
- [Boot Sequence](#boot-sequence)
- [Protocol Command Reference](#protocol-command-reference)
- [Repository Structure](#repository-structure)
- [Driver Abstraction Layer](#driver-abstraction-layer)
- [Porting to a New MCU](#porting-to-a-new-mcu)
- [Configuration](#configuration)
- [Integration Example (STM32CubeIDE / HAL)](#integration-example-stm32cubeide--hal)
- [License](#license)

---

## Overview

```
┌─────────────────────────────────────────────────────┐
│                    HOST PC                          │
│   Re-BOOT tool  ──►  UART / CAN  ──►  target MCU   │
└─────────────────────────────────────────────────────┘
                                         │
                               Re-BOOT-Firmware
                               ┌─────────────────────┐
                               │  bootloader_exe()   │
                               │  ├─ cmd processor   │
                               │  ├─ sector buffer   │
                               │  ├─ CRC32 verify    │
                               │  └─ app jump        │
                               ├─────────────────────┤
                               │  transport layer    │
                               │  (UART / CAN)       │
                               ├─────────────────────┤
                               │  driver abstraction │
                               │  (flash/uart/can/   │
                               │   delay/jump)       │
                               └─────────────────────┘
```

Key properties:

- **MCU-agnostic** — zero HAL or CMSIS includes in the library. All hardware operations are supplied by the application via registered callbacks.
- **Single-sector pipeline** — receives, buffers, CRC-checks, and flashes one sector at a time. Host sends the next sector only after CRC ACK.
- **Firmware-validity flag** — one byte at `APP_START_ADDRESS - 1` controls the boot decision without modifying the application image.
- **Non-blocking polling** — UART/CAN ISR feeds a ring buffer; the main loop polls without stalling.
- **Proactive target announcement** — broadcasts `RESP_TARGET_INFO` immediately on power-on so the host does not need to send `CMD_RESET_REQ`.

---

## Flash Memory Layout (STM32F411)

```
┌─────────────────────────────────────────────────────────┐
│  Address        │  Size   │  Content                    │
├─────────────────┼─────────┼─────────────────────────────┤
│  0x08000000     │  16 KB  │  Bootloader (Sector 0)      │
├─────────────────┼─────────┼─────────────────────────────┤
│  0x08004000     │  16 KB  │  Reserved for bootloader    │
│                 │         │  (Sector 1)                 │
│  0x08007FFE     │  1 B    │  (unused)                   │
│  0x08007FFF     │  1 B    │  ← APP_VALID_FLAG           │
│                 │         │    0x00 = valid firmware    │
│                 │         │    0xFF = no firmware       │
├─────────────────┼─────────┼─────────────────────────────┤
│  0x08008000     │  ...    │  Application (APP_START)    │
│  (Sector 2+)    │         │                             │
└─────────────────────────────────────────────────────────┘
```

`APP_VALID_FLAG_ADDR = APP_START_ADDRESS - 1 = 0x08007FFF`

---

## Boot Sequence

```
Power-on / Reset
      │
      ▼
Interface init (UART or CAN)
      │
      ▼
Proactive broadcast ── RESP_TARGET_INFO ──► Host
      │
      ▼
Read APP_VALID_FLAG_ADDR (0x08007FFF)
      │
      ├── 0x00 (valid firmware) ──────────────────────┐
      │                                               │
      │   500 ms trigger window                       │
      │   ┌──────────────────────────────────┐        │
      │   │ poll transport_receive_packet()  │        │
      │   │ every 1 ms (500 ticks)           │        │
      │   │                                  │        │
      │   │  packet received? ──► YES ───────┼──┐     │
      │   │                                  │  │     │
      │   │  timeout? ──────────► YES ───────┼──┼─────┼──► bl_app_jump()
      │   └──────────────────────────────────┘  │     │
      │                                         │     │
      └── 0xFF (no firmware) ───────────────────┘     │
                   │                                   │
                   ▼                                   │
         Process received packet                       │
                   │                                   │
                   ▼                                   │
         ┌─────────────────────────────┐               │
         │  Main receive loop  (while) │               │
         │  CMD_RESET_REQ    → RESP_TARGET_INFO        │
         │  CMD_PIPELINE_DATA→ buffer segment          │
         │  CMD_PIPELINE_VERIFY → CRC32 check          │
         │                   → flash_erase+write       │
         │                   → RESP_CRC_ACK/NACK       │
         │  CMD_START_APP    → write flag 0x00         │
         │                   → RESP_APP_JUMP_ACK       │
         │                   → bl_app_jump() ──────────┘
         └─────────────────────────────┘
```

---

## Protocol Command Reference

All packets use the Re-BOOT serial/CAN frame format defined by the transport layer.

| Command | Code | Direction | Response | Payload |
|---|---|---|---|---|
| `CMD_RESET_REQ` | `0x10` | Host → Target | `RESP_TARGET_INFO` | — |
| `CMD_PIPELINE_DATA` | `0x11` | Host → Target | `RESP_SEG_ACK` | `[addr:4B][data:NB]` |
| `CMD_PIPELINE_VERIFY` | `0x13` | Host → Target | `RESP_CRC_ACK` / `RESP_CRC_NACK` | `[crc32:4B]` |
| `CMD_START_APP` | `0x14` | Host → Target | `RESP_APP_JUMP_ACK` | — |

| Response | Code | Payload |
|---|---|---|
| `RESP_TARGET_INFO` | `0x30` | `[APP_START:4B][SECTOR_SIZE:2B][SEG_SIZE:2B]` |
| `RESP_SEG_ACK` | `0x31` | `[1]` |
| `RESP_CRC_ACK` | `0x33` | `[1]` |
| `RESP_CRC_NACK` | `0x34` | — |
| `RESP_APP_JUMP_ACK` | `0x37` | `[1]` |

### CRC32 Algorithm

Polynomial `0xEDB88320` (reflected), initial value `0xFFFFFFFF`, final XOR `0xFFFFFFFF`.
Matches the host-side `pipeline_sector_crc()` function exactly.

---

## Repository Structure

```
Re-BOOT-Firmware/
├── init/
│   └── bootloader.c        # Main bootloader: command loop, sector write, app jump
├── comm/
│   ├── transport.c         # Frame encode/decode (UART or CAN) + send/receive API
│   └── comm_global.c       # Shared comm state
├── driver/
│   ├── drv_uart.c          # UART abstraction + ISR ring buffer
│   ├── drv_can.c           # CAN abstraction + ISR frame ring buffer
│   ├── drv_flash.c         # Flash erase/write/read abstraction
│   ├── drv_delay.c         # Millisecond delay abstraction
│   └── drv_jump.c          # Application jump abstraction
├── include/
│   ├── booloader.h         # Top-level include (aggregates all headers)
│   ├── bl_types.h          # comm_packet_t, stdint, stddef
│   ├── bl_protocol_config.h# CMD_* and RESP_* command codes
│   ├── reboot_config.h     # APP_START_ADDRESS, FLASH_SECTOR_SIZE, interface select
│   ├── drv_uart.h          # uart_ops_t, bl_uart_driver_register(), ISR callback
│   ├── drv_can.h           # can_ops_t, bl_can_driver_register(), ISR callback
│   ├── drv_flash.h         # flash_ops_t, bl_flash_driver_register()
│   ├── drv_delay.h         # delay_ms_fn_t, bl_delay_driver_register()
│   ├── drv_jump.h          # app_jump_fn_t, bl_jump_driver_register()
│   ├── transport.h         # transport_send_packet(), transport_receive_packet()
│   └── global.h            # Project-wide shared definitions
├── utility/                # Optional utility helpers
├── LICENSE                 # GNU GPL v3
└── README.md
```

---

## Driver Abstraction Layer

All MCU-specific operations are injected at runtime through function-pointer structs. The bootloader library contains **no** `stm32f4xx_hal.h`, `stm32f4xx.h`, or any vendor header.

| Driver | Registration function | ops struct | Purpose |
|---|---|---|---|
| Flash | `bl_flash_driver_register()` | `flash_ops_t` | erase / write / read |
| UART | `bl_uart_driver_register()` | `uart_ops_t` | init / transmit / receive |
| CAN | `bl_can_driver_register()` | `can_ops_t` | init / transmit / receive |
| Delay | `bl_delay_driver_register()` | `delay_ms_fn_t` | millisecond delay |
| Jump | `bl_jump_driver_register()` | `app_jump_fn_t` | application jump sequence |

### ISR Callbacks

Wire these to your MCU interrupt handlers:

```c
/* UART RX ISR — push one received byte into the ring buffer */
void drv_cb_uart_rx_byte(uint8_t byte);

/* CAN RX ISR — push one received frame into the frame ring buffer */
void drv_cb_can_rx_frame(const can_frame_t *frame);
```

---

## Porting to a New MCU

1. **Implement flash operations** — erase a sector, write words/bytes, read back.
2. **Implement UART operations** — init at the target baud rate, blocking transmit, byte-level RX ISR that calls `drv_cb_uart_rx_byte()`.
3. **Implement CAN operations** (if using CAN) — init, transmit a frame, frame RX ISR that calls `drv_cb_can_rx_frame()`.
4. **Implement `delay_ms`** — a millisecond-accurate blocking delay (e.g., `HAL_Delay`).
5. **Implement the application jump** — disable IRQs, reset SysTick/NVIC, relocate VTOR, load MSP, branch to reset handler. Keep all CMSIS/vendor code here, away from the library.
6. **Register everything** before calling `bootloader_exe()`.
7. **Set `reboot_config.h`** for your target: `APP_START_ADDRESS`, `FLASH_SECTOR_SIZE`, `COMM_SEGMENT_SIZE`, `BOOT_INTERFACE`.

---

## Configuration

Edit `include/reboot_config.h` (or add it to your project's include path):

```c
/* Application start address — must be sector-aligned */
#define APP_START_ADDRESS    0x08008000UL

/* Flash sector size in bytes */
#define FLASH_SECTOR_SIZE    16384U          /* 16 KB */

/* Max firmware data bytes per CMD_PIPELINE_DATA packet */
#define COMM_SEGMENT_SIZE    64U

/* Communication interface selection */
#define BL_INTERFACE_UART    1
#define BL_INTERFACE_CAN     2
#define BL_INTERFACE_TCP     3

#define BOOT_INTERFACE       BL_INTERFACE_UART

/* RX ring buffer sizes */
#define UART_RX_BUFFER_SIZE  256U
#define CAN_RX_BUFFER_SIZE   32U
```

---

## Integration Example (STM32CubeIDE / HAL)

```c
/* Core/Src/main.c */
#include "booloader.h"

/* ---- 1. MCU-specific implementations -------------------------------- */

static int32_t mcu_flash_erase(uint32_t address, uint32_t size)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef cfg = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = /* derive from address */,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t err;
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&cfg, &err);
    HAL_FLASH_Lock();
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t mcu_flash_write(uint32_t address,
                                const uint8_t *data, uint32_t size)
{
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < size; i += 4)
    {
        uint32_t word = *(uint32_t *)(data + i);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address + i, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }
    /* Single-byte path for non-word-aligned writes (e.g. validity flag) */
    if (size == 1u)
    {
        HAL_StatusTypeDef s =
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address,
                              (uint64_t)data[0]);
        HAL_FLASH_Lock();
        return (s == HAL_OK) ? 0 : -1;
    }
    HAL_FLASH_Lock();
    return 0;
}

static int32_t mcu_flash_read(uint32_t address,
                               uint8_t *buf, uint32_t size)
{
    memcpy(buf, (void *)address, size);
    return 0;
}

static void mcu_jump_to_application(void)
{
    __disable_irq();
    SysTick->CTRL = 0u; SysTick->LOAD = 0u; SysTick->VAL = 0u;
    for (uint8_t i = 0; i < 8u; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SCB->VTOR = (uint32_t)APP_START_ADDRESS;
    __set_MSP(*((volatile uint32_t *)APP_START_ADDRESS));
    __enable_irq();
    void (*reset_handler)(void) =
        (void (*)(void))(*((volatile uint32_t *)(APP_START_ADDRESS + 4u)));
    reset_handler();
    while (1) {}
}

/* ---- 2. UART RX interrupt — feed the ring buffer -------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    drv_cb_uart_rx_byte(rx_byte);
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

/* ---- 3. Register drivers and run ------------------------------------ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    static const flash_ops_t flash_ops = {
        .erase = mcu_flash_erase,
        .write = mcu_flash_write,
        .read  = mcu_flash_read,
    };
    static const uart_ops_t uart_ops = {
        .init     = mcu_uart_init,
        .close    = mcu_uart_close,
        .transmit = mcu_uart_transmit,
        .receive  = mcu_uart_receive,
    };

    bl_flash_driver_register(&flash_ops);
    bl_uart_driver_register(&uart_ops);
    bl_delay_driver_register(HAL_Delay);
    bl_jump_driver_register(mcu_jump_to_application);

    bootloader_exe();   /* never returns */
}
```

---

## License

Re-BOOT-Firmware is free software distributed under the **GNU General Public License v3.0**.
See [LICENSE](LICENSE) or <https://www.gnu.org/licenses/> for details.

Author: Subhajit Roy — subhajitroy005@gmail.com
