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

Place `reboot_config.h` on your compiler's include path. Below is the actual configuration used for the STM32F411 reference target:

```c
/*
 * reboot_config.h
 */

#ifndef INC_REBOOT_CONFIG_H_
#define INC_REBOOT_CONFIG_H_

/* ---- Interface selection ------------------------------------------- */
#define BL_INTERFACE_UART   1
#define BL_INTERFACE_CAN    2
#define BL_INTERFACE_TCP    3

#define BOOT_INTERFACE      BL_INTERFACE_UART   /* change to CAN or TCP as needed */

/* ---- RX ring buffer sizes ------------------------------------------ */
#define UART_RX_BUFFER_SIZE 512
#define CAN_RX_BUFFER_SIZE  512

/* ---- Flash / protocol parameters ----------------------------------- */
#define APP_START_ADDRESS   (0x08008000U)   /* Sector 2 on STM32F411 */
#define FLASH_SECTOR_SIZE   (256)           /* bytes per pipeline sector */
#define COMM_SEGMENT_SIZE   (8)             /* bytes per CMD_PIPELINE_DATA payload */

#endif /* INC_REBOOT_CONFIG_H_ */
```

> **Note:** `FLASH_SECTOR_SIZE` is the pipeline transfer unit, not the physical erase sector size.
> The physical sector size on the STM32F411 is 16 KB; each erase call covers the full physical sector.

---

## Integration Example (STM32CubeIDE / HAL)

The following is the complete `Core/Src/main.c` from the reference STM32F411 project.
Add source files from Re-BOOT-Firmware to your STM32CubeIDE project and drop this user code
into the generated `main.c` skeleton.

### Step 1 — Includes and private variables

```c
/* USER CODE BEGIN Includes */
#include "booloader.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
volatile uint8_t uart_rx_byte;
/* USER CODE END PV */
```

### Step 2 — Function prototypes

```c
/* USER CODE BEGIN PFP */
void    mcu_uart_init(void);
void    mcu_uart_close(void);
int32_t mcu_uart_tx(const uint8_t *data, int16_t len);
int32_t mcu_uart_rx(uint8_t *data, uint16_t maxlen);

static int32_t mcu_flash_erase(uint32_t address, uint32_t size);
static int32_t mcu_flash_write(uint32_t address, const uint8_t *data, uint32_t size);
static int32_t mcu_flash_read (uint32_t address, uint8_t *buf, uint32_t size);
static int32_t addr_to_sector (uint32_t address, uint32_t *sector);
static void    mcu_jump_to_application(void);
/* USER CODE END PFP */
```

### Step 3 — Driver ops structs (before main)

```c
/* USER CODE BEGIN 0 */
uart_ops_t uart_ops =
{
    .init     = mcu_uart_init,
    .close    = mcu_uart_close,
    .transmit = mcu_uart_tx,
    .receive  = mcu_uart_rx
};

static const flash_ops_t flash_ops =
{
    .erase = mcu_flash_erase,
    .write = mcu_flash_write,
    .read  = mcu_flash_read,
};
/* USER CODE END 0 */
```

### Step 4 — Register and run (inside main)

```c
/* USER CODE BEGIN 2 */
bl_uart_driver_register(&uart_ops);
bl_flash_driver_register(&flash_ops);
bl_delay_driver_register(HAL_Delay);
bl_jump_driver_register(mcu_jump_to_application);
bootloader_exe();   /* never returns */
/* USER CODE END 2 */
```

### Step 5 — UART driver implementation

```c
/* USER CODE BEGIN 4 */

void mcu_uart_init(void)
{
    /* Peripheral already initialised by MX_USART2_UART_Init().
       Start the first byte-level RX interrupt so the ring buffer
       begins accumulating data immediately.                        */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_byte, 1);
}

void mcu_uart_close(void)
{
    HAL_UART_AbortReceive_IT(&huart2);
    HAL_UART_DeInit(&huart2);
}

int32_t mcu_uart_tx(const uint8_t *data, int16_t len)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY) == HAL_OK)
        return len;
    return -1;
}

int32_t mcu_uart_rx(uint8_t *data, uint16_t maxlen)
{
    if (HAL_UART_Receive(&huart2, data, maxlen, HAL_MAX_DELAY) == HAL_OK)
        return maxlen;
    return -1;
}

/* UART RX complete callback — called from HAL UART ISR.
   Push each byte into the Re-BOOT ring buffer, then restart
   the interrupt for the next byte.                            */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    drv_cb_uart_rx_byte(uart_rx_byte);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_byte, 1);
}
```

### Step 6 — Flash driver implementation

```c
/* Helper: translate an absolute flash address to an STM32F4 sector number.
 *
 * STM32F411 sector map:
 *   Sector 0 : 0x0800_0000 – 0x0800_3FFF   16 KB  (bootloader)
 *   Sector 1 : 0x0800_4000 – 0x0800_7FFF   16 KB  (flag byte at 0x0800_7FFF)
 *   Sector 2 : 0x0800_8000 – 0x0800_BFFF   16 KB  (application start)
 *   Sector 3 : 0x0800_C000 – 0x0800_FFFF   16 KB
 *   Sector 4 : 0x0801_0000 – 0x0801_FFFF   64 KB
 *   Sector 5 : 0x0802_0000 – 0x0803_FFFF  128 KB
 *   Sector 6 : 0x0804_0000 – 0x0805_FFFF  128 KB
 *   Sector 7 : 0x0806_0000 – 0x0807_FFFF  128 KB
 */
static int32_t addr_to_sector(uint32_t address, uint32_t *sector)
{
    if      (address < 0x08004000u) { *sector = FLASH_SECTOR_0; }
    else if (address < 0x08008000u) { *sector = FLASH_SECTOR_1; }
    else if (address < 0x0800C000u) { *sector = FLASH_SECTOR_2; }
    else if (address < 0x08010000u) { *sector = FLASH_SECTOR_3; }
    else if (address < 0x08020000u) { *sector = FLASH_SECTOR_4; }
    else if (address < 0x08040000u) { *sector = FLASH_SECTOR_5; }
    else if (address < 0x08060000u) { *sector = FLASH_SECTOR_6; }
    else if (address < 0x08080000u) { *sector = FLASH_SECTOR_7; }
    else                            { return -1; }
    return 0;
}

static int32_t mcu_flash_erase(uint32_t address, uint32_t size)
{
    (void)size;  /* STM32F4 erases full sectors only */

    uint32_t sector = 0u;
    if (addr_to_sector(address, &sector) != 0)
        return -1;

    if (HAL_FLASH_Unlock() != HAL_OK)
        return -1;

    FLASH_EraseInitTypeDef erase_cfg =
    {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,   /* 2.7 V – 3.6 V supply */
        .Sector       = sector,
        .NbSectors    = 1u,
    };

    uint32_t sector_error = 0u;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    HAL_FLASH_Lock();

    return (status == HAL_OK) ? 0 : -1;
}

static int32_t mcu_flash_write(uint32_t address,
                                const uint8_t *data,
                                uint32_t size)
{
    if (HAL_FLASH_Unlock() != HAL_OK)
        return -1;

    HAL_StatusTypeDef status = HAL_OK;

    /* Single-byte path — used for the validity flag at APP_START_ADDRESS-1
       which is not word-aligned.  FLASH_TYPEPROGRAM_BYTE handles any address. */
    if (size == 1u)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                                   address, (uint64_t)data[0]);
        HAL_FLASH_Lock();
        return (status == HAL_OK) ? 0 : -1;
    }

    /* Multi-byte word-aligned path (normal sector programming).
       Packs 4 bytes per word, padding the final partial word with 0xFF. */
    uint32_t i = 0u;
    while ((i < size) && (status == HAL_OK))
    {
        uint32_t word       = 0xFFFFFFFFu;
        uint32_t bytes_left = size - i;
        uint32_t chunk      = (bytes_left >= 4u) ? 4u : bytes_left;

        for (uint32_t b = 0u; b < chunk; b++)
            word = (word & ~(0xFFu << (b * 8u))) |
                   ((uint32_t)data[i + b] << (b * 8u));

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   (uint32_t)(address + i), (uint64_t)word);
        i += 4u;
    }

    HAL_FLASH_Lock();
    return (status == HAL_OK) ? 0 : -1;
}

static int32_t mcu_flash_read(uint32_t address, uint8_t *buf, uint32_t size)
{
    if (buf == NULL)
        return -1;
    /* Internal flash is memory-mapped — direct memcpy is correct */
    memcpy(buf, (const void *)address, size);
    return 0;
}
```

### Step 7 — Application jump implementation

```c
/* All CMSIS register access is isolated here so bootloader.c has zero
   MCU dependencies.  Sequence:
     1. Disable global IRQs
     2. Stop SysTick
     3. Clear NVIC ICER / ICPR (all 8 registers → 240 IRQs)
     4. Relocate VTOR to APP_START_ADDRESS
     5. Load application MSP from its vector table
     6. Re-enable IRQs
     7. Branch to application Reset_Handler              */
static void mcu_jump_to_application(void)
{
    __disable_irq();

    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL  = 0u;

    for (uint8_t i = 0u; i < 8u; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    SCB->VTOR = (uint32_t)APP_START_ADDRESS;
    __set_MSP(*((volatile uint32_t *)APP_START_ADDRESS));
    __enable_irq();

    void (*app_reset_handler)(void) =
        (void (*)(void))(*((volatile uint32_t *)(APP_START_ADDRESS + 4u)));

    app_reset_handler();
    while (1) {}   /* never reached */
}

/* USER CODE END 4 */
```

---

## License

Re-BOOT-Firmware is free software distributed under the **GNU General Public License v3.0**.
See [LICENSE](LICENSE) or <https://www.gnu.org/licenses/> for details.

Author: Subhajit Roy — subhajitroy005@gmail.com
