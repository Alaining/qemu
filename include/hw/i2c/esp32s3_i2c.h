/*
 * ESP32-S3 I2C master controller (I2C0 / I2C1).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/sysbus.h"
#include "chardev/char-fe.h"

#define TYPE_ESP32S3_I2C "esp32s3.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32S3I2CState, ESP32S3_I2C)

#define ESP32S3_I2C_IO_SIZE     0x1000
#define ESP32S3_I2C_FIFO_LEN    32

struct ESP32S3I2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    /* Bus index (0/1). Devices live outside QEMU: if a chardev "i2c<index>-bridge" exists, bus
     * phases are forwarded to it (see esp32s3_i2c.c); otherwise every address NACKs. */
    uint32_t index;
    CharBackend bridge;
    bool has_bridge;

    uint32_t regs[ESP32S3_I2C_IO_SIZE / 4];
    uint8_t txfifo[ESP32S3_I2C_FIFO_LEN], rxfifo[ESP32S3_I2C_FIFO_LEN];
    int tx_head, tx_count, rx_head, rx_count;

    int cmd_index;          /* next command to run (0 after STOP/END/NACK) */
    bool expect_addr;       /* after RSTART, the next byte written is the address byte */
    bool selected;          /* a device ACKed its address and the bus is between START and STOP */
};
