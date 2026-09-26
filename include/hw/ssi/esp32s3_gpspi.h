/*
 * ESP32-S3 general-purpose SPI master (GPSPI2 / GPSPI3), CPU-controlled transfers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32S3_GPSPI "esp32s3.gpspi"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32S3GpSpiState, ESP32S3_GPSPI)

#define ESP32S3_GPSPI_IO_SIZE   0x1000

struct ESP32S3GpSpiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    SSIBus *bus;
    qemu_irq irq;
    uint32_t regs[ESP32S3_GPSPI_IO_SIZE / 4];
};
