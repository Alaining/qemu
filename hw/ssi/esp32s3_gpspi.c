/*
 * ESP32-S3 general-purpose SPI master (GPSPI2 / GPSPI3), CPU-controlled transfers.
 *
 * Every register reads back what was written. Setting CMD.USR runs one user transaction on the
 * SSI bus: optional command and address phases, then the data phase from/to the W0..W15 buffer
 * (up to 64 bytes, full duplex: bytes clocked in replace the bytes sent). The transaction then
 * completes at once: CMD.USR clears and DMA_INT_RAW.TRANS_DONE is set (and the interrupt raised
 * if enabled). This covers Arduino's SPI HAL and ESP-IDF's spi_master without DMA. DMA (GDMA)
 * transfers are not modelled yet.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32s3_gpspi.h"

#define R_CMD           (0x00 / 4)
#define   CMD_UPDATE      BIT(23)
#define   CMD_USR         BIT(24)
#define R_ADDR          (0x04 / 4)
#define R_CTRL          (0x08 / 4)
#define   CTRL_WR_BIT_ORDER BIT(25)   /* 1 = LSB first */
#define R_USER          (0x10 / 4)
#define   USER_DOUTDIN    BIT(0)
#define   USER_MISO_HIGHPART BIT(24)
#define   USER_MOSI_HIGHPART BIT(25)
#define   USER_USR_MOSI   BIT(27)
#define   USER_USR_MISO   BIT(28)
#define   USER_USR_ADDR   BIT(30)
#define   USER_USR_COMMAND BIT(31)
#define R_USER1         (0x14 / 4)    /* [31:27] address bit length - 1 */
#define R_USER2         (0x18 / 4)    /* [31:28] command bit length - 1, [15:0] command value */
#define R_MS_DLEN       (0x1C / 4)    /* [17:0] data bit length - 1 */
#define R_DMA_INT_ENA   (0x34 / 4)
#define R_DMA_INT_CLR   (0x38 / 4)
#define R_DMA_INT_RAW   (0x3C / 4)
#define R_DMA_INT_ST    (0x40 / 4)
#define R_DMA_INT_SET   (0x44 / 4)
#define   INT_TRANS_DONE  BIT(12)
#define R_W0            (0x98 / 4)
#define W_BYTES         64

static uint8_t bit_reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    return (b & 0xAA) >> 1 | (b & 0x55) << 1;
}

static uint8_t esp32s3_gpspi_xfer(ESP32S3GpSpiState *s, uint8_t out)
{
    bool lsb_first = s->regs[R_CTRL] & CTRL_WR_BIT_ORDER;
    uint8_t in = ssi_transfer(s->bus, lsb_first ? bit_reverse(out) : out);
    return lsb_first ? bit_reverse(in) : in;
}

static void esp32s3_gpspi_update_irq(ESP32S3GpSpiState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_DMA_INT_RAW] & s->regs[R_DMA_INT_ENA]));
}

static void esp32s3_gpspi_transaction(ESP32S3GpSpiState *s)
{
    uint32_t user = s->regs[R_USER];
    uint8_t *w = (uint8_t *)&s->regs[R_W0];

    if (user & USER_USR_COMMAND) {
        /* Command phase: bytes of the command value, least significant byte first (drivers
         * pre-swap multi-byte commands for MSB-first order, as for the data buffer). */
        int bytes = ((s->regs[R_USER2] >> 28) + 1 + 7) / 8;
        for (int i = 0; i < bytes; i++) {
            esp32s3_gpspi_xfer(s, s->regs[R_USER2] >> (8 * i));
        }
    }
    if (user & USER_USR_ADDR) {
        /* Address phase: the address is left-aligned in ADDR, sent most significant bit first */
        int bits = (s->regs[R_USER1] >> 27) + 1;
        for (int i = 0; i < (bits + 7) / 8; i++) {
            esp32s3_gpspi_xfer(s, s->regs[R_ADDR] >> (24 - 8 * i));
        }
    }
    if (user & (USER_USR_MOSI | USER_USR_MISO | USER_DOUTDIN)) {
        int bytes = MIN((int)(((s->regs[R_MS_DLEN] & 0x3FFFF) + 1 + 7) / 8), W_BYTES);
        int tx_base = (user & USER_MOSI_HIGHPART) ? 32 : 0;
        int rx_base = (user & USER_MISO_HIGHPART) ? 32 : 0;
        bool mosi = user & (USER_USR_MOSI | USER_DOUTDIN);
        bool miso = user & (USER_USR_MISO | USER_DOUTDIN);
        for (int i = 0; i < bytes; i++) {
            uint8_t out = mosi ? w[(tx_base + i) % W_BYTES] : 0xFF;
            uint8_t in = esp32s3_gpspi_xfer(s, out);
            if (miso) {
                w[(rx_base + i) % W_BYTES] = in;
            }
        }
    }
    s->regs[R_DMA_INT_RAW] |= INT_TRANS_DONE;
    esp32s3_gpspi_update_irq(s);
}

static uint64_t esp32s3_gpspi_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(opaque);
    if (addr / 4 == R_DMA_INT_ST) {
        return s->regs[R_DMA_INT_RAW] & s->regs[R_DMA_INT_ENA];
    }
    return s->regs[addr / 4];
}

static void esp32s3_gpspi_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(opaque);
    switch (addr / 4) {
    case R_CMD:
        /* UPDATE (sync config to the SPI clock domain) and USR (start) complete instantly */
        s->regs[R_CMD] = value & ~(CMD_UPDATE | CMD_USR);
        if (value & CMD_USR) {
            esp32s3_gpspi_transaction(s);
        }
        break;
    case R_DMA_INT_CLR:
        s->regs[R_DMA_INT_RAW] &= ~value;
        esp32s3_gpspi_update_irq(s);
        break;
    case R_DMA_INT_SET:
        s->regs[R_DMA_INT_RAW] |= value;
        esp32s3_gpspi_update_irq(s);
        break;
    case R_DMA_INT_ENA:
        s->regs[R_DMA_INT_ENA] = value;
        esp32s3_gpspi_update_irq(s);
        break;
    default:
        s->regs[addr / 4] = value;
        break;
    }
}

static const MemoryRegionOps esp32s3_gpspi_ops = {
    .read = esp32s3_gpspi_read,
    .write = esp32s3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
}

static void esp32s3_gpspi_init(Object *obj)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpspi_ops, s, TYPE_ESP32S3_GPSPI,
                          ESP32S3_GPSPI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = ssi_create_bus(DEVICE(s), "spi");
}

static void esp32s3_gpspi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_gpspi_reset_hold;
}

static const TypeInfo esp32s3_gpspi_info = {
    .name = TYPE_ESP32S3_GPSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GpSpiState),
    .instance_init = esp32s3_gpspi_init,
    .class_init = esp32s3_gpspi_class_init,
};

static void esp32s3_gpspi_register_types(void)
{
    type_register_static(&esp32s3_gpspi_info);
}

type_init(esp32s3_gpspi_register_types)
