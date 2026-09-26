/*
 * ESP32-S3 I2C master controller (I2C0 / I2C1).
 *
 * Registers read back what was written. DATA (0x1C) pushes into the 32-byte TX FIFO on write and
 * pops the RX FIFO on read. Setting CTR.TRANS_START runs the command list COMD0..COMD7 at once:
 *   RSTART  - (repeated) start; the next byte written is the address byte (addr << 1 | read)
 *   WRITE n - n bytes from the TX FIFO (address byte first after a start, then data)
 *   READ n  - n bytes from the device into the RX FIFO
 *   STOP    - ends the transfer: TRANS_COMPLETE interrupt
 *   END     - pauses: END_DETECT interrupt; the next TRANS_START runs from COMD0 again
 * Each executed command gets its DONE bit. An address or data byte that is not acknowledged (with
 * ACK checking enabled) aborts the list with the NACK interrupt.
 *
 * The devices on the bus are modelled outside QEMU (Python, see espemu): each bus phase is sent
 * to the chardev "i2c<N>-bridge" as a small binary request and the vCPU waits for the reply, so
 * virtual time stands still while the model answers:
 *   request  [op, bus, a, b] (+ payload)     reply
 *   'S' start  a = 7-bit address, b = read    1 byte: 1 = ACK, 0 = NACK
 *   'W' write  a = n, payload n bytes         1 byte: number of bytes ACKed
 *   'R' read   a = n                          n bytes
 *   'P' stop                                  1 byte (ignored)
 * Without a bridge (or if it goes away) every address NACKs at once.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/i2c/esp32s3_i2c.h"

#define R_CTR           (0x04 / 4)
#define   CTR_TRANS_START   BIT(5)
#define   CTR_FSM_RST       BIT(10)
#define   CTR_CONF_UPGATE   BIT(11)
#define R_SR            (0x08 / 4)
#define   SR_RESP_REC       BIT(0)
#define R_FIFO_CONF     (0x18 / 4)
#define   FIFO_RX_RST       BIT(12)
#define   FIFO_TX_RST       BIT(13)
#define R_DATA          (0x1C / 4)
#define R_INT_RAW       (0x20 / 4)
#define R_INT_CLR       (0x24 / 4)
#define R_INT_ENA       (0x28 / 4)
#define R_INT_STATUS    (0x2C / 4)
#define   INT_END_DETECT    BIT(3)
#define   INT_TRANS_COMPLETE BIT(7)
#define   INT_NACK          BIT(10)
#define R_COMD0         (0x58 / 4)
#define   CMD_BYTE_NUM(c)   ((c) & 0xFF)
#define   CMD_ACK_CHECK     BIT(8)
#define   CMD_OPCODE(c)     (((c) >> 11) & 7)
#define   CMD_DONE          BIT(31)
#define R_SCL_SP_CONF   (0x80 / 4)
#define   SCL_RST_SLV_EN    BIT(0)

enum { OP_WRITE = 1, OP_STOP = 2, OP_READ = 3, OP_END = 4, OP_RSTART = 6 };

static void esp32s3_i2c_update_irq(ESP32S3I2CState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_INT_RAW] & s->regs[R_INT_ENA]));
}

/* ---- bridge to the device models --------------------------------------------------------- */

static bool esp32s3_i2c_bridge_call(ESP32S3I2CState *s, uint8_t op, uint8_t a, uint8_t b,
                                    const uint8_t *payload, int payload_len,
                                    uint8_t *reply, int reply_len)
{
    if (!s->has_bridge || !qemu_chr_fe_backend_connected(&s->bridge)) {
        return false;
    }
    uint8_t hdr[4] = { op, s->index, a, b };
    if (qemu_chr_fe_write_all(&s->bridge, hdr, sizeof(hdr)) != sizeof(hdr) ||
        (payload_len && qemu_chr_fe_write_all(&s->bridge, payload, payload_len) != payload_len) ||
        qemu_chr_fe_read_all(&s->bridge, reply, reply_len) != reply_len) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32s3.i2c%u: bridge I/O failed\n", s->index);
        return false;
    }
    return true;
}

static bool esp32s3_i2c_dev_start(ESP32S3I2CState *s, uint8_t addr, bool read)
{
    uint8_t ack = 0;
    return esp32s3_i2c_bridge_call(s, 'S', addr, read, NULL, 0, &ack, 1) && ack == 1;
}

static int esp32s3_i2c_dev_write(ESP32S3I2CState *s, const uint8_t *data, int len)
{
    uint8_t acked = 0;
    return esp32s3_i2c_bridge_call(s, 'W', len, 0, data, len, &acked, 1) ? acked : 0;
}

static void esp32s3_i2c_dev_read(ESP32S3I2CState *s, uint8_t *data, int len)
{
    if (!esp32s3_i2c_bridge_call(s, 'R', len, 0, NULL, 0, data, len)) {
        memset(data, 0xFF, len);    /* released bus */
    }
}

static void esp32s3_i2c_dev_stop(ESP32S3I2CState *s)
{
    uint8_t dummy;
    esp32s3_i2c_bridge_call(s, 'P', 0, 0, NULL, 0, &dummy, 1);
}

/* ---- FIFOs ------------------------------------------------------------------------------- */

static uint8_t esp32s3_i2c_tx_pop(ESP32S3I2CState *s)
{
    if (s->tx_count == 0) {
        return 0xFF;
    }
    uint8_t v = s->txfifo[s->tx_head];
    s->tx_head = (s->tx_head + 1) % ESP32S3_I2C_FIFO_LEN;
    s->tx_count--;
    return v;
}

static void esp32s3_i2c_rx_push(ESP32S3I2CState *s, uint8_t v)
{
    if (s->rx_count < ESP32S3_I2C_FIFO_LEN) {
        s->rxfifo[(s->rx_head + s->rx_count) % ESP32S3_I2C_FIFO_LEN] = v;
        s->rx_count++;
    }
}

/* ---- command execution ------------------------------------------------------------------- */

/* Finish the command list: `irq` is TRANS_COMPLETE, END_DETECT or NACK. */
static void esp32s3_i2c_finish(ESP32S3I2CState *s, int cmd, uint32_t irq)
{
    s->regs[R_COMD0 + cmd] |= CMD_DONE;
    s->cmd_index = 0;
    s->regs[R_INT_RAW] |= irq;
    esp32s3_i2c_update_irq(s);
}

static void esp32s3_i2c_nack(ESP32S3I2CState *s, int cmd)
{
    if (s->selected) {
        esp32s3_i2c_dev_stop(s);
        s->selected = false;
    }
    s->regs[R_SR] |= SR_RESP_REC;
    esp32s3_i2c_finish(s, cmd, INT_NACK);
}

static void esp32s3_i2c_run(ESP32S3I2CState *s)
{
    for (int i = s->cmd_index; i < 8; i++) {
        uint32_t cmd = s->regs[R_COMD0 + i];
        int n = CMD_BYTE_NUM(cmd);
        bool ack_check = cmd & CMD_ACK_CHECK;

        switch (CMD_OPCODE(cmd)) {
        case OP_RSTART:
            s->expect_addr = true;
            break;

        case OP_WRITE: {
            uint8_t data[256];
            int len = 0;
            for (int k = 0; k < n; k++) {
                uint8_t b = esp32s3_i2c_tx_pop(s);
                if (s->expect_addr) {
                    s->expect_addr = false;
                    s->selected = esp32s3_i2c_dev_start(s, b >> 1, b & 1);
                    s->regs[R_SR] = (s->regs[R_SR] & ~SR_RESP_REC) | (s->selected ? 0 : SR_RESP_REC);
                    if (!s->selected && ack_check) {
                        esp32s3_i2c_nack(s, i);
                        return;
                    }
                } else {
                    data[len++] = b;
                }
            }
            if (len) {
                int acked = s->selected ? esp32s3_i2c_dev_write(s, data, len) : 0;
                if (acked < len && ack_check) {
                    esp32s3_i2c_nack(s, i);
                    return;
                }
            }
            break;
        }

        case OP_READ: {
            uint8_t data[256];
            if (s->selected) {
                esp32s3_i2c_dev_read(s, data, n);
            } else {
                memset(data, 0xFF, n);
            }
            for (int k = 0; k < n; k++) {
                esp32s3_i2c_rx_push(s, data[k]);
            }
            break;
        }

        case OP_STOP:
            if (s->selected) {
                esp32s3_i2c_dev_stop(s);
                s->selected = false;
            }
            s->expect_addr = false;
            esp32s3_i2c_finish(s, i, INT_TRANS_COMPLETE);
            return;

        case OP_END:
            esp32s3_i2c_finish(s, i, INT_END_DETECT);
            return;

        default:
            break;
        }
        s->regs[R_COMD0 + i] |= CMD_DONE;
    }
    s->cmd_index = 0;
}

/* ---- registers --------------------------------------------------------------------------- */

static uint64_t esp32s3_i2c_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);
    switch (addr / 4) {
    case R_SR:
        return (s->regs[R_SR] & SR_RESP_REC) | (s->rx_count << 8) | (s->tx_count << 18);
    case R_DATA: {
        if (s->rx_count == 0) {
            return 0;
        }
        uint8_t v = s->rxfifo[s->rx_head];
        s->rx_head = (s->rx_head + 1) % ESP32S3_I2C_FIFO_LEN;
        s->rx_count--;
        return v;
    }
    case R_INT_STATUS:
        return s->regs[R_INT_RAW] & s->regs[R_INT_ENA];
    default:
        return s->regs[addr / 4];
    }
}

static void esp32s3_i2c_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);
    switch (addr / 4) {
    case R_CTR:
        /* TRANS_START, CONF_UPGATE and FSM_RST take effect at once and read back as 0 */
        s->regs[R_CTR] = value & ~(CTR_TRANS_START | CTR_CONF_UPGATE | CTR_FSM_RST);
        if (value & CTR_FSM_RST) {
            s->cmd_index = 0;
            s->expect_addr = false;
            if (s->selected) {
                esp32s3_i2c_dev_stop(s);
                s->selected = false;
            }
        }
        if (value & CTR_TRANS_START) {
            if (s->cmd_index == 0) {
                s->expect_addr = !s->selected;  /* a list begins with RSTART anyway */
            }
            esp32s3_i2c_run(s);
        }
        break;
    case R_FIFO_CONF:
        s->regs[R_FIFO_CONF] = value;
        if (value & FIFO_TX_RST) {
            s->tx_head = s->tx_count = 0;
        }
        if (value & FIFO_RX_RST) {
            s->rx_head = s->rx_count = 0;
        }
        break;
    case R_DATA:
        if (s->tx_count < ESP32S3_I2C_FIFO_LEN) {
            s->txfifo[(s->tx_head + s->tx_count) % ESP32S3_I2C_FIFO_LEN] = value;
            s->tx_count++;
        }
        break;
    case R_INT_CLR:
        s->regs[R_INT_RAW] &= ~value;
        esp32s3_i2c_update_irq(s);
        break;
    case R_INT_ENA:
        s->regs[R_INT_ENA] = value;
        esp32s3_i2c_update_irq(s);
        break;
    case R_SCL_SP_CONF:
        s->regs[R_SCL_SP_CONF] = value & ~SCL_RST_SLV_EN;  /* bus clear completes at once */
        break;
    default:
        s->regs[addr / 4] = value;
        break;
    }
}

static const MemoryRegionOps esp32s3_i2c_ops = {
    .read = esp32s3_i2c_read,
    .write = esp32s3_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_i2c_reset_hold(Object *obj, ResetType type)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);
    memset(s->regs, 0, sizeof(s->regs));
    s->tx_head = s->tx_count = s->rx_head = s->rx_count = 0;
    s->cmd_index = 0;
    s->expect_addr = false;
    s->selected = false;
    qemu_set_irq(s->irq, 0);
}

static void esp32s3_i2c_realize(DeviceState *dev, Error **errp)
{
    ESP32S3I2CState *s = ESP32S3_I2C(dev);
    g_autofree char *name = g_strdup_printf("i2c%u-bridge", s->index);
    Chardev *chr = qemu_chr_find(name);
    if (chr) {
        s->has_bridge = qemu_chr_fe_init(&s->bridge, chr, errp);
    }
}

static void esp32s3_i2c_init(Object *obj)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &esp32s3_i2c_ops, s, TYPE_ESP32S3_I2C, ESP32S3_I2C_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32s3_i2c_properties[] = {
    DEFINE_PROP_UINT32("index", ESP32S3I2CState, index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->realize = esp32s3_i2c_realize;
    rc->phases.hold = esp32s3_i2c_reset_hold;
    device_class_set_props(dc, esp32s3_i2c_properties);
}

static const TypeInfo esp32s3_i2c_info = {
    .name = TYPE_ESP32S3_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3I2CState),
    .instance_init = esp32s3_i2c_init,
    .class_init = esp32s3_i2c_class_init,
};

static void esp32s3_i2c_register_types(void)
{
    type_register_static(&esp32s3_i2c_info);
}

type_init(esp32s3_i2c_register_types)
