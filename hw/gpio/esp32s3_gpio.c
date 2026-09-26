/*
 * ESP32-S3 GPIO emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "chardev/char.h"
#include "hw/gpio/esp32s3_gpio.h"

#define PINS_MASK ((1ULL << ESP32S3_GPIO_PINS) - 1)

/* Pins whose interrupt is enabled towards the CPU, or as NMI */
static uint64_t esp32s3_gpio_enabled(ESP32S3GPIOState *s, bool nmi)
{
    uint32_t want = nmi ? ESP32S3_GPIO_INT_ENA_NMI : ESP32S3_GPIO_INT_ENA_CPU;
    uint64_t mask = 0;
    for (int i = 0; i < ESP32S3_GPIO_PINS; i++) {
        if (FIELD_EX32(s->pin[i], GPIO_PIN0, INT_ENA) & want) {
            mask |= 1ULL << i;
        }
    }
    return mask;
}

static void esp32s3_gpio_trace(ESP32S3GPIOState *s, uint64_t changed, uint64_t pads)
{
    changed &= s->trace_mask;
    if (!changed || !s->has_trace || !qemu_chr_fe_backend_connected(&s->trace)) {
        return;
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (int i = 0; i < 64; i++) {
        if (changed & (1ULL << i)) {
            char line[96];
            int n = snprintf(line, sizeof(line), "{\"t_ns\": %" PRId64 ", \"pin\": %d, \"level\": %d}\n",
                             now, i, (int)((pads >> i) & 1));
            qemu_chr_fe_write_all(&s->trace, (const uint8_t *)line, n);
        }
    }
}

/* Pads may have changed: latch edge/level events into the status and update the interrupt lines.
 * A level-triggered pin keeps setting its status bit while the level lasts, so clearing it in the
 * ISR doesn't end the interrupt until the level goes away, as on the chip. */
static void esp32s3_gpio_update(Esp32GpioState *base)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(base);
    uint64_t pads = esp32_gpio_pads(base);
    uint64_t changed = pads ^ s->pads;

    esp32s3_gpio_trace(s, changed, pads);
    for (int i = 0; i < ESP32S3_GPIO_PINS; i++) {
        uint64_t bit = 1ULL << i;
        bool level = pads & bit;
        bool fire;
        switch (FIELD_EX32(s->pin[i], GPIO_PIN0, INT_TYPE)) {
        case 1: fire = (changed & bit) && level; break;    /* rising edge */
        case 2: fire = (changed & bit) && !level; break;   /* falling edge */
        case 3: fire = changed & bit; break;               /* any edge */
        case 4: fire = !level; break;                      /* low level */
        case 5: fire = level; break;                       /* high level */
        default: fire = false; break;
        }
        if (fire) {
            s->status |= bit;
        }
    }
    s->pads = pads;
    qemu_set_irq(base->irq, !!(s->status & esp32s3_gpio_enabled(s, false)));
    qemu_set_irq(s->nmi_irq, !!(s->status & esp32s3_gpio_enabled(s, true)));
}

static bool esp32s3_gpio_read_ext(Esp32GpioState *base, hwaddr addr, uint64_t *value)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(base);
    switch (addr) {
    case A_GPIO_STATUS:
    case A_GPIO_STATUS_NEXT:
        *value = (uint32_t)s->status;
        return true;
    case A_GPIO_STATUS1:
    case A_GPIO_STATUS_NEXT1:
        *value = s->status >> 32;
        return true;
    case A_GPIO_PCPU_INT:
        *value = (uint32_t)(s->status & esp32s3_gpio_enabled(s, false));
        return true;
    case A_GPIO_PCPU_INT1:
        *value = (s->status & esp32s3_gpio_enabled(s, false)) >> 32;
        return true;
    case A_GPIO_PCPU_NMI_INT:
        *value = (uint32_t)(s->status & esp32s3_gpio_enabled(s, true));
        return true;
    case A_GPIO_PCPU_NMI_INT1:
        *value = (s->status & esp32s3_gpio_enabled(s, true)) >> 32;
        return true;
    }
    if (addr >= A_GPIO_PIN0 && addr < A_GPIO_PIN0 + 4 * ESP32S3_GPIO_PINS) {
        *value = s->pin[(addr - A_GPIO_PIN0) / 4];
        return true;
    }
    return false;
}

static bool esp32s3_gpio_write_ext(Esp32GpioState *base, hwaddr addr, uint64_t value)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(base);
    uint64_t v = (uint32_t)value;
    switch (addr) {
    case A_GPIO_STATUS:
        s->status = ((s->status & ~0xFFFFFFFFULL) | v) & PINS_MASK;
        return true;
    case A_GPIO_STATUS_W1TS:
        s->status = (s->status | v) & PINS_MASK;
        return true;
    case A_GPIO_STATUS_W1TC:
        s->status &= ~v;
        return true;
    case A_GPIO_STATUS1:
        s->status = ((s->status & 0xFFFFFFFFULL) | (v << 32)) & PINS_MASK;
        return true;
    case A_GPIO_STATUS1_W1TS:
        s->status = (s->status | (v << 32)) & PINS_MASK;
        return true;
    case A_GPIO_STATUS1_W1TC:
        s->status &= ~(v << 32);
        return true;
    }
    if (addr >= A_GPIO_PIN0 && addr < A_GPIO_PIN0 + 4 * ESP32S3_GPIO_PINS) {
        s->pin[(addr - A_GPIO_PIN0) / 4] = value;
        return true;
    }
    return false;
}

static void esp32s3_gpio_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GPIOClass *k = ESP32S3_GPIO_GET_CLASS(obj);
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);

    if (k->parent_phases.hold) {
        k->parent_phases.hold(obj, type);
    }
    memset(s->pin, 0, sizeof(s->pin));
    s->status = 0;
    s->pads = esp32_gpio_pads(&s->parent);
    qemu_set_irq(s->parent.irq, 0);
    qemu_set_irq(s->nmi_irq, 0);
}

static void esp32s3_gpio_realize(DeviceState *dev, Error **errp)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(dev);
    Chardev *chr = qemu_chr_find("gpio-trace");
    if (chr) {
        s->has_trace = qemu_chr_fe_init(&s->trace, chr, errp);
    }
}

static void esp32s3_gpio_init(Object *obj)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);

    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32S3_STRAP_MODE_FLASH_BOOT, &error_fatal);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->nmi_irq);
    object_property_add_uint64_ptr(obj, "status", &s->status, OBJ_PROP_FLAG_READ);
}

static Property esp32s3_gpio_properties[] = {
    DEFINE_PROP_UINT64("trace-mask", ESP32S3GPIOState, trace_mask, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32GpioClass *gc = ESP32_GPIO_CLASS(klass);
    ESP32S3GPIOClass *k = ESP32S3_GPIO_CLASS(klass);

    gc->read_ext = esp32s3_gpio_read_ext;
    gc->write_ext = esp32s3_gpio_write_ext;
    gc->pads_changed = esp32s3_gpio_update;
    resettable_class_set_parent_phases(rc, NULL, esp32s3_gpio_reset_hold, NULL, &k->parent_phases);
    dc->realize = esp32s3_gpio_realize;
    device_class_set_props(dc, esp32s3_gpio_properties);
}

static const TypeInfo esp32s3_gpio_info = {
    .name = TYPE_ESP32S3_GPIO,
    .parent = TYPE_ESP32_GPIO,
    .instance_size = sizeof(ESP32S3GPIOState),
    .instance_init = esp32s3_gpio_init,
    .class_init = esp32s3_gpio_class_init,
    .class_size = sizeof(ESP32S3GPIOClass),
};

static void esp32s3_gpio_register_types(void)
{
    type_register_static(&esp32s3_gpio_info);
}

type_init(esp32s3_gpio_register_types)
