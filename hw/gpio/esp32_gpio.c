/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qapi/visitor.h"
#include "hw/gpio/esp32_gpio.h"

static void esp32_gpio_pads_changed(Esp32GpioState *s)
{
    Esp32GpioClass *k = ESP32_GPIO_GET_CLASS(s);
    if (k->pads_changed) {
        k->pads_changed(s);
    }
}


static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_GPIO_OUT:
        r = s->out;
        break;
    case A_GPIO_OUT1:
        r = s->out1;
        break;
    case A_GPIO_ENABLE:
        r = s->enable;
        break;
    case A_GPIO_ENABLE1:
        r = s->enable1;
        break;
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;
    /* A pin configured as output reads back its own level, like the real pad does */
    case A_GPIO_IN:
        r = (s->out & s->enable) | (s->ext_in & ~s->enable);
        break;
    case A_GPIO_IN1:
        r = (s->out1 & s->enable1) | (s->ext_in1 & ~s->enable1);
        break;

    default: {
        Esp32GpioClass *k = ESP32_GPIO_GET_CLASS(s);
        if (k->read_ext) {
            k->read_ext(s, addr, &r);
        }
        break;
    }
    }
    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    switch (addr) {
    case A_GPIO_OUT:
        s->out = value;
        break;
    case A_GPIO_OUT_W1TS:
        s->out |= value;
        break;
    case A_GPIO_OUT_W1TC:
        s->out &= ~value;
        break;
    case A_GPIO_OUT1:
        s->out1 = value;
        break;
    case A_GPIO_OUT1_W1TS:
        s->out1 |= value;
        break;
    case A_GPIO_OUT1_W1TC:
        s->out1 &= ~value;
        break;
    case A_GPIO_ENABLE:
        s->enable = value;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->enable |= value;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->enable &= ~value;
        break;
    case A_GPIO_ENABLE1:
        s->enable1 = value;
        break;
    case A_GPIO_ENABLE1_W1TS:
        s->enable1 |= value;
        break;
    case A_GPIO_ENABLE1_W1TC:
        s->enable1 &= ~value;
        break;

    default: {
        Esp32GpioClass *k = ESP32_GPIO_GET_CLASS(s);
        if (k->write_ext) {
            k->write_ext(s, addr, value);
        }
        break;
    }
    }
    esp32_gpio_pads_changed(s);
}

/* Levels applied from outside: "in"/"in1" hold GPIO0-31/32+; writing "pin-high"/"pin-low" with a pin
 * number changes that one pin atomically (several host tools may drive different pins at once) */
static void esp32_gpio_get_in(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    uint32_t *field = opaque;
    uint32_t value = *field;
    visit_type_uint32(v, name, &value, errp);
}

static void esp32_gpio_set_in(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    uint32_t value;
    if (visit_type_uint32(v, name, &value, errp)) {
        *(uint32_t *)opaque = value;
        esp32_gpio_pads_changed(ESP32_GPIO(obj));
    }
}

static void esp32_gpio_set_pin(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    uint32_t pin;
    if (!visit_type_uint32(v, name, &pin, errp)) {
        return;
    }
    if (pin >= 64) {
        error_setg(errp, "GPIO%u does not exist", pin);
        return;
    }
    uint32_t *field = pin < 32 ? &s->ext_in : &s->ext_in1;
    uint32_t bit = 1u << (pin % 32);
    *field = opaque ? (*field | bit) : (*field & ~bit);
    esp32_gpio_pads_changed(s);
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);

    /* Guest-side state resets; levels applied from outside stay as they are */
    s->out = s->out1 = 0;
    s->enable = s->enable1 = 0;
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);

    /* Pin state for host tools: read with qom-get, drive inputs with qom-set on "in"/"in1" */
    object_property_add_uint32_ptr(obj, "out", &s->out, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "out1", &s->out1, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "enable", &s->enable, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "enable1", &s->enable1, OBJ_PROP_FLAG_READ);
    object_property_add(obj, "in", "uint32", esp32_gpio_get_in, esp32_gpio_set_in, NULL, &s->ext_in);
    object_property_add(obj, "in1", "uint32", esp32_gpio_get_in, esp32_gpio_set_in, NULL, &s->ext_in1);
    object_property_add(obj, "pin-high", "uint32", NULL, esp32_gpio_set_pin, NULL, (void *)1);
    object_property_add(obj, "pin-low", "uint32", NULL, esp32_gpio_set_pin, NULL, NULL);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
