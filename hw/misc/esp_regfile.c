/*
 * Register-file peripheral stub: every register reads back the last value written to it.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp_regfile.h"

/* Any access width works (firmware often updates register bitfields with 8/16-bit stores):
 * only the bytes actually written change. */
static uint64_t esp_regfile_read(void *opaque, hwaddr addr, unsigned int size)
{
    EspRegFileState *s = ESP_REGFILE(opaque);
    return ldn_le_p((uint8_t *)s->regs + addr, size);
}

static void esp_regfile_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    EspRegFileState *s = ESP_REGFILE(opaque);
    stn_le_p((uint8_t *)s->regs + addr, size, value);
}

static const MemoryRegionOps esp_regfile_ops = {
    .read = esp_regfile_read,
    .write = esp_regfile_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp_regfile_realize(DeviceState *dev, Error **errp)
{
    EspRegFileState *s = ESP_REGFILE(dev);

    if (s->size == 0 || s->size % 4) {
        error_setg(errp, "esp.regfile: size must be a non-zero multiple of 4");
        return;
    }
    s->regs = g_new0(uint32_t, s->size / 4);
    memory_region_init_io(&s->iomem, OBJECT(dev), &esp_regfile_ops, s, TYPE_ESP_REGFILE, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void esp_regfile_reset_hold(Object *obj, ResetType type)
{
    EspRegFileState *s = ESP_REGFILE(obj);
    if (s->regs) {
        memset(s->regs, 0, s->size);
    }
}

static Property esp_regfile_properties[] = {
    DEFINE_PROP_UINT32("size", EspRegFileState, size, 0x1000),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp_regfile_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp_regfile_realize;
    rc->phases.hold = esp_regfile_reset_hold;
    device_class_set_props(dc, esp_regfile_properties);
}

static const TypeInfo esp_regfile_info = {
    .name = TYPE_ESP_REGFILE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(EspRegFileState),
    .class_init = esp_regfile_class_init,
};

static void esp_regfile_register_types(void)
{
    type_register_static(&esp_regfile_info);
}

type_init(esp_regfile_register_types)

EspRegFileState *esp_regfile_create(MemoryRegion *mr, const char *name, hwaddr addr, uint32_t size)
{
    DeviceState *dev = qdev_new(TYPE_ESP_REGFILE);
    qdev_prop_set_uint32(dev, "size", size);
    object_property_add_child(qdev_get_machine(), name, OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    /* Above the SoC's catch-all peripheral I/O region (priority 0), which would shadow it */
    memory_region_add_subregion_overlap(mr, addr, sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0), 1);
    return ESP_REGFILE(dev);
}
