/*
 * Register-file peripheral stub: every register reads back the last value written to it.
 *
 * Enough for drivers that configure a peripheral and read the configuration back (e.g. LEDC
 * computing a timer frequency from its divider), and it lets host tools inspect what the
 * firmware programmed. It has no behaviour of its own.
 */
#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP_REGFILE "esp.regfile"
OBJECT_DECLARE_SIMPLE_TYPE(EspRegFileState, ESP_REGFILE)

struct EspRegFileState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t size;      /* bytes, property "size" */
    uint32_t *regs;
};

/* Create a register file of `size` bytes named `name` and map it at `addr` in `mr`. */
EspRegFileState *esp_regfile_create(MemoryRegion *mr, const char *name, hwaddr addr, uint32_t size);
