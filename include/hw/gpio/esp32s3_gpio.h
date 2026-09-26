#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "chardev/char-fe.h"
#include "esp32_gpio.h"

#define TYPE_ESP32S3_GPIO "esp32s3.gpio"
#define ESP32S3_GPIO(obj)           OBJECT_CHECK(ESP32S3GPIOState, (obj), TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3GPIOClass, obj, TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3GPIOClass, klass, TYPE_ESP32S3_GPIO)

/* Bootstrap options for ESP32-S3 (4-bit) */
#define ESP32S3_STRAP_MODE_FLASH_BOOT 0x4   /* SPI Boot */

#define ESP32S3_GPIO_PINS 49

/* Interrupt registers (the rest of the map is in esp32_gpio.h) */
REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004C)
REG32(GPIO_STATUS1, 0x0050)
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)
REG32(GPIO_PCPU_INT, 0x005C)
REG32(GPIO_PCPU_NMI_INT, 0x0060)
REG32(GPIO_CPUSDIO_INT, 0x0064)
REG32(GPIO_PCPU_INT1, 0x0068)
REG32(GPIO_PCPU_NMI_INT1, 0x006C)
REG32(GPIO_CPUSDIO_INT1, 0x0070)
REG32(GPIO_PIN0, 0x0074)
    FIELD(GPIO_PIN0, INT_TYPE, 7, 3)
    FIELD(GPIO_PIN0, INT_ENA, 13, 5)
REG32(GPIO_STATUS_NEXT, 0x014C)
REG32(GPIO_STATUS_NEXT1, 0x0150)

/* GPIO_PINn INT_ENA bits */
#define ESP32S3_GPIO_INT_ENA_CPU  BIT(0)
#define ESP32S3_GPIO_INT_ENA_NMI  BIT(1)

/* Sysbus IRQs: 0 = GPIO interrupt (ETS_GPIO_INTR_SOURCE), 1 = GPIO NMI (ETS_GPIO_NMI_SOURCE) */
#define ESP32S3_GPIO_IRQ_INT 0
#define ESP32S3_GPIO_IRQ_NMI 1

typedef struct ESP32S3State {
    Esp32GpioState parent;
    qemu_irq nmi_irq;
    uint32_t pin[ESP32S3_GPIO_PINS];  /* GPIO_PINn_REG: interrupt type and enables */
    uint64_t status;                  /* interrupt status, bit n = GPIOn */
    uint64_t pads;                    /* pad levels at the last evaluation, for edge detection */
    /* Pad change trace: JSON lines {"t_ns", "pin", "level"} on the chardev "gpio-trace" for the pins in
     * trace-mask (virtual time, so the timing is the firmware's) */
    CharBackend trace;
    bool has_trace;
    uint64_t trace_mask;
} ESP32S3GPIOState;

typedef struct ESP32S3GPIOClass {
    Esp32GpioClass parent;
    ResettablePhases parent_phases;
} ESP32S3GPIOClass;
