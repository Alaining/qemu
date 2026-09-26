/*
 * ILI9341 240x320 TFT controller on an SPI bus (4-wire: the D/C line is a GPIO).
 *
 * Decodes the MIPI DCS commands drivers use to draw: SWRESET, SLPIN/SLPOUT, DISPON/DISPOFF,
 * INVON/INVOFF, MADCTL (rotation/mirroring/BGR), COLMOD, CASET, PASET, RAMWR, RAMWRC. Pixels are
 * RGB565, high byte first. The image is shown as the physical portrait panel, oriented the way
 * common ILI9341 modules look with Adafruit's rotation 0 (MADCTL = MX | BGR). Parameters of
 * other commands are accepted and ignored; reads return 0.
 *
 * Board wiring comes from properties: "dc-gpio" (required; -1 disables the display) and an
 * optional "cs-gpio" (a GPIO chip select: bytes are ignored while it is high).
 * The read-only QOM property "pixels" counts pixels written, so tools can tell whether the
 * firmware drives this display.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/gpio/esp32_gpio.h"
#include "ui/console.h"

#define TYPE_ILI9341 "ili9341"
OBJECT_DECLARE_SIMPLE_TYPE(Ili9341State, ILI9341)

#define WIDTH   240
#define HEIGHT  320

#define MADCTL_MY   0x80
#define MADCTL_MX   0x40
#define MADCTL_MV   0x20
#define MADCTL_BGR  0x08

struct Ili9341State {
    SSIPeripheral parent_obj;
    QemuConsole *con;
    Esp32GpioState *gpio;
    int32_t dc_gpio, cs_gpio;

    uint8_t cmd;            /* command whose parameters/data are being received */
    uint32_t nparam;        /* parameter bytes received for it */
    uint8_t param[4];
    uint16_t col_start, col_end, page_start, page_end;
    uint16_t col, page;     /* RAM write position */
    uint8_t madctl;
    bool pixel_high_done;   /* first byte of an RGB565 pixel received */
    uint8_t pixel_high;
    bool display_on, inverted;
    uint32_t pixels;        /* pixels written, for tools ("pixels" property) */
    uint32_t *fb;           /* physical panel, x8r8g8b8 */
    bool dirty;
};

static void ili9341_reset_state(Ili9341State *s)
{
    s->cmd = 0;
    s->nparam = 0;
    s->col_start = 0;
    s->col_end = WIDTH - 1;
    s->page_start = 0;
    s->page_end = HEIGHT - 1;
    s->col = s->page = 0;
    s->madctl = 0;
    s->pixel_high_done = false;
    s->display_on = false;
    s->inverted = false;
    s->dirty = true;
}

/* Frame-memory position (host column/page) -> pixel on the viewed portrait panel */
static uint32_t *ili9341_pixel_at(Ili9341State *s, int col, int page)
{
    int mem_col = col, mem_row = page;
    if (s->madctl & MADCTL_MV) {
        mem_col = page;
        mem_row = col;
    }
    if (s->madctl & MADCTL_MX) {
        mem_col = WIDTH - 1 - mem_col;
    }
    if (s->madctl & MADCTL_MY) {
        mem_row = HEIGHT - 1 - mem_row;
    }
    /* Modules are mounted mirrored: MX set is the upright portrait image */
    int x = WIDTH - 1 - mem_col, y = mem_row;
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
        return NULL;
    }
    return &s->fb[y * WIDTH + x];
}

static void ili9341_put_pixel(Ili9341State *s, uint16_t rgb565)
{
    uint32_t r = (rgb565 >> 11) & 0x1F, g = (rgb565 >> 5) & 0x3F, b = rgb565 & 0x1F;
    if (!(s->madctl & MADCTL_BGR)) {  /* the panel's subpixels are BGR */
        uint32_t t = r;
        r = b;
        b = t;
    }
    uint32_t px = (r << 3 | r >> 2) << 16 | (g << 2 | g >> 4) << 8 | (b << 3 | b >> 2);
    if (s->inverted) {
        px ^= 0xFFFFFF;
    }
    uint32_t *dst = ili9341_pixel_at(s, s->col, s->page);
    if (dst) {
        *dst = px;
        s->dirty = true;
    }
    s->pixels++;
    /* Advance within the CASET/PASET window, wrapping to the next page */
    if (s->col < s->col_end) {
        s->col++;
    } else {
        s->col = s->col_start;
        s->page = s->page < s->page_end ? s->page + 1 : s->page_start;
    }
}

static void ili9341_command(Ili9341State *s, uint8_t cmd)
{
    s->cmd = cmd;
    s->nparam = 0;
    s->pixel_high_done = false;
    switch (cmd) {
    case 0x01:  /* SWRESET */
        ili9341_reset_state(s);
        break;
    case 0x20:  /* INVOFF */
    case 0x21:  /* INVON */
        s->inverted = cmd == 0x21;
        s->dirty = true;
        break;
    case 0x28:  /* DISPOFF */
    case 0x29:  /* DISPON */
        s->display_on = cmd == 0x29;
        s->dirty = true;
        break;
    case 0x2C:  /* RAMWR: restart at the window origin */
        s->col = s->col_start;
        s->page = s->page_start;
        break;
    default:    /* RAMWRC (0x3C) continues where RAMWR stopped; others need nothing */
        break;
    }
}

static void ili9341_data(Ili9341State *s, uint8_t v)
{
    switch (s->cmd) {
    case 0x2A:  /* CASET: start and end column, 16 bits each */
    case 0x2B:  /* PASET */
        if (s->nparam < 4) {
            s->param[s->nparam] = v;
        }
        if (++s->nparam == 4) {
            uint16_t start = s->param[0] << 8 | s->param[1];
            uint16_t end = s->param[2] << 8 | s->param[3];
            if (s->cmd == 0x2A) {
                s->col_start = start;
                s->col_end = end;
            } else {
                s->page_start = start;
                s->page_end = end;
            }
        }
        break;
    case 0x36:  /* MADCTL */
        if (s->nparam++ == 0) {
            s->madctl = v;
        }
        break;
    case 0x2C:  /* RAMWR */
    case 0x3C:  /* RAMWRC */
        if (!s->pixel_high_done) {
            s->pixel_high = v;
            s->pixel_high_done = true;
        } else {
            ili9341_put_pixel(s, s->pixel_high << 8 | v);
            s->pixel_high_done = false;
        }
        break;
    default:
        s->nparam++;
        break;
    }
}

static uint32_t ili9341_transfer(SSIPeripheral *dev, uint32_t data)
{
    Ili9341State *s = ILI9341(dev);
    if (!s->gpio || s->dc_gpio < 0) {
        return 0;
    }
    if (s->cs_gpio >= 0 && esp32_gpio_level(s->gpio, s->cs_gpio)) {
        return 0;   /* not selected */
    }
    if (esp32_gpio_level(s->gpio, s->dc_gpio)) {
        ili9341_data(s, data);
    } else {
        ili9341_command(s, data);
    }
    return 0;
}

static void ili9341_update_display(void *opaque)
{
    Ili9341State *s = ILI9341(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    if (!s->dirty || !surface) {
        return;
    }
    uint32_t *dest = surface_data(surface);
    if (s->display_on) {
        memcpy(dest, s->fb, WIDTH * HEIGHT * 4);
    } else {
        memset(dest, 0, WIDTH * HEIGHT * 4);   /* DISPOFF (and before DISPON): black */
    }
    dpy_gfx_update(s->con, 0, 0, WIDTH, HEIGHT);
    s->dirty = false;
}

static void ili9341_invalidate(void *opaque)
{
    ILI9341(opaque)->dirty = true;
}

static const GraphicHwOps ili9341_ops = {
    .invalidate = ili9341_invalidate,
    .gfx_update = ili9341_update_display,
};

static void ili9341_realize(SSIPeripheral *d, Error **errp)
{
    Ili9341State *s = ILI9341(d);
    s->fb = g_new0(uint32_t, WIDTH * HEIGHT);
    s->con = graphic_console_init(DEVICE(d), 0, &ili9341_ops, s);
    qemu_console_resize(s->con, WIDTH, HEIGHT);
    ili9341_reset_state(s);
}

static void ili9341_init(Object *obj)
{
    Ili9341State *s = ILI9341(obj);
    object_property_add_uint32_ptr(obj, "pixels", &s->pixels, OBJ_PROP_FLAG_READ);
}

static Property ili9341_properties[] = {
    DEFINE_PROP_LINK("gpio", Ili9341State, gpio, TYPE_ESP32_GPIO, Esp32GpioState *),
    DEFINE_PROP_INT32("dc-gpio", Ili9341State, dc_gpio, -1),
    DEFINE_PROP_INT32("cs-gpio", Ili9341State, cs_gpio, -1),
    DEFINE_PROP_END_OF_LIST(),
};

static void ili9341_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = ili9341_realize;
    k->transfer = ili9341_transfer;
    k->cs_polarity = SSI_CS_NONE;   /* selection is by GPIO (cs-gpio), sampled per byte */
    device_class_set_props(dc, ili9341_properties);
}

static const TypeInfo ili9341_info = {
    .name = TYPE_ILI9341,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(Ili9341State),
    .instance_init = ili9341_init,
    .class_init = ili9341_class_init,
};

static void ili9341_register_types(void)
{
    type_register_static(&ili9341_info);
}

type_init(ili9341_register_types)
