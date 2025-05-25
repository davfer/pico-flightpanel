// DVI
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "sprite.h"

// DVDD 1.2V (1.1V seems ok too)
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

struct dvi_inst dvi0;
uint16_t framebuf[FRAME_WIDTH * FRAME_HEIGHT];

// USB
#include <stdio.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"

// I2C
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
    BLINK_FAIL = 100,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
    BLINK_ALWAYS_ON = UINT32_MAX,
    BLINK_ALWAYS_OFF = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void core0_upkeep(void);
void core1_upkeep(void);

//--------------------------------------------------------------------+

void core1_main()
{
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    dvi_scanbuf_main_16bpp(&dvi0);
    __builtin_unreachable();
}

void core1_scanline_callback()
{
    // Discard any scanline pointers passed back
    uint16_t *bufptr;
    while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
        core1_upkeep();
    // // Note first two scanlines are pushed before DVI start
    static uint scanline = 2;
    bufptr = &framebuf[FRAME_WIDTH * scanline];
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    scanline = (scanline + 1) % FRAME_HEIGHT;
}


int main()
{
    // DVI
    vreg_set_voltage(VREG_VSEL);
    sleep_ms(10);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

    setup_default_uart();

    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi0.scanline_callback = core1_scanline_callback;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // Once we've given core 1 the framebuffer, it will just keep on displaying
    // it without any intervention from core 0
    sprite_fill16(framebuf, 0xffff, FRAME_WIDTH * FRAME_HEIGHT);
    uint16_t *bufptr = framebuf;
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    bufptr += FRAME_WIDTH;
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

    // Core 1 will wait until it sees the first colour buffer, then start up the
    // DVI signalling.
    multicore_launch_core1(core1_main);

    // USB
    board_init();

    // init device stack on configured roothub port
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    // I2C
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    while (true)
    {
        tud_task();
        core1_upkeep();
    }
}

//--------------------------------------------------------------------+
// Vendor callbacks
//--------------------------------------------------------------------+
void core1_upkeep()
{
    // Impact: DVI could fail to render
    // Tasks to be performed on core 1:
    // - Blinking LED
    // - Power LEDs
    // - PWM of backlight

    led_blinking_task();
    
}

void core0_upkeep()
{
    // Impact: USB could fail to respond
    // Tasks to be performed on core 1:
    // - Read I2C
}

//--------------------------------------------------------------------+
// Vendor callbacks
//--------------------------------------------------------------------+
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *data, uint16_t len)
{
    (void)itf;

    uint8_t *packet = (uint8_t *)data;
    uint8_t cmd = packet[0];
    if (cmd == 0x01)
    {
        if (len < 5)
        {
            blink_interval_ms = BLINK_FAIL;
            // Invalid packet length, handle this error or return
            return;
        }

        // Command 0x01: Upload pixel data to framebuffer
        // The next 2 bytes are X, and the next 2 bytes are Y
        uint16_t x = (packet[1] << 8) | packet[2]; // X-coordinate
        uint16_t y = (packet[3] << 8) | packet[4]; // Y-coordinate

        // Ensure the coordinates are within framebuffer bounds
        if (x >= FRAME_WIDTH || y >= FRAME_HEIGHT)
        {
            blink_interval_ms = BLINK_FAIL;
            // Out of bounds, handle this error or return
            return;
        }

        // The rest of the packet is the pixel data to copy into the framebuffer
        uint8_t *pixel_data = &packet[5];

        // Calculate the position in the framebuffer
        uint32_t pos = (y * FRAME_WIDTH) + x; // This gives the index (x + y * width) in the framebuffer

        // Calculate the number of pixels to copy (framebuffer stores 16-bit values, so each pixel is 2 bytes)
        uint16_t *framebuffer_ptr = &framebuf[pos];

        // Ensure the remaining pixel data fits in the framebuffer
        if (len - 5 >= 2)
        {
            // Assuming pixel_data contains 16-bit pixel values
            for (int i = 0; i < (len - 5) / 2; ++i)
            {
                framebuffer_ptr[i] = (pixel_data[i * 2] << 8) | pixel_data[i * 2 + 1];
            }
        }

        blink_interval_ms = BLINK_ALWAYS_ON;
        tud_vendor_read_flush();
        return;
    }

    blink_interval_ms = BLINK_ALWAYS_OFF;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    (void)rhport;

    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    if (request->bRequest == 0x01)
    {
        uint8_t data[64] = {0}; // Example response
        data[0] = 0xAA;         // Example data

        return tud_control_xfer(rhport, request, data, sizeof(data));
    }

     blink_interval_ms = BLINK_FAIL;
    return false;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;

    if (blink_interval_ms == BLINK_ALWAYS_ON)
    {
        board_led_write(true);
        return;
    }
    else if (blink_interval_ms == BLINK_ALWAYS_OFF)
    {
        board_led_write(false);
        return;
    }

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms)
        return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}