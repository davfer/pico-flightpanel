
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
#include "inconsola.c"


// DVDD 1.2V (1.1V seems ok too)
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#define CHAR_WIDTH 24
#define CHAR_HEIGHT 32
#define CHAR_COLS 24
#define CHAR_ROWS 14
#define SCREEN_MARGIN_X 8
#define SCREEN_MARGIN_Y 8
#define CHAR_MARGIN_X 2
#define CHAR_MARGIN_Y 2
#define FONT_HEADER_SIZE 4

struct dvi_inst dvi0;

// Grid struct {char, color, type} for each character in the grid
uint8_t grid_char[CHAR_COLS * CHAR_ROWS];
uint8_t grid_color[CHAR_COLS * CHAR_ROWS];
uint8_t grid_type[CHAR_COLS * CHAR_ROWS];

// USB
#include <stdio.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_FAIL = 100,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

//--------------------------------------------------------------------+

// Returns an array of 16bit bpp values representing a char [0-CHAR_WIDTH]
// Pre: c is a valid ASCII character and y < CHAR_HEIGHT
uint16_t* compute_char_scanline(uint y, char c, uint16_t fg, char type) {
    const char char_width = Inconsola[0];

    uint16_t* char_buffer = malloc(char_width * sizeof(uint16_t));
    for (uint x = 0; x < char_width; x++) {

        // Calculate the pixel value from the font data
        // Inconsola is a CHAR_WIDTH x CHAR_HEIGHT font
        // Font has a header that specifies the width and height of each character
        // Calculate the index in the font data (start font data) + (first pos of char) + (x pos in char) + (y pos in char)
        uint char_index = (FONT_HEADER_SIZE) +  ((c - 0x20) * char_width * CHAR_HEIGHT) + x + (y * char_width);
        uint8_t val = Inconsola[char_index];

        // Store the pixel value in the buffer
        uint8_t pos = (y * char_width) + x;
        if (val == 1) {
            char_buffer[pos] = fg;
        } else {
            char_buffer[pos] = 0x0000;
        }
    }
    return char_buffer;
    
}

// Returns an array of 16bit bpp values representing a scanline SCREEN_WIDTH for the DVI output.
// Pre: y < SCREEN_HEIGHT
uint16_t* compute_scanline(uint y) {
    static uint16_t scanline_buffer[SCREEN_WIDTH];

    uint16_t* buffer;
    for (uint x = 0; x < SCREEN_WIDTH; x++) {
        scanline_buffer[x] = 0xFF00; 

        // compute row/col of the char grid based on x and y if not out of grid
        if (x >= CHAR_COLS * CHAR_WIDTH || y >= CHAR_ROWS * CHAR_HEIGHT) {
            scanline_buffer[x] = 0x0000;
            continue;
        }
        uint16_t char_pos = (x / CHAR_WIDTH) + ((y / CHAR_HEIGHT) * CHAR_COLS);

        buffer = compute_char_scanline(y, grid_char[char_pos], grid_color[char_pos], grid_type[char_pos]);
        for (uint i = 0; i < CHAR_WIDTH; i++) {
            if (x >= SCREEN_WIDTH) {
                continue;
            }
            scanline_buffer[x] = buffer[i];
            x++;
        }
    }
    return scanline_buffer;
}


//--------------------------------------------------------------------+

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	dvi_scanbuf_main_16bpp(&dvi0);
	__builtin_unreachable();
}

void core1_scanline_callback() {
	// Discard any scanline pointers passed back
	uint16_t *bufptr;
	while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
		;
	// // Note first two scanlines are pushed before DVI start
	static uint scanline = 2;
	bufptr = compute_scanline(scanline);
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	scanline = (scanline + 1) % SCREEN_HEIGHT;
}

int main() {
    // DVI
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi0.scanline_callback = core1_scanline_callback;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // // Once we've given core 1 the framebuffer, it will just keep on displaying
	// // it without any intervention from core 0
	// sprite_fill16(framebuf, 0xffff, FRAME_WIDTH * FRAME_HEIGHT);
	// uint16_t *bufptr = framebuf;
	// queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	// bufptr += FRAME_WIDTH;
	// queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

	// Core 1 will wait until it sees the first colour buffer, then start up the
	// DVI signalling.
	multicore_launch_core1(core1_main);
	
    // USB
   board_init();

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
	
    while (true) {
        tud_task(); // tinyusb device task
        led_blinking_task();
    }
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
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
    (void) report_id;
    (void) report_type;

    uint8_t back[CFG_TUD_HID_EP_BUFSIZE] = {0};
    uint8_t errNo = 0;
    switch (buffer[0]) {
        case 0x01:
            if (bufsize < 6) {
                errNo = 10;
                break;
            }
            if (buffer[1] >= CHAR_COLS || buffer[2] >= CHAR_ROWS) {
                errNo = 11;
                break;
            }
            if (buffer[3] >= 0x80) {
                errNo = 12;
                break;
            }
            if (buffer[4] >= 10) {
                errNo = 13;
                break;
            }
            if (buffer[5] >= 2) {
                errNo = 14;
                break;
            }

            uint8_t pos = buffer[1] + (buffer[2] * CHAR_COLS);
            grid_char[pos] = buffer[3];
            grid_color[pos] = buffer[4];
            grid_type[pos] = buffer[5];

            back[0] = 0; // Success
            blink_interval_ms = BLINK_MOUNTED;
            break;
        default:
            errNo = 1; // Unknown command
            break;
    }

    if (errNo != 0) {
        blink_interval_ms = BLINK_FAIL;
        back[0] = 255; // Error
        back[1] = errNo; // Error Number
    }
    
    
    // sprite_fill16(framebuf, buffer[0] << 8 + buffer[1], FRAME_WIDTH * FRAME_HEIGHT);
	// uint16_t *bufptr = framebuf;
	// queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	// bufptr += FRAME_WIDTH;
	// queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    
    while (!tud_hid_ready()) tud_task();
    tud_hid_report(0, back, CFG_TUD_HID_EP_BUFSIZE);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}