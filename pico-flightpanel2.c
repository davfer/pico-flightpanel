
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
#include "hardware/structs/bus_ctrl.h"
#include "hardware/dma.h"
#include "pico/sem.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"

#include "font_24x32.h"


// DVDD 1.2V (1.1V seems ok too)
// #define SCREEN_WIDTH 640
// #define SCREEN_HEIGHT 480
// #define FRAME_WIDTH SCREEN_WIDTH
// #define FRAME_HEIGHT SCREEN_HEIGHT
// #define VREG_VSEL VREG_VOLTAGE_1_20
// #define DVI_TIMING dvi_timing_640x480p_60hz

// #define CHAR_WIDTH 24
// #define CHAR_HEIGHT 32
// #define CHAR_COLS 24
// #define CHAR_ROWS 14
// #define SCREEN_MARGIN_X 8
// #define SCREEN_MARGIN_Y 8
// #define CHAR_MARGIN_X 2
// #define CHAR_MARGIN_Y 2
// #define FONT_HEADER_SIZE 4

// struct dvi_inst dvi0;
// uint16_t framebuf[FRAME_WIDTH];

#define FONT_FIRST_ASCII 32
#define FONT_LAST_ASCII 126
#define FONT_N_CHARS FONT_LAST_ASCII-FONT_FIRST_ASCII+1 // 32-126
#define FONT_CHAR_WIDTH 24
#define FONT_CHAR_HEIGHT 32
#define BYTES_PER_ROW (FONT_CHAR_WIDTH / 8)  // 3
#define FONT_BYTES_PER_CHAR (FONT_CHAR_HEIGHT * BYTES_PER_ROW)  // 96
#define CHAR_COLS 24
#define CHAR_ROWS 14
#define CHUNKS_PER_HLINE (FRAME_WIDTH / 8)


// Pick one:
#define MODE_640x480_60Hz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#define LED_PIN 16

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

// Grid struct {char, color, type} for each character in the grid
char grid_char[CHAR_COLS * CHAR_ROWS];
char grid_color[CHAR_COLS * CHAR_ROWS];
char grid_type[CHAR_COLS * CHAR_ROWS];

static inline uint8_t reverse_byte(uint8_t b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

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

// // Returns an array of 16bit bpp values representing a char [0-CHAR_WIDTH]
// // Pre: c is a valid ASCII character and y < CHAR_HEIGHT
// uint16_t* compute_char_scanline(uint y, char c, uint16_t fg, char type) {
//     const uint8_t char_width = Inconsola[0];
//     const uint8_t char_height = Inconsola[1];
//     const uint8_t bytes_per_row = (char_width + 7) / 8; // 1 byte per 8 pixels (ceiling)

//     uint16_t* char_buffer = malloc(char_width * sizeof(uint16_t));
//     if (!char_buffer) return NULL;

//     // Calculate starting index of this character's bitmap
//     uint char_offset = FONT_HEADER_SIZE +               // Skip header
//                        ((c - 0x20) * char_height * bytes_per_row) + // Each char has height rows of bytes (96 bytes per char)
//                        (y * bytes_per_row);               // Row offset in current char
//     for (uint x = 0; x < char_width; x++) {
//         uint8_t byte = Inconsola[char_offset + x / 8];
//         uint8_t bit = 7 - (x % 8); // MSB first
//         uint8_t val = (byte >> bit) & 1;

//         char_buffer[x] = val ? fg : 0x0000;
//     }
//     return char_buffer;
// }

// // Returns an array of 16bit bpp values representing a scanline SCREEN_WIDTH for the DVI output.
// // Pre: y < SCREEN_HEIGHT
// void compute_scanline(uint y) {
//     uint16_t* buffer;
//     for (uint x = 0; x < SCREEN_WIDTH; x++) {
//         if (x > 300) {
//             // Fill first 10 pixels with white
//             framebuf[x] = 0xFFFF; // White
//             continue;
//         }
//         if (y > 240) {
//             // Fill first 10 pixels with white
//             framebuf[x] = 0xFF00; // White
//             continue;
//         }


//         framebuf[x] = 0x0000; 

//         // compute row/col of the char grid based on x and y if not out of grid
//         // if (x >= CHAR_COLS * CHAR_WIDTH || y >= CHAR_ROWS * CHAR_HEIGHT) {
//         //     framebuf[x] = 0xF000;
//         //     continue;
//         // }
//         uint16_t char_pos = (x / CHAR_WIDTH) + ((y / CHAR_HEIGHT) * CHAR_COLS);

//         // buffer = compute_char_scanline(y % CHAR_HEIGHT, grid_char[char_pos], grid_color[char_pos], grid_type[char_pos]);
//         // for (uint i = 0; i < CHAR_WIDTH; i++) {
//         //     if (x >= SCREEN_WIDTH) {
//         //         continue;
//         //     }
//         //     framebuf[x] = buffer[i];
//         //     x++;
//         // }
//     }
// }


//--------------------------------------------------------------------+

// void core1_main() {
// 	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
// 	dvi_start(&dvi0);
// 	dvi_scanbuf_main_16bpp(&dvi0);
// 	__builtin_unreachable();
// }

// void core1_scanline_callback() {
// 	// // Discard any scanline pointers passed back
// 	// uint16_t *bufptr;
// 	// while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
// 	// 	;
// 	// // // Note first two scanlines are pushed before DVI start
// 	// static uint scanline = 2;
// 	// // bufptr = compute_scanline(scanline);
//     // for (uint i = 0; i < SCREEN_WIDTH; i++) {
//     //     bufptr[i] = 0xFF00;
//     // }
// 	// queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
// 	// scanline = (scanline + 1) % SCREEN_HEIGHT;
// }

static inline void prepare_scanline(const char *chars, uint y) {
	// static uint8_t scanbuf[CHUNKS_PER_HLINE]; // scanbuf is 8bit to fill all the width of the screen
	// // First blit font into 1bpp scanline buffer, then encode scanbuf into tmdsbuf
	// for (uint i = 0; i < CHUNKS_PER_HLINE; ++i) {
	// 	//uint c = chars[(i / BYTES_PER_CHAR) + ((y / FONT_CHAR_HEIGHT) * CHAR_COLS)]; // virtual 2d array [i][j] -> [i + j * CHAR_COLS]
	// 	// c contains the ASCII code of the character to be displayed, while y is 0-7 it results in 0 added offset in j

	// 	// Y POS = ((c - FONT_FIRST_ASCII) * FONT_BYTES_PER_CHAR) -> [first, last char]
	// 	// FONT-CHUNK base pos [0 - 95 char byte] defines Y 0-32
	// 	// FONT-CHUNK 3byte offset = i%3
	// 	uint c = chars[65-32];
	// 	// uint pos = ((c - FONT_FIRST_ASCII) * FONT_BYTES_PER_CHAR) + ((y % FONT_CHAR_HEIGHT) * BYTES_PER_CHAR) + (i % BYTES_PER_CHAR);

	// 	uint pos = ((c - FONT_FIRST_ASCII) * FONT_BYTES_PER_CHAR) 
	// 			+ ((y % FONT_CHAR_HEIGHT) * BYTES_PER_CHAR) // row offset
	// 			+ (i % BYTES_PER_CHAR); // byte within the row

	// 	scanbuf[i] = font_24x32[pos];
	// }

	// char A[] = {0x00,0x20,0x00,0x00,0x30,0x00,0x00,0x30,0x00,0x00,0x30,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0xFC,0x00,0x00,0xCC,0x00,0x00,0xCE,0x00,0x01,0x86,0x00,0x01,0x86,0x00,0x01,0x87,0x00,0x03,0x03,0x00,0x03,0x03,0x80,0x03,0xFF,0x80,0x07,0xFF,0x80,0x06,0x01,0xC0,0x0E,0x01,0xC0,0x0E,0x00,0xC0,0x0C,0x00,0xE0,0x1C,0x00,0xE0,0x1C,0x00,0x70,0x18,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	// for (uint i = 0; i < 3; ++i) {
	// 	uint pos = i + ((y % FONT_CHAR_HEIGHT) * BYTES_PER_CHAR);
	// 	scanbuf[i] = A[pos];
	// }

	static uint8_t scanbuf[CHUNKS_PER_HLINE];
	const uint char_row = y / FONT_CHAR_HEIGHT;      // Vertical char row
	const uint row_in_char = y % FONT_CHAR_HEIGHT;   // Row within a character

	#define MARGIN_LEFT_BYTES 1
	for (uint i = 0; i < CHAR_COLS; ++i) {  // One character per 24px block
		if (y >= FONT_CHAR_HEIGHT*CHAR_ROWS) {
			scanbuf[i] = 0;
			continue;
		}

		char c = chars[char_row * CHAR_COLS + i];
		if (c < FONT_FIRST_ASCII || c > FONT_LAST_ASCII) c = '?';

		uint glyph_base = (c - FONT_FIRST_ASCII) * FONT_BYTES_PER_CHAR;
		uint row_offset = row_in_char * BYTES_PER_ROW;

		// Copy 3 bytes of this row from the font into scanline
		for (int b = 0; b < BYTES_PER_ROW; ++b) {
			scanbuf[(i * BYTES_PER_ROW) + b + MARGIN_LEFT_BYTES] = reverse_byte(font_24x32[glyph_base + row_offset + b]);
		}
	}

	// Zero-fill any remaining scanbuf
	for (uint i = CHAR_COLS * BYTES_PER_ROW; i < CHUNKS_PER_HLINE; ++i) {
		scanbuf[i] = 0;
	}

	uint32_t *tmdsbuf;
	queue_remove_blocking(&dvi0.q_tmds_free, &tmdsbuf);
	tmds_encode_1bpp((const uint32_t*)scanbuf, tmdsbuf, FRAME_WIDTH);
	queue_add_blocking(&dvi0.q_tmds_valid, &tmdsbuf);
}

void core1_scanline_callback() {
	static uint y = 1;
	prepare_scanline(grid_char, y); // charbuf is ascii char 0-9a-zA-Z
	y = (y + 1) % FRAME_HEIGHT;
}

void __not_in_flash("main") core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	sem_acquire_blocking(&dvi_start_sem);
	dvi_start(&dvi0);

	// The text display is completely IRQ driven (takes up around 30% of cycles @
	// VGA). We could do something useful, or we could just take a nice nap
	while (1) 
		__wfi();
	__builtin_unreachable();
}


int __not_in_flash("main") main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
#ifdef RUN_FROM_CRYSTAL
	set_sys_clock_khz(12000, true);
#else
	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
#endif

	setup_default_uart();

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi0.scanline_callback = core1_scanline_callback;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	printf("Prepare first scanline\n");
	for (int i = 0; i < CHAR_ROWS * CHAR_COLS; ++i)
		grid_char[i] = (i % FONT_N_CHARS)+FONT_FIRST_ASCII;
	prepare_scanline(grid_char, 0);

	printf("Core 1 start\n");
	sem_init(&dvi_start_sem, 0, 1);
	hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
	multicore_launch_core1(core1_main);

	sem_release(&dvi_start_sem);

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