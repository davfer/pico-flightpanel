// USB
#include <stdio.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"

// DVI
#include <stdlib.h>
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/dma.h"
#include "pico/sync.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"

#include "inconsola.c"

#define CHAR_COLS 24
#define CHAR_ROWS 14
#define SCREEN_MARGIN_X 8
#define SCREEN_MARGIN_Y 4
#define CHAR_MARGIN_X 2
#define CHAR_MARGIN_Y 2
#define CHUNKS_PER_ROW (FRAME_WIDTH / 8)

typedef struct
{
    const unsigned char char_width;  // 24
    const unsigned char char_height; // 32
    const unsigned char first_ascii; // 32
    const unsigned char n_chars;     // 95
    const unsigned char *fontdata;
    const unsigned char bytes_per_row;  // 3
    const unsigned char bytes_per_char; // 96
} font_t;

const font_t bigfont = {
    .char_width = Inconsola[0],
    .char_height = Inconsola[1],
    .first_ascii = Inconsola[2],
    .n_chars = Inconsola[3],
    .fontdata = &Inconsola[4],
    .bytes_per_row = (Inconsola[0] + 7) / 8,
    .bytes_per_char = ((Inconsola[0] + 7) / 8) * Inconsola[1],
};

// Pick one:
#define MODE_640x480_60Hz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;
struct mutex copying_sem;

// Grid struct {char, color, type} for each character in the grid
char grid_char[CHAR_COLS * CHAR_ROWS];
char grid_color[CHAR_COLS * CHAR_ROWS];
char grid_type[CHAR_COLS * CHAR_ROWS];
uint8_t framebuf[CHUNKS_PER_ROW * FRAME_HEIGHT];  // 1bpp framebuffer for the DVI output
uint8_t renderbuf[CHUNKS_PER_ROW * FRAME_HEIGHT]; // 1bpp render buffer for the DVI output
uint8_t renderrequest = 0;                        // Flag to indicate a render request

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
    LED_OFF = 0,
    LED_ON = UINT32_MAX,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

//--------------------------------------------------------------------+

static inline void compute_char(uint dest_x, uint dest_y, char c)
{
    uint char_index = c - bigfont.first_ascii;
    for (uint y = 0; y < bigfont.char_height; ++y)
    {
        const uint framebuf_posy = (dest_y + y) * CHUNKS_PER_ROW;

        for (uint x = 0; x < bigfont.char_width; ++x)
        {
            uint char_offset = (y * bigfont.bytes_per_row) + (char_index * bigfont.bytes_per_char);
            uint8_t font_byte = bigfont.fontdata[char_offset + (x / 8)];
            uint8_t bit = 7 - (x % 8);
            uint8_t pix = (font_byte >> bit) & 1;

            const uint framebuf_posx = dest_x + x;
            const uint framebuf_pos = framebuf_posy + (framebuf_posx / 8); // x7 0 , x8 1
            const uint framebuf_bit = framebuf_posx % 8;                   // b0 , b7

            if (pix == 0)
            {
                framebuf[framebuf_pos] &= ~(1 << framebuf_bit); // Clear bit (black pixel)
            }
            else
            {
                framebuf[framebuf_pos] |= (1 << framebuf_bit); // Set bit (white pixel)
            }
        }
    }
}

static inline void compute_render()
{
    mutex_enter_blocking(&copying_sem);

    // clear the frame buffer
    for (uint i = 0; i < CHUNKS_PER_ROW * FRAME_HEIGHT; ++i)
    {
        framebuf[i] = 0; // Clear the frame buffer
    }

    for (uint grid_y = 0; grid_y < CHAR_ROWS; grid_y++)
    {
        for (uint grid_x = 0; grid_x < CHAR_COLS; grid_x++)
        {
            uint16_t char_pos = grid_x + (grid_y * CHAR_COLS);
            char c = grid_char[char_pos];
            if (c == 0)
                c = ' '; // Use space for empty characters
            else if (c < bigfont.first_ascii || c >= bigfont.first_ascii + bigfont.n_chars)
                c = '?'; // Invalid character, use qustion mark


            int8_t marginy = 0;

            switch (grid_y) {
                case 0:
                    marginy = 0;
                    break;
                case 8:
                    marginy = -12;
                    break;
                case 10:
                    marginy = -10;
                    break;
                case 12:
                    marginy = -8;
                    break;
                case CHAR_ROWS - 1:
                    marginy = 0;
                    break;
                default:
                    marginy = -16;
            }

            if (grid_y != CHAR_ROWS - 1 && grid_y % 2 != 0 && c >= 'A' && c <= 'Z'){
                c = c + 32; // Convert to lowercase for odd rows
            }

            uint16_t charpos_x = (grid_x * bigfont.char_width) + (grid_x * CHAR_MARGIN_X) + SCREEN_MARGIN_X;
            uint16_t charpos_y = (grid_y * bigfont.char_height) + (grid_y * CHAR_MARGIN_Y) + SCREEN_MARGIN_Y;
            compute_char(charpos_x, charpos_y + marginy, c);
        }
    }

    mutex_exit(&copying_sem);
    renderrequest = 1; // Set render request flag
    sleep_ms(10);      // Give some time for the render to be processed
}

static inline void prepare_scanline(uint y)
{
    static uint8_t scanbuf[CHUNKS_PER_ROW];
    const uint16_t ypos = y * CHUNKS_PER_ROW;

    for (uint x = 0; x < CHUNKS_PER_ROW; ++x)
    {
        scanbuf[x] = renderbuf[ypos + x]; // Clear the scanline buffer
    }

    uint32_t *tmdsbuf;
    queue_remove_blocking(&dvi0.q_tmds_free, &tmdsbuf);
    tmds_encode_1bpp((const uint32_t *)scanbuf, tmdsbuf, FRAME_WIDTH);
    queue_add_blocking(&dvi0.q_tmds_valid, &tmdsbuf);
}

void core1_scanline_callback()
{
    static uint y = 1;
    static uint copying = 0;

    if (y == 0 && renderrequest != 0 && mutex_enter_timeout_us(&copying_sem, 1))
    {
        copying = 1; // We are copying the render buffer to the scanline buffer
    }
    if (copying)
    {
        // Copy the render buffer to the scanline buffer
        const uint16_t ypos = y * CHUNKS_PER_ROW;
        for (uint x = 0; x < CHUNKS_PER_ROW; ++x)
        {
            renderbuf[ypos + x] = framebuf[ypos + x];
        }

        if (y == FRAME_HEIGHT - 1)
        {
            renderrequest = 0;
            copying = 0;
            mutex_exit(&copying_sem);
        }
    }

    prepare_scanline(y);
    y = (y + 1) % FRAME_HEIGHT;
}

void __not_in_flash("main") core1_main()
{
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    sem_acquire_blocking(&dvi_start_sem);
    dvi_start(&dvi0);

    // The text display is completely IRQ driven (takes up around 30% of cycles @
    // VGA). We could do something useful, or we could just take a nice nap
    while (1)
        compute_render();

    __builtin_unreachable();
}

int __not_in_flash("main") main()
{
    vreg_set_voltage(VREG_VSEL);
    sleep_ms(10);
#ifdef RUN_FROM_CRYSTAL
    set_sys_clock_khz(12000, true);
#else
    // Run system at TMDS bit clock
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
#endif

    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi0.scanline_callback = core1_scanline_callback;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    mutex_init(&copying_sem);
    for (int i = 0; i < CHAR_ROWS * CHAR_COLS; ++i)
        grid_char[i] = ' ';

    memcpy(&grid_char[(6 * 24) + 5], "Fluffy FMC v1.0", 15);

    // for (int i = 0; i < CHAR_ROWS * CHAR_COLS; ++i)
    // 	grid_char[i] = (i % 10) + '0'; // Fill with numbers for testing

    compute_render();
    for (int i = 0; i < CHUNKS_PER_ROW * FRAME_HEIGHT; ++i)
    {
        framebuf[i] = renderbuf[i];
    }
    prepare_scanline(0);

    sem_init(&dvi_start_sem, 0, 1);
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
    multicore_launch_core1(core1_main);

    sem_release(&dvi_start_sem);

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

    while (true)
    {
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
    (void)remote_wakeup_en;
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
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void)itf;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    // This example doesn't use multiple report and report ID
    (void)report_id;
    (void)report_type;

    uint8_t back[CFG_TUD_HID_EP_BUFSIZE] = {0};
    uint8_t errNo = 0;
    switch (buffer[0])
    {
    case 0x01:
        if (bufsize < 4)
        {
            errNo = 10;
            break;
        }
        if (buffer[1] >= CHAR_COLS || buffer[2] >= CHAR_ROWS)
        {
            errNo = 11;
            break;
        }
        if (buffer[3] > 20)
        {
            errNo = 15;
            break;
        }

        uint16_t pos = buffer[1] + (buffer[2] * CHAR_COLS);
        if (pos >= CHAR_COLS * CHAR_ROWS || pos + buffer[3] > CHAR_COLS * CHAR_ROWS)
        {
            errNo = 16;
            break;
        }

        for (uint8_t i = 0; i < buffer[3]; ++i)
        {
            uint8_t offset = (i * 3) + 4; // Start at 4 to skip command and position bytes
            if (buffer[offset + 0] >= 0x80)
            {
                errNo = 12;
                break;
            }
            if (buffer[offset + 1] >= 10)
            {
                errNo = 13;
                break;
            }
            if (buffer[offset + 2] >= 2)
            {
                errNo = 14;
                break;
            }

            grid_char[pos + i] = buffer[offset + 0];
            grid_color[pos + i] = buffer[offset + 1];
            grid_type[pos + i] = buffer[offset + 2];
        }

        back[0] = 0; // Success
        blink_interval_ms = BLINK_MOUNTED;

        break;
    default:
        errNo = 1; // Unknown command
        break;
    }

    if (errNo != 0)
    {
        blink_interval_ms = BLINK_FAIL;
        back[0] = 255;   // Error
        back[1] = errNo; // Error Number
    }

    while (!tud_hid_ready())
        tud_task();
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
    if (board_millis() - start_ms < blink_interval_ms)
        return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}