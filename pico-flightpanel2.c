// USB
#include <stdio.h>
#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"

// STUFF
#include "hardware/pwm.h"
#include "hardware/i2c.h"

// MCP23017
#define MCP23017_I2C_ADDR 0x20 // I2C address for MCP23017
#define MCP23017_NUM 0 // Number of MCP23017 devices
#define MCP23017_I2C_SDA_PIN 0 // GPIO pin for I2C SDA
#define MCP23017_I2C_SCL_PIN 1 // GPIO pin for I2C SCL
#define MCP23017_POLLING_TIME 20 // GPIO pin for I2C SCL
#if MCP23017_NUM > 0
#include "mcp23017.h"
#endif

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

#define PWM_PIN 2
#define PWM_FREQ 1000

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
struct mutex switchbuff;

// Grid struct {char, color, type} for each character in the grid
char grid_char[CHAR_COLS * CHAR_ROWS];
char grid_color[CHAR_COLS * CHAR_ROWS];
char grid_type[CHAR_COLS * CHAR_ROWS];
uint8_t renderbuf_A[CHUNKS_PER_ROW * FRAME_HEIGHT];  
uint8_t renderbuf_B[CHUNKS_PER_ROW * FRAME_HEIGHT];
uint8_t renderreg = 0;                       


#if MCP23017_NUM > 0
//--------------------------------------------------------------------+
// I2C MCP23017
//--------------------------------------------------------------------+

uint16_t mcp23017_gpio[MCP23017_NUM]; // MCP23017 GPIO states
uint16_t mcp23017_gpio_masked[MCP23017_NUM]; // MCP23017 GPIO states
uint16_t mcp23017_gpio_dir[MCP23017_NUM] ; // MCP23017 GPIO direction states
uint8_t mcp23017_status[MCP23017_NUM]; // MCP23017 status

const uint16_t mcp23017_gpio_default_dir[MCP23017_NUM] = {
    0xFFFF, // 0x20: IN1 {CLR, V, /, W, DEL, ...}
    0xFFFF, // 0x21: IN2 {G, K, E, ...}
    // 0xF0F0, // 0x22: OUT {EXEC, FAIL, ...}
    0b1110000011110000, // 0x22: OUT {EXEC, FAIL, ...}
    0xFFFF, // 0x23: IN3 {0, ., 4, ...}
    0xFFFF, // 0x24: IN3 {+/-, CLB, F, ...}
};

const uint8_t mcp23017_address[MCP23017_NUM] = {
    MCP23017_I2C_ADDR, // 0x20
    MCP23017_I2C_ADDR + 1, // 0x21
    MCP23017_I2C_ADDR + 2, // 0x22
    MCP23017_I2C_ADDR + 3, // 0x23
    MCP23017_I2C_ADDR + 4, // 0x24
};


int mcp23017_write_register(uint8_t address, uint8_t reg, uint8_t value) {
	uint8_t command[] = { reg, value };
	int result = i2c_write_blocking(i2c0, address, command, 2, false);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}
	return PICO_ERROR_NONE;
}

uint16_t mcp23017_read_register(uint8_t address, uint8_t reg) {
	uint8_t buffer = 0;
	int result;
	result = i2c_write_blocking(i2c0, address,  &reg, 1, true);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}

	result = i2c_read_blocking(i2c0, address, &buffer, 1, false);
	if (result == PICO_ERROR_GENERIC)
		return result;

	return buffer;
}

int mcp23017_write_dual_registers(uint8_t address, uint8_t reg, int value) {
	uint8_t command[] = {
			reg,
			value & 0xff,
			(value>>8) & 0xff
	};
	int result = i2c_write_blocking(i2c0, address, command, 3, false);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}
	return PICO_ERROR_NONE;
}

uint16_t mcp23017_read_dual_registers(uint8_t address, uint8_t reg, uint8_t *err) {
	uint8_t buffer[2] = {0, 0};
	int result;
	result = i2c_write_blocking(i2c0, address,  &reg, 1, true);
	if (result == PICO_ERROR_GENERIC) {
		*err = 2;
		return 0;
	}

	result = i2c_read_blocking(i2c0, address, buffer, 2, false);
	if (result == PICO_ERROR_GENERIC) {
		*err = 3;
		return 0;
	}

	return (buffer[1]<<8) + buffer[0];
}

int mcp23017_set_io_direction(uint8_t address, int direction) {
	return mcp23017_write_dual_registers(address, MCP23017_IODIRA, direction); 
}

int mcp23017_set_pullup(uint8_t address, int direction) {
	return mcp23017_write_dual_registers(address, MCP23017_GPPUA, direction);
}

int mcp23017_set_pol(uint8_t address, int direction) {
	return mcp23017_write_dual_registers(address, MCP23017_IPOLA, direction);
}

void mcp23017_init(void)
{
    i2c_init(i2c0, 100 * 1000); // Initialize I2C at 100kHz
    gpio_set_function(MCP23017_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MCP23017_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MCP23017_I2C_SDA_PIN);
    gpio_pull_up(MCP23017_I2C_SCL_PIN);

    // Initialize MCP23017 devices
    for (uint8_t i = 0; i < MCP23017_NUM; i++)
    {
        uint8_t addr = mcp23017_address[i];
        int res = mcp23017_set_io_direction(addr, mcp23017_gpio_default_dir[i]);
        if (res != PICO_ERROR_NONE) {
            mcp23017_status[i] = 4;
            continue; // Error setting direction
        }
        res = mcp23017_set_pullup(addr, mcp23017_gpio_default_dir[i]);
        if (res != PICO_ERROR_NONE) {
            mcp23017_status[i] = 5;
            continue; // Error setting pullup
        }
        res = mcp23017_set_pol(addr, 0xFFFF);
        if (res != PICO_ERROR_NONE) {
            mcp23017_status[i] = 6;
            continue; // Error setting polarity
        }
        mcp23017_gpio_dir[i] = mcp23017_gpio_default_dir[i];
        mcp23017_status[i] = 1; // Device initialized successfully
    }
}

void mcp23017_polling_task(void)
{
    static uint32_t poll_ms = 0;
    if (board_millis() - poll_ms < MCP23017_POLLING_TIME)
        return; // not enough time
    poll_ms += MCP23017_POLLING_TIME;

    static uint8_t mcp23017_selected = 0;

    uint8_t addr = mcp23017_address[mcp23017_selected];
    mcp23017_gpio[mcp23017_selected] = mcp23017_read_dual_registers(addr, MCP23017_GPIOA, &mcp23017_status[mcp23017_selected]);
    mcp23017_gpio_masked[mcp23017_selected] |= mcp23017_gpio[mcp23017_selected];

    mcp23017_selected = (mcp23017_selected + 1) % MCP23017_NUM; // Cycle through MCP23017 devices

    // TEST CODE Update the grid_char array based on the MCP23017 GPIO states
    // const char x = 4;
    // const char y = 8;

    // for (uint8_t i = 0; i < MCP23017_NUM; i++)
    // {
    //     uint16_t gpio_value = mcp23017_gpio[i];
    //     for (uint16_t j = 0; j < 16; j++)
    //     {
    //         if (gpio_value & (1 << (j % 16)))
    //         {
    //             grid_char[((y + i) * CHAR_COLS) + x + j] = '1'; // Example character for pressed button
    //         }
    //         else
    //         {
    //             grid_char[((y + i) * CHAR_COLS) + x + j] = '0'; // Example character for unpressed button
    //         }
    //     }

    //     switch (mcp23017_status[i]) 
    //     {
    //     case 0:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = '-';
    //         break;
    //     case 1:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = 'S';
    //         break;
    //     case 2:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = 'F';
    //         break;
    //     case 3:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = 'G';
    //         break;
    //     case 4:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = 'H';
    //         break;
    //     case 5:
    //         grid_char[((y + i) * CHAR_COLS) + x - 2] = 'P';
    //         break;
    //     default:
    //         break;
    //     }
    // }
}

#endif

//--------------------------------------------------------------------+
// POLLING
//--------------------------------------------------------------------+

void poll_task(void)
{
    #if MCP23017_NUM > 0
    mcp23017_polling_task(); // Poll MCP23017 devices
    #endif
}

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
    uint8_t *framebuf = (renderreg == 0) ? renderbuf_B : renderbuf_A; // If renderreg is 0, use renderbuf_B, else use renderbuf_A

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
    // TODO: Render if needed

    uint8_t *framebuf = (renderreg == 0) ? renderbuf_B : renderbuf_A; // If renderreg is 0, use renderbuf_B, else use renderbuf_A

    // clear the frame buffer
    for (uint i = 0; i < CHUNKS_PER_ROW * FRAME_HEIGHT; ++i)
    {
        framebuf[i] = 0; // Clear the frame buffer
    }

    for (int grid_y = CHAR_ROWS - 1; grid_y >= 0; grid_y--) // Loop through rows in reverse order
    {
        for (int grid_x = 0; grid_x < CHAR_COLS; grid_x++)
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
    renderreg = (renderreg + 1) % 2; // Toggle between renderbuf_A and renderbuf_B
    
    sleep_ms(10);      // Give some time for the render to be processed
}

static inline void prepare_scanline(uint y)
{
    uint8_t *renderbuf = (renderreg == 0) ? renderbuf_A : renderbuf_B; // If renderreg is 0, use renderbuf_A, else use renderbuf_B
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
    while (1) {
        compute_render();
        // tus_task(); // tinyusb device task
    }
        
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

    mutex_init(&switchbuff);
    for (int i = 0; i < CHAR_ROWS * CHAR_COLS; ++i)
        grid_char[i] = ' ';

    memcpy(&grid_char[(6 * 24) + 5], "Fluffy FMC v1.0", 15);
    memcpy(&grid_char[(7 * 24) + 5], "Connect to App", 15);

    // for (int i = 0; i < CHAR_ROWS * CHAR_COLS; ++i)
    // 	grid_char[i] = (i % 10) + '0'; // Fill with numbers for testing

    renderreg = 1; // Compute in A
    compute_render();
    renderreg = 0; // Render back to A (compute in B)
    prepare_scanline(0);

    sem_init(&dvi_start_sem, 0, 1);
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
    multicore_launch_core1(core1_main);

    sem_release(&dvi_start_sem);

    // USB
    board_init();

    // PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, PWM_FREQ - 1); 
    pwm_set_gpio_level(PWM_PIN, PWM_FREQ / 2);
    pwm_set_enabled(slice_num, true);

    // I2C
    #if MCP23017_NUM > 0
    mcp23017_init();
    #endif

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
        poll_task();
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
// COMMANDS
//--------------------------------------------------------------------+
// SET_PIXEL [x, y, count, {char, color, type}...]
uint set_pixels(uint8_t const *buffer, uint16_t bufsize) {
    if (bufsize < 3)
        return 10;
    if (buffer[0] >= CHAR_COLS || buffer[1] >= CHAR_ROWS)
        return 11;
    if (buffer[2] > 20)
        return 15;
    

    uint16_t pos = buffer[0] + (buffer[1] * CHAR_COLS);
    if (pos >= CHAR_COLS * CHAR_ROWS || pos + buffer[2] > CHAR_COLS * CHAR_ROWS)
        return 16;

    for (uint8_t i = 0; i < buffer[2]; ++i)
    {
        uint8_t offset = (i * 3) + 3; // Start at 3 to skip position bytes
        if (buffer[offset + 0] >= 0x80)
            return 12; // Invalid character
        if (buffer[offset + 1] >= 10)
            return 13; // Invalid color
        if (buffer[offset + 2] >= 2)
            return 14; // Invalid type

        grid_char[pos + i] = buffer[offset + 0];
        grid_color[pos + i] = buffer[offset + 1];
        grid_type[pos + i] = buffer[offset + 2];
    }

    return 0;
}
uint set_pwm(uint8_t const *buffer, uint16_t bufsize) {
    if (bufsize < 1)
        return 10;

    // Set the PWM duty cycle based on the first byte of the buffer
    uint8_t duty_cycle = buffer[0];
    if (duty_cycle > 100)
        duty_cycle = 100; // Clamp to 100%

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_gpio_level(PWM_PIN, (PWM_FREQ * duty_cycle) / 100);

    return 0; // Success
}

#if MCP23017_NUM > 0
uint set_mcp_pin(uint8_t const *buffer, uint16_t bufsize) {
    if (bufsize < 2)
        return 10; // Not enough data

    // bufsize[0] contains the number of pin
    // bufsize[1] contains the value

    uint8_t pin = buffer[0];
    uint8_t value = buffer[1];

    if (pin >= 16 * MCP23017_NUM)
        return 11; // Invalid pin number

    uint8_t mcp_index = pin / 16; // Determine which MCP23017 device
    if (mcp23017_gpio_dir[mcp_index] & (1 << (pin % 16))) {
        // If the pin is configured as input, we cannot set its value
        return 12; // Pin is configured as input
    }

    uint16_t current_gpio = mcp23017_gpio[mcp_index];
    if (value)
        mcp23017_gpio[mcp_index] |= (1 << (pin % 16)); // Set the pin
    else
        mcp23017_gpio[mcp_index] &= ~(1 << (pin % 16)); // Clear the pin

    int res = mcp23017_write_dual_registers(mcp23017_address[mcp_index], MCP23017_GPIOA, mcp23017_gpio[mcp_index]);
    if (res != PICO_ERROR_NONE) {
        mcp23017_status[mcp_index] = 3; // Error writing to MCP23017
        return 13; // Error writing to MCP23017
    }

    return 0; // Success
}
uint get_mcp_pins(uint8_t const *buffer, uint16_t bufsize, uint8_t *back) {
    // if (bufsize < 1)
    //     return 10; // Not enough data

    // buffer[0] contains the pin number to read

    // #define MCP23017_NUM_PINS MCP23017_NUM*16 // 5 MCP23017 devices, 16 pins each
    // if (buffer[0] >= MCP23017_NUM_PINS)
    //     return 11; // Invalid pin number

    // back[0] = MCP23017_NUM_PINS - buffer[0]; // Return the number of pins to read (max 60)
    // if (back[0] > 60)
    //     back[0] = 60;

    // for (uint8_t i = 0; i < back[0]; ++i)
    // {
    //     uint8_t pin = buffer[0] + i;
    //     back[i + 1] = mcp23017_gpio[pin];
    // }

    for (uint8_t i = 0; i < MCP23017_NUM; ++i)
    {
        back[i * 2 + 0] = (mcp23017_gpio[i] >> 8) & 0xFF; // High byte
        back[i * 2 + 1] = mcp23017_gpio[i] & 0xFF; // Low byte
    }
    return 0; // Success
}
uint get_mcp_pins_masked(uint8_t const *buffer, uint16_t bufsize, uint8_t *back) {
    for (uint8_t i = 0; i < MCP23017_NUM; ++i)
    {
        back[i * 2 + 0] = (mcp23017_gpio_masked[i] >> 8) & 0xFF; // High byte
        back[i * 2 + 1] = mcp23017_gpio_masked[i] & 0xFF; // Low byte

        // Reset the masked GPIO state
        mcp23017_gpio_masked[i] = 0; // Clear the masked GPIO state
    }
    return 0; // Success
}
#endif

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
        errNo = set_pixels(buffer + 1, bufsize - 1);
        break;
    case 0x02:
        errNo = set_pwm(buffer + 1, bufsize - 1);
        break;
    #if MCP23017_NUM > 0
    case 0x03:
        errNo = set_mcp_pin(buffer + 1, bufsize - 1);
        break;
    case 0x04:
        errNo = get_mcp_pins(buffer + 1, bufsize - 1, back + 1);
        break;
    case 0x05:
        errNo = get_mcp_pins_masked(buffer + 1, bufsize - 1, back + 1);
        break;
    #endif
    // case 0xFE:
    //     reset_usb_
    //     break;
    default:
        errNo = 1; // Unknown command
        break;
    }

    if (errNo != 0)
    {
        blink_interval_ms = BLINK_FAIL;
        back[0] = 255;   // Error
        back[1] = errNo; // Error Number
    } else {
        blink_interval_ms = BLINK_MOUNTED; // Reset blink interval on success
        back[0] = 0; // Success
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