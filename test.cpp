#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "tm1637.pio.h"

/* This example demonstrate HID Generic raw Input & Output.
 * It will receive data from Host (In endpoint) and echo back (Out endpoint).
 * HID Report descriptor use vendor for usage page (using template TUD_HID_REPORT_DESC_GENERIC_INOUT)
 *
 * There are 2 ways to test the sketch
 * 1. Using nodejs
 *    - Install nodejs and npm to your PC
 *    - Install excellent node-hid (https://github.com/node-hid/node-hid) by
 *      $ npm install node-hid
 *    - Run provided hid test script
 *      $ node hid_test.js
 *
 * 2. Using python hidRun
 *    - Python and `hid` package is required, for installation please follow https://pypi.org/project/hid/
 *    - Run provided hid test script to send and receive data to this device.
 *      $ python3 hid_test.py
 */

//--------------------------------------------------------------------+
// USER CODE
//--------------------------------------------------------------------+

#define CLUSTER_SIZE 16
#define CLUSTERS  1
#define ENCODERS  1
#define DISPLAYS  1

#define POOL_DELAY_SAMPLE_US 100
#define POOL_DELAY_SWITCH_US 10
#define POOL_VALID_INPUT_MS 200

const uint8_t MUX_PIN0 = 2;
const uint8_t MUX_PIN1 = 3;
const uint8_t MUX_PIN2 = 4;
const uint8_t MUX_PIN3 = 5;

static const uint8_t digitToSegment[] = {
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
};

uint32_t ts_port[CLUSTERS][CLUSTER_SIZE];
uint8_t buff_encs[ENCODERS][3];

uint8_t port[CLUSTERS][CLUSTER_SIZE];
uint8_t tris[CLUSTERS][CLUSTER_SIZE];
uint8_t encs[ENCODERS];
uint8_t digs[DISPLAYS][6] = {
    { 2, 1, 0, 5, 4, 3 }
};
uint16_t adc = 0;

uint8_t mux_addr[CLUSTERS] = { 0 };
uint8_t enc_addr[ENCODERS][2] = { { 6, 7 } }; // B0 B1
uint8_t dig_addr[DISPLAYS][2] = { { 8, 9 } }; // CL DA

uint8_t idx = 0;

void setPinDir(uint8_t address, uint8_t dir) {
    tris[address/CLUSTER_SIZE][address % CLUSTER_SIZE] = dir > 0;
}
void setPinValue(uint8_t address, uint8_t value) {
    port[address/CLUSTER_SIZE][address % CLUSTER_SIZE] = value > 0;
}
uint8_t getPinValue(uint8_t address) {
    return port[address/CLUSTER_SIZE][address % CLUSTER_SIZE];
}
void setDigit(uint8_t cluster, uint8_t idx, uint8_t digit, uint8_t brightness, uint8_t colon) {
    if (brightness > 0b00000111) {
        brightness = 0b00000111;
    }

    uint8_t addr = 0xC0 + digs[cluster][idx];
    uint8_t brig = 0x88 + brightness;
    uint8_t valu = digitToSegment[digit];

    if (colon > 0) {
        valu += 0b10000000;
    }

    uint32_t data = (valu << 24) + (addr << 16) + (0x44 << 8) + (brig); // 0xffc0448a
    printf("%x\n", data);

    pio_sm_put_blocking(pio0, cluster, data);
}

void analog_init() {
    adc_init();
    adc_gpio_init(28);
    adc_select_input(2);
}
void encoders_init() {
    for (uint8_t i = 0; i < ENCODERS; i++) {
        gpio_init(enc_addr[i][0]);
        gpio_init(enc_addr[i][1]);

        gpio_set_dir(enc_addr[i][0], 0);
        gpio_set_dir(enc_addr[i][0], 0);

        gpio_pull_down(enc_addr[i][0]);
        gpio_pull_down(enc_addr[i][1]);

        buff_encs[i][0] = gpio_get(enc_addr[i][0]);
        buff_encs[i][1] = gpio_get(enc_addr[i][1]);
        buff_encs[i][2] = 0;
    }
}
void digits_init() {
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &tm1637_program);
    for (uint8_t i = 0; i < DISPLAYS; i++) {
        tm1637_program_init(pio, i, offset, dig_addr[i][0], dig_addr[i][1]);
    }
}
void gpio_init() {
    gpio_init(MUX_PIN0);
    gpio_init(MUX_PIN1);
    gpio_init(MUX_PIN2);
    gpio_init(MUX_PIN3);

    gpio_set_dir(MUX_PIN0, 1);
    gpio_set_dir(MUX_PIN1, 1);
    gpio_set_dir(MUX_PIN2, 1);
    gpio_set_dir(MUX_PIN3, 1);

    for (uint8_t i = 0; i < CLUSTERS; i++) {
        gpio_init(mux_addr[i]);
        gpio_pull_down(mux_addr[i]);
    }
}

void loop() {
    uint32_t ts = to_ms_since_boot(get_absolute_time());

    // READ ENCODERS
    for (uint8_t i = 0; i < ENCODERS; i++) {
        uint8_t b1 = gpio_get(enc_addr[i][0]);
        uint8_t b2 = gpio_get(enc_addr[i][1]);

        uint8_t val = 0; // 0:01 1:11 2:10 3:00
        if (b2 == 1 && b1 == 1) {
            val = 1;
        } else if (b2 == 1 && b1 == 0) {
            val = 2;
        } else if (b2 == 0 && b1 == 0) {
            val = 3;
        }

        if (val != buff_encs[i][2]) {
            buff_encs[i][0] <<= 1;
            buff_encs[i][1] <<= 1;

            buff_encs[i][0] |= b1;
            buff_encs[i][1] |= b2;

            buff_encs[i][0] &= 0x0F;
            buff_encs[i][1] &= 0x0F;

            if (buff_encs[i][0] == 0b00000110 && buff_encs[i][1] == 0b00001100) {
                encs[i] -= 1;
            }
            if (buff_encs[i][0] == 0b00001100 && buff_encs[i][1] == 0b00000110) {
                encs[i] += 1;
            }

            buff_encs[i][2] = val;
        }
    }

    // SET MUX ADDRESS
    gpio_put(MUX_PIN0, ((1 << 0) & idx) > 0);
    gpio_put(MUX_PIN1, ((1 << 1) & idx) > 0);
    gpio_put(MUX_PIN2, ((1 << 2) & idx) > 0);
    gpio_put(MUX_PIN3, ((1 << 3) & idx) > 0);
    gpio_set_dir(mux_addr[0], 0);

    for (uint8_t i = 0; i < CLUSTERS; i++) {
        gpio_set_dir(mux_addr[i], tris[i][idx]); 

        if (tris[i][idx] == 1) { 
            gpio_put(mux_addr[i], port[i][idx]);
        }
    }

    sleep_us(POOL_DELAY_SWITCH_US);

    // READ ADC
    const float conversion_factor = 3.3f / (1 << 12);
    adc = adc_read();

    // GET MUX
    for (uint8_t i = 0; i < CLUSTERS; i++) {
        if (tris[i][idx] == 0) {
            if (port[i][idx] != gpio_get(mux_addr[i])) {
                if (ts_port[i][idx] == 0) {
                    ts_port[i][idx] = ts;
                } else if (ts_port[i][idx] + POOL_VALID_INPUT_MS < ts) {
                    port[i][idx] = gpio_get(mux_addr[i]);
                }
            } else {
                ts_port[i][idx] = 0; // Reset time
            }
        }
    }

    // SAMPLE
    sleep_us(POOL_DELAY_SAMPLE_US);

    // CHECK READ
    idx = (idx + 1) % CLUSTER_SIZE;
}



uint8_t cmd_set_dir(uint8_t address, uint8_t dir) {
    tris[address/CLUSTER_SIZE][address % CLUSTER_SIZE] = dir > 0;
    
    return 1;
}
uint8_t cmd_set_out(uint8_t address, uint8_t val) {
    port[address/CLUSTER_SIZE][address % CLUSTER_SIZE] = val > 0;
    
    return 1;
}
uint8_t cmd_get_in(uint8_t address) {
    return port[address/CLUSTER_SIZE][address % CLUSTER_SIZE] > 0;
}
uint8_t cmd_get_encoder(uint8_t address) {
    return encs[address];
}
uint8_t cmd_set_digit(uint8_t display, uint8_t address, uint8_t value, uint8_t bright) {
    setDigit(display, address, value, bright, 0);

    return 1;
}

uint16_t getCluster(uint8_t offset) {
    uint16_t res = 0;
    for (uint8_t i = 0; i < CLUSTERS; i++) {
        if (port[offset][i] > 0) {
            res += (1 << i);
        }
    }

    return res;
}


//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

/*------------- MAIN -------------*/
int main(void) {
    board_init();
    tusb_init();
    
    analog_init();
    encoders_init();
    gpio_init();
    digits_init();

    while (1) {
        tud_task(); // tinyusb device task
        led_blinking_task();
        loop();
    }

    return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // TODO not Implemented
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // This example doesn't use multiple report and report ID
    (void) report_id;
    (void) report_type;

    uint8_t back[bufsize] = {0};
    switch (buffer[0])
    {
        case 0x01:
            back[0] = cmd_set_dir(buffer[1], buffer[2]);
            back[1] = buffer[1];
            back[2] = buffer[2];

            break;
        case 0x02:
            back[0] = cmd_set_out(buffer[1], buffer[2]);
            back[1] = buffer[1];
            back[2] = buffer[2];

            break;
        case 0x03:
            back[0] = cmd_get_in(buffer[1]);
            back[1] = buffer[1];

            break;
        case 0x04:
            back[0] = cmd_get_encoder(buffer[1]);
            back[1] = buffer[1];

            break;
        case 0x05:
            back[0] = cmd_set_digit(buffer[1], buffer[2], buffer[3], buffer[4]);
            back[1] = buffer[1];
            back[2] = buffer[2];
            back[3] = buffer[3];
            back[4] = buffer[4];

            break;
        case 0x06:
            back[0] = 0x00;

            break;
        case 0x07:
            back[0] = 0x07;
            back[1] = gpio_get_all() & 0xFF;
            back[2] = gpio_get_all() >> 8;
            back[3] = gpio_get_all() >> 16;
            back[4] = gpio_get_all() >> 24;

            break;
        
        default:
            break;
    }

    // echo back anything we received from host
    tud_hid_report(0, back, bufsize);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}
