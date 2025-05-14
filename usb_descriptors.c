#include "tusb.h"

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210, 
.bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCAFE,
    .idProduct = 0x4001,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 1
};

#define ITF_NUM_VENDOR  0
#define STRID_VENDOR    5
#define EPNUM_VENDOR_OUT  0x01
#define EPNUM_VENDOR_IN   0x81  // 0x80 | 1

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

uint8_t const desc_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // en-US
    "My Company",                  // Manufacturer
    "RP2040 Vendor Transfer",      // Product
    "123456",                      // Serial Number
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t desc_str[32];
    (void) langid;

    uint8_t chr_count;
    if (index == 0) {
        desc_str[1] = 0x0409;
        return desc_str;
    }

    const char* str = string_desc_arr[index];
    chr_count = strlen(str);
    desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2 * chr_count + 2);
    for(uint8_t i=0; i<chr_count; i++) {
        desc_str[1+i] = str[i];
    }

    return desc_str;
}
