#!/usr/bin/env python3
# Install python3 HID package https://pypi.org/project/hid/
import hid

# default is TinyUSB (0xcafe), Adafruit (0x239a), RaspberryPi (0x2e8a), Espressif (0x303a) VID
USB_VID = (0xcafe, 0x239a, 0x2e8a, 0x303a)

print("VID list: " + ", ".join('%02x' % v for v in USB_VID))

# get char and x and y from args
x,y = 0, 0
import sys
if len(sys.argv) > 1:
    try:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    except ValueError:
        print("Invalid x value, using default 0")
c = 'c'  # default character to send
if len(sys.argv) > 3:
    c = sys.argv[3]


for vid in  USB_VID:
    for dict in hid.enumerate(vid):
        print(dict)
        dev = hid.Device(dict['vendor_id'], dict['product_id'])
        if dev is not None:
            print("Found device:", dict['product_string'], "at", dict['path'])
            print("Vendor ID:", hex(dict['vendor_id']), "Product ID:", hex(dict['product_id']))

            command = bytes([1, x, y, ord(c), 0x00, 0x00])
            try:
                dev.write(command)
                print("Command sent:", command)
            except Exception as e:
                print("Error sending command:", e)
            try:
                response = dev.read(64) 
                print("Response received:", response)
            except Exception as e:
                print("Error reading response:", e)
            # Close the device
            dev.close()
            print("Device closed.")