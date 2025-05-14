import usb.core
import usb.util
import struct
import time

# Replace with your RP2040 Vendor/Product ID (check with `system_profiler SPUSBDataType`)
VID = 0xcafe  # Raspberry Pi USB VID (default)
PID = 0x4001  # Custom device PID (you may have set this in your TinyUSB descriptor)

# Find the device
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    raise ValueError("Device not found")

# Detach from kernel driver if needed (macOS may attach a default driver)
if dev.is_kernel_driver_active(0):
    dev.detach_kernel_driver(0)

# Set configuration
dev.set_configuration()

# Interface and endpoints (adapt to your descriptor)
cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]

dev.ctrl_transfer(
    bmRequestType=usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR,
    bRequest=0x01,
    wValue=0,  # Custom value
    wIndex=0,  # Interface number (usually 0)
    data_or_wLength=None  # No data in this request
)

# Find endpoints
ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
ep_in = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

assert ep_out is not None, "OUT endpoint not found"
assert ep_in is not None, "IN endpoint not found"

# Frame parameters
color = 0x001F  # blue
width, height = 320, 240
CHUNK_PIXELS = 29  # 29 pixels * 2 = 58 bytes + 5 byte header = 63 bytes total (1 spare)

for y in range(height):
    for x in range(0, width, CHUNK_PIXELS):
        # Compute how many pixels to send in this chunk
        px_chunk = min(CHUNK_PIXELS, width - x)
        
        # Header: CMD=1, x offset, y offset
        header = struct.pack(">BHH", 0x01, x, y)
        
        # Pixel data: RGB565
        pixels = struct.pack(">" + "H" * px_chunk, *([color] * px_chunk))
        
        # Final packet
        print(f"Sending chunk: x={x}, y={y}, header={header}")
        packet = header + pixels
        ep_out.write(packet)

print("Frame transfer completed with chunked headers.")
