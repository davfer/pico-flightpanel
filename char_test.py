import matplotlib.pyplot as plt
import numpy as np
import re
from pathlib import Path

# Load the font data
font_c_path = Path("inconsola_updated.c")
font_c_content = font_c_path.read_text()
hex_values = re.findall(r'0x[0-9A-Fa-f]+', font_c_content)
font_bytes = [int(h, 16) for h in hex_values]

# Font metadata
char_width = font_bytes[0]
char_height = font_bytes[1]
first_ascii = font_bytes[2]
num_chars = font_bytes[3]

bytes_per_row = (char_width + 7) // 8
bytes_per_char = char_height * bytes_per_row

# Choose a character to edit
target_char = '_'
char_index = ord(target_char) - first_ascii
char_data_offset = 4 + char_index * bytes_per_char
bitmap_data = font_bytes[4:]

# Load character bitmap into a grid
char_grid = np.zeros((char_height, char_width), dtype=int)
for y in range(char_height):
    row_offset = char_data_offset - 4 + y * bytes_per_row
    for byte_index in range(bytes_per_row):
        font_byte = bitmap_data[row_offset + byte_index]
        for b in range(8):
            x = byte_index * 8 + (7 - b)  # MSB-first
            if x < char_width:
                char_grid[y, x] = (font_byte >> b) & 1

# Setup the interactive plot
fig, ax = plt.subplots()
img = ax.imshow(char_grid, cmap='Greys', interpolation='nearest')
ax.set_title(f"Edit character '{target_char}' - press 's' to save")
ax.set_xticks([])
ax.set_yticks([])

def onclick(event):
    if event.inaxes != ax:
        print("Click inside the character grid to toggle pixels.")
        return
    x, y = int(event.xdata + 0.5), int(event.ydata + 0.5)
    if 0 <= x < char_width and 0 <= y < char_height:
        char_grid[y, x] ^= 1
        img.set_data(char_grid)
        fig.canvas.draw()
    else:
        print(f"Click at ({x}, {y}) is out of bounds for character size {char_width}x{char_height}.")

def onkey(event):
    if event.key == 'j':
        # Convert the char grid back into font bytes (MSB-first)
        new_bytes = []
        for y in range(char_height):
            for byte_index in range(bytes_per_row):
                b = 0
                for bit in range(8):
                    x = byte_index * 8 + (7 - bit)
                    if x < char_width and char_grid[y, x]:
                        b |= (1 << bit)
                new_bytes.append(b)
        
        # Replace the character bytes in the font array
        for i in range(bytes_per_char):
            font_bytes[char_data_offset + i] = new_bytes[i]

        # Write updated font data to new C file
        formatted_bytes = [f"0x{b:02X}" for b in font_bytes]
        lines = []
        for i in range(0, len(formatted_bytes), 16):
            lines.append(",".join(formatted_bytes[i:i+16]))
        c_output = f"fontdatatype Inconsola[{len(font_bytes)}] PROGMEM={{\n" + ",\n".join(lines) + "\n};\n"
        
        updated_path = Path("inconsola_updated.c")
        updated_path.write_text(c_output)
        print(f"Character '{target_char}' saved to inconsola_updated.c")

fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', onkey)
plt.show()
