import tkinter as tk
from tkinter import filedialog
import os

def bin_to_c_array():
    # Hide Tkinter root window
    root = tk.Tk()
    root.withdraw()

    # Ask user to select .bin file
    bin_path = filedialog.askopenfilename(
        title="Select a .bin file",
        filetypes=[("Binary files", "*.bin")]
    )

    if not bin_path:
        print("No file selected.")
        return

    # Read raw binary data
    with open(bin_path, "rb") as f:
        data = f.read()

    # Build C-style array
    array_name = os.path.splitext(os.path.basename(bin_path))[0]
    byte_lines = []

    # Optional: group bytes 16 per line for readability
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        line = ", ".join(f"0x{b:02X}" for b in chunk)
        byte_lines.append("    " + line)

    c_array_text = (
        f"// Generated from {os.path.basename(bin_path)}\n"
        f"// Size: {len(data)} bytes\n\n"
        f"const uint8_t {array_name}[] = {{\n"
        + ",\n".join(byte_lines) +
        "\n};\n"
    )

    # Output .txt or .c file
    out_path = bin_path + ".c"

    with open(out_path, "w") as out:
        out.write(c_array_text)

    print(f"C-style array written to: {out_path}")

bin_to_c_array()