import esptool
import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox

def list_serial_ports():
    """List available COM ports with descriptions."""
    return [(port.device, port.description) for port in serial.tools.list_ports.comports()]

def select_com_port():
    """Popup to select COM port."""
    ports = list_serial_ports()
    if not ports:
        messagebox.showerror("Error", "No COM ports found. Connect your LevelMinder and try again.")
        return None

    port_window = tk.Tk()
    port_window.title("Select COM Port")

    # Display COM + description
    port_options = [f"{dev} ({desc})" for dev, desc in ports]

    selected_port = tk.StringVar(port_window)
    selected_port.set(port_options[0])  # Default to first available

    tk.Label(port_window, text="Choose COM Port:").pack(pady=5)
    tk.OptionMenu(port_window, selected_port, *port_options).pack(pady=5)

    def confirm():
        port_window.destroy()

    tk.Button(port_window, text="OK", command=confirm).pack(pady=5)
    port_window.mainloop()

    # Extract just the device port (COMx)
    chosen = selected_port.get().split(" ")[0]
    return chosen

def select_bin_file():
    """Popup to select binary file."""
    return filedialog.askopenfilename(
        title="Select Binary File",
        filetypes=[("Binary Files", "*.bin")]
    )

def flash_firmware(port, bin_path):
    """Flash firmware to LevelMinder."""
    try:
        # ESP32-S3 default flash settings
        esptool_args = [
            "--chip", "esp32s3",
            "--port", port,
            "--baud", "460800",
            "write_flash", "-z",
            "0x10000", bin_path
        ]
        print(f"Running esptool with args: {esptool_args}")
        esptool.main(esptool_args)
        messagebox.showinfo("Success", f"Flashed {bin_path} to {port} successfully.")
    except Exception as e:
        messagebox.showerror("Flashing Error", str(e))

def main():
    port = select_com_port()
    if not port:
        return

    bin_file = select_bin_file()
    if not bin_file:
        messagebox.showwarning("Canceled", "No binary file selected.")
        return

    flash_firmware(port, bin_file)

if __name__ == "__main__":
    main()