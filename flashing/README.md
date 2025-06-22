# PCB Flashing Instructions

This guide explains how to flash your PCB.

## Prerequisites
- Ensure you have the required drivers installed for your PCB.
- Download/Build the firmware file (e.g., `firmware.hex` or `firmware.bin`).
- Connect your PCB to your computer via USB.

## Files
For Windows:
- `WindowsFlash.bat`: The batch script to automate the flashing process.
- `firmware.hex` or `firmware.bin`: The firmware to be flashed.
- `esptool.py`: Python script for flashing ESP-based devices. Generally included in platformio when the firmware is built. i.e: \.platformio\packages\tool-esptoolpy\esptool.py

## Flashing Steps

1. **Run Script**  
Run the following command with the appropriate parameters:
```
./WindowsFlash.bat 921600 COM1 C:\path\to\esptool.py C:\path\to\flashfile.bin
```
   - Replace `921600` with the desired baud rate.
   - Replace `COM1` with the correct COM port for your PCB.
   - Replace `C:\path\to\esptool.py` with the path to your `esptool.py`.
   - Replace `C:\path\to\flashfile.bin` with the path to your firmware file.

2. **Connect PCB**  
    Plug your PCB into your computer.

