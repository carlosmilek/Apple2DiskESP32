# ESP32-S3 Apple II Disk II Emulator

![Apple II](https://img.shields.io/badge/Apple-II-success?style=flat-square&logo=apple)
![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Supported-blue?style=flat-square&logo=espressif)
![Status](https://img.shields.io/badge/Status-v0.1-green?style=flat-square)

FOR NOW IT´S JUST READ ONLY!! WRITE WILL BE IMPLEMENTED SOON!

A highly optimized, dual-core Wi-Fi floppy drive emulator for the Apple II series, built around the ESP32-S3 microcontroller. This project allows you to load, mount, and manage your vintage Apple II disk images wirelessly through a retro green-phosphor themed web interface. Use ESP32-S3-DevKitC-1 16mb Flash 8mb Psram.

## 🚀 Features

- **Dual-Core Architecture:** 
  - **Core 1 (Bare-Metal):** Dedicated entirely to real-time Disk II bus emulation. Handles high-speed pin polling, strict phase timing, and bit-level read/write operations with interrupts disabled during critical transmit windows.
  - **Core 0 (System/Network):** Handles the asynchronous workload, including the Wi-Fi stack, HTTP Web Server, LittleFS file management, and heavy on-the-fly GCR conversions.
- **On-the-Fly Conversion:** Upload standard `.dsk` (DOS 3.3 logical order) or `.po` (ProDOS block order) files via the Web UI. The ESP32-S3 automatically calculates the correct physical track skew and converts them into low-level `.nib` files securely saved to the flash memory.
- **Retro Web UI:** A built-in HTTP server featuring a "Green Phosphor CRT" design with scanlines, allowing you to upload new disks, mount them into the virtual drive, and delete them to free up space.
- **Hot-Swapping:** Disks are mounted into the PSRAM only when the drive motor is off (Drive `/ENABLE` line is HIGH), ensuring safe and crash-free disk swaps.

---

## 🔌 Hardware Architecture & Wiring Summary

The Apple II logic bus operates at **5V**, while the ESP32-S3 is strictly **3.3V tolerant**. To ensure signal integrity and prevent damage, we avoid cheap automatic bidirectional logic level converters (like TXS/TXB series) which often fail on vintage, noisy buses. 

Instead, we use dedicated, unidirectional ICs for clean voltage translation:

1. **Inputs to ESP32 (Apple II 5V → ESP32 3.3V):**
   Uses a **CD4050** (Non-Inverting Hex Buffer). The IC is powered by the ESP32's 3.3V, but its input pins are 5V tolerant, providing extremely fast and clean step-down conversion.
2. **Outputs to Apple II (ESP32 3.3V → Apple II 5V):**
   Uses a **74LS06** (Inverting Hex Buffer with Open-Collector outputs). When the ESP32 outputs HIGH, the 74LS06 pulls the line to GND (Active-Low). When the ESP32 outputs LOW, the 74LS06 enters High-Z (high impedance), allowing the Apple II's internal pull-up resistors to safely pull the line to 5V.

### Wiring Netlist / Pinout

Use this table to wire your prototype or design your PCB (e.g., in KiCad/EasyEDA):

| Apple II Signal (5V) | Direction | Interface Component | ESP32-S3 Pin (3.3V) | Emulator Function |
| :--- | :--- | :--- | :--- | :--- |
| **PHASE 0** | Input | CD4050 (Non-Inverting) | GPIO 4 | Stepper Motor Phase Control |
| **PHASE 1** | Input | CD4050 (Non-Inverting) | GPIO 5 | Stepper Motor Phase Control |
| **PHASE 2** | Input | CD4050 (Non-Inverting) | GPIO 6 | Stepper Motor Phase Control |
| **PHASE 3** | Input | CD4050 (Non-Inverting) | GPIO 7 | Stepper Motor Phase Control |
| **_WREQ** | Input | CD4050 (Non-Inverting) | GPIO 15 | Mode Select (High=Read / Low=Write) |
| **_ENABLE** | Input | CD4050 (Non-Inverting) | GPIO 16 | Drive Select (Active Low / Motor ON) |
| **WDATA** | Input | CD4050 (Non-Inverting) | GPIO 18 | Write Bit Stream from Apple II |
| **RDATA** | Output | 74LS06 (Inverting OC) | GPIO 2 | Read Bit Stream sent to Apple II |
| **WPROT** | Output | 74LS06 (Inverting OC) | GPIO 8 | Write Protect Flag |

*(Note: Ensure all grounds (GND) are tied together between the Apple II, the level shifter ICs, and the ESP32-S3).*

---

## 🛠️ Software Setup & Installation

### Requirements
- **ESP32-S3 Board** with **PSRAM** enabled (Crucial for loading the 232KB `.nib` buffer).
- PlatformIO or Arduino IDE with ESP32 board definitions installed.

### Configuration
1. Open `main.cpp` (or your `.ino` file).
2. Set your local Wi-Fi credentials:
   ```cpp
   const char* ssid = "YOUR_SSID";
   const char* password = "YOUR_PASSWORD";
   ```
3. **Partition Scheme:** Ensure you configure your ESP32-S3 to use a partition scheme with adequate **LittleFS** space (e.g., "16MB Flash, 8MB LittleFS" or similar, depending on your board).
4. **Compile & Upload:** Flash the code to your ESP32-S3.

---

## 🕹️ Usage

1. **Power On:** Connect the emulator to the Apple II Disk Controller card and turn on the Apple II.
2. **Connect to Web UI:** Open the Serial Monitor at `115200` baud to find the IP address of your ESP32-S3 (or check your router). Open that IP address in your web browser.
3. **Upload Disks:** Use the web interface to upload `.dsk`, `.po`, or `.nib` files. The system will automatically convert `.dsk` and `.po` into raw `.nib` GCR formats.
4. **Mount Disks:** Click "Mount" next to a disk image. The ESP32 will queue the disk swap and safely inject it into the virtual drive mechanism once the Apple II motor turns off.
5. **Boot:** Type `PR#6` on your Apple II to boot the mounted disk!

---

## 📝 Supported Formats

- **`.nib`**: Native raw bitstream format. Loads instantly.
- **`.dsk`**: Standard DOS 3.3 logical sector ordering. Automatically converted to physical GCR interleaving upon upload.
- **`.po`**: ProDOS block ordering. Automatically converted using the physical skew required by the Apple II ROM to boot correctly.

---

## 📜 Acknowledgments

- GCR 6-and-2 encoding/nibbilization logic adapted and heavily optimized from community implementations (special thanks to the logic structures seen in `dsk2nib` tools).
- Thanks to the retrocomputing community for preserving the low-level physical timing documentation of the Apple II Disk II controller.

# Apple2DiskESP32

Some part of the code is based in https://github.com/vibr77/AppleIIDiskIIStm32F411
Coded with AI support!
