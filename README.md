# ESP-Scope (ESP32 Fork)

This is a fork of [MatAtBread's ESP-Scope](https://github.com/MatAtBread/esp-scope/), adapted to run on the **Standard ESP32** (e.g., ESP32-WROOM, ESP32-DevKitC) instead of the ESP32-C6.

## ⚠️ Known Limitations
**Frequency Mismatch:**
There is currently a known issue where the frequency of the generated sine wave does not properly calibrate.
* **Symptoms:** You will see the correct wave **shape** (Sine), but the frequency of the wave may differ significantly from the frequency you requested in the "Func Gen" tab. Attempting to use the calibration feature may not fix it.
* **Status:** Under investigation.

## Overview
ESP-Scope is a web-based oscilloscope built using the ESP-IDF framework. It allows users to visualize analog signals in real-time through a web browser using the ESP32's internal ADC. It now includes a built-in **Function Generator** capable of producing Square, Sine, and Triangle waves.

The project launches a dedicated WiFi Access Point (or connects to your existing network) and serves a responsive web interface for signal visualization and generation.

## Hardware Configuration
This firmware is configured for standard ESP32 boards (target `esp32`).

| Function | Pin | Description |
| :--- | :--- | :--- |
| **Signal Input** | **GPIO 36** (VP) | ADC1 Channel 0. The probe input. |
| **Func Gen Output**| **GPIO 25** | DAC1. Outputs Square, Sine, or Triangle waves. |
| **Status LED** | **GPIO 2** | Standard onboard LED. |
| **Mode Button** | **GPIO 0** | "Boot" button. Hold to reset WiFi settings. |

*Note: Ensure your input voltage on GPIO 36 does not exceed 3.3V.*

## Features
- **Real-time Visualization**: High-speed plotting via WebSockets.
- **Function Generator**:
  - **Square Wave**: Generated via PWM (LEDC).
  - **Sine Wave**: Generated via internal Hardware Cosine Generator (CWG).
  - **Triangle Wave**: Generated via DAC software timer.
  - *Note: All outputs are on GPIO 25.*
- **Adjustable Scope Settings**: Sample rate, attenuation, and bit width.
- **Triggering**: Software trigger with adjustable level and rising/falling edge detection.
- **WiFi Management**:
  - Functions as an AP (`ESP-Scope`) by default.
  - Can connect to an existing network via the Web UI.
  - Captive portal support for easy configuration.
- **Persistent Calibration**: Save voltage and frequency correction factors to NVS to fix hardware offsets.

## Calibration
This firmware includes a calibration system to correct for hardware inaccuracies (e.g., VRef offsets, clock drift). Calibration data is saved to Non-Volatile Storage (NVS) and persists across reboots.

### Voltage Calibration (Oscilloscope)
To correct voltage readings (e.g., if 3.3V reads as 3.1V):
1.  Navigate to the Oscilloscope tab.
2.  Click the Calibrate Voltage button (wrench icon &#128736;).
3.  Apply a stable voltage to the input pin.
4.  Click `+ Add Point`.
5.  Enter the Measured value (what the scope currently shows) and the Actual value (measured by a reliable multimeter).
6.  Repeat for at least 2 points (e.g., one low voltage and one high voltage).
7.  Click `Calculate & Save`. The system calculates a linear regression ($y = mx + c$) and applies it to future readings.

### Frequency Calibration (Function Generator)
To fix frequency mismatches:
1.  Navigate to the Func Gen tab.
2.  Select the Wave type you want to calibrate (Square, Sine, or Triangle).
3.  Set a Freq (e.g., 1000 Hz) and turn it on.
4.  Click the Calibrate Frequency button (wrench icon &#128736;).
5.  Measure the real output frequency using an external counter or the scope itself.
6.  Enter the Target Freq (e.g., 1000) and the Actual Output (e.g., 950).
7.  Click Update Factor. The system calculates a correction ratio and saves it for that specific wave type.

*Note: You can reset all calibration data to factory defaults via the "Reset" button (circular arrow &olarr;) in the Function Generator tab.*

## Getting Started

### Prerequisites
- [ESP-IDF](https://github.com/espressif/esp-idf) installed and configured (v5.0+ recommended).
- A standard ESP32 development board.

### Building and Flashing

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/aarushmagic/esp-scope.git](https://github.com/aarushmagic/esp-scope.git)
   cd esp-scope
   ```   
2. **Set the target to ESP32:**
   ```bash
   idf.py set-target esp32
   ```
3. **Build the project:**
   ```bash
   idf.py build
   ```
4. **Flash and Monitor:**
   ```bash
   idf.py -p [PORT] flash monitor
   ```
   *(Replace [PORT] with your device path, e.g., /dev/ttyUSB0 or COM3)*

### Usage
1. Connect:
   * On first boot, the device creates a WiFi network named ESP-Scope. Connect to it (no password by default).
   * Alternatively, monitor the Serial output to see the IP address if you configured station mode.
2. Open the Interface:
   * Navigate to http://esp-scope (or http://192.168.4.1 if using the AP).
   * If the page does not load, try disabling mobile data or disconnect from other networks.
3. Oscilloscope Tab:
   * Rate: Adjust the sampling frequency.
   * Atten: Change ADC attenuation (input range).
   * WiFi Button: Configure the device to connect to your home router.
   * Reset Button: Hold the physical BOOT button (GPIO 0) on the device to wipe WiFi settings and return to AP mode.
4. Func Gen Tab:
   * Wave: Select Square, Sine, or Triangle.
   * Freq: Set the target frequency (see Limitations above).
   * Amp: Adjust amplitude (0-100%).

### LED Status Codes
* Solid: Starting up or connecting.
* 1s Flash: AP Mode (Connect to "ESP-Scope").
* Slow Blinks: Connected to WiFi station.
* Rapid Blinks: Sending WebSocket data to a client

## Credits
* Original project by [MatAtBread](https://github.com/MatAtBread).
* Adapted for ESP32 Standard.

## License
This project is licensed under the MIT License. See the LICENSE file for details.This project is licensed under the MIT License. See the LICENSE file for details.