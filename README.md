# ESP-Scope (ESP32 Fork)

This is a fork of [MatAtBread's ESP-Scope](https://github.com/MatAtBread/esp-scope/), adapted to run on the **Standard ESP32** (e.g., ESP32-WROOM, ESP32-DevKitC) instead of the ESP32-C6.

## Overview
ESP-Scope is a web-based oscilloscope built using the ESP-IDF framework. It allows users to visualize analog signals in real-time through a web browser using the ESP32's internal ADC.

The project launches a dedicated WiFi Access Point (or connects to your existing network) and serves a responsive web interface for signal visualization, complete with trigger controls and sample rate adjustments.

## Hardware Configuration
This fork is pre-configured for standard ESP32 boards (target `esp32`).

| Function | Pin | Note |
| :--- | :--- | :--- |
| **Signal Input** | **GPIO 36** (VP) | ADC1 Channel 0 |
| **Test Signal** | **GPIO 25** | Generates a square wave for testing |
| **Status LED** | **GPIO 2** | Standard onboard LED |
| **Mode Button** | **GPIO 0** | "Boot" button. Hold to reset WiFi settings. |

*Note: Ensure your input voltage does not exceed 3.3V (or the attenuated limit if settings are changed).*

## Features
- **Real-time Visualization**: High-speed plotting via WebSockets.
- **Adjustable Settings**: Sample rate (up to ~83kHz), attenuation, and bit width.
- **Triggering**: Software trigger with adjustable level and rising/falling edge detection.
- **Math**: Delta measurements (Voltage/Time) using mouse-over crosshairs.
- **Test Signal**: Built-in square wave generator on GPIO 25.
- **WiFi Management**: Functions as an AP (`ESP-Scope`) by default; can connect to an existing network via the Web UI.

## Getting Started

### Prerequisites
- [ESP-IDF](https://github.com/espressif/esp-idf) installed and configured (v5.0+ recommended).
- A standard ESP32 development board.

### Building and Flashing

1. **Clone the repository:**
   ```bash
   git clone https://github.com/aarushmagic/esp-scope.git
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
3. Controls:
   * Rate: Adjust the sampling frequency.
   * Atten: Change ADC attenuation (input range).
   * TestHz: Change the frequency of the square wave on GPIO 25.
   * WiFi Button: Configure the device to connect to your home router.
   * Reset Button: Hold the physical BOOT button (GPIO 0) on the device to wipe WiFi settings and return to AP mode.

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