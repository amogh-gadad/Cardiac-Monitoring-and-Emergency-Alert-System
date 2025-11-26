# Cardiac-Monitoring-and-Emergency-Alert-System

## 🌟 1. Project Overview

This project implements a critical, automated health monitoring system designed to detect severe cardiac anomalies (**Bradycardia**: BPM < 60 or **Tachycardia**: BPM > 120) and immediately alert designated emergency contacts via **SMS** and a voice call.

The system is built on the **Raspberry Pi Pico W** and utilizes an **AD8232 ECG sensor** for precise heart signal monitoring and an **A7670E GSM/GPS modem** for communication and location services.

### Key Features:

* **Real-time ECG Monitoring:** Uses the AD8232 and MCP3008 for accurate, continuous BPM calculation.
* **Automatic Alert:** Triggers an emergency sequence if the anomaly persists for **5 seconds** (ANOMALY\_WINDOW\_MS).
* **Location Fallback:** Prioritizes **Live GPS Fix**; falls back to **Last Known Fix**; and finally sends **Cell Tower LAC/CID** if GPS is unavailable.
* **Automated Response:** Directly initiates SMS alerts and a voice call (no manual cancellation mechanism).

---

## 🩺 2. Alert Logic and Flow

The project's operational sequence follows the process detailed in the flow diagram:

<img src="assets/Flow_Diagram.png" alt="Flow Diagram" width="50%"/>

1.  **Monitoring:** The system continuously calculates the BPM from the AD8232 sensor.
2.  **Anomaly Detection:** If BPM < 60 or BPM > 120, an internal timer starts.
3.  **Trigger Condition:** If the anomaly persists for **5000 milliseconds (5 seconds)**, the alert sequence begins.
4.  **Alert Sequence:**
    * Buzzer sounds for 3 seconds.
    * OLED displays "!!! AUTO ALERT !!!".
    * **SMS is sent** to both emergency numbers with location data.
    * System waits 5 seconds (`CALL_AFTER_SMS_DELAY`) to ensure SMS delivery before calling.
    * **Voice Call** is placed to the primary number ($\text{EMERGENCY1}$).
5.  **Halt:** The system enters a perpetual loop, displaying a **"RESET REQUIRED"** message until power-cycled.

---

## 🛠️ 3. Hardware Requirements

| Component | Function | Notes |
| :--- | :--- | :--- |
| **Microcontroller** | Raspberry Pi Pico W | Core processing unit. |
| **ECG Sensor** | AD8232 | Heart rate signal acquisition. |
| **ADC** | MCP3008 (10-bit) | External ADC necessary for reading the AD8232 analog output via SPI. |
| **Communication** | A7670E GSM/GPS Modem | Handles UART for AT commands, SMS, Call, and GPS logging. **Requires external high-current power.** |
| **Display** | 128x64 OLED | I2C display for real-time status and alerts. |
| **Alert** | Active Buzzer | Local audible alarm. |

---

## 🔌 4. Wiring and Pin Configuration (Critical)

The system architecture and connections are detailed in the block diagram, with the following precise pin assignments:

<img src="assets/Block_Diagram.png" alt="Block Diagram" width="50%"/>

The pin assignments were carefully chosen to resolve the hardware conflict between the MCP3008 (SPI) and the OLED (I2C):

| Component | Pin Function | Pico W **GPIO** Pin | Pico W **Physical Pin** (`P#`) | Communication Protocol |
| :--- | :--- | :--- | :--- | :--- |
| **MCP3008 CLK** | SPI Clock | **GPIO 2** | **P4** | **Software SPI** |
| **MCP3008 DIN** | SPI Data Out | **GPIO 3** | **P5** | **Software SPI** |
| **MCP3008 DOUT**| SPI Data In | **GPIO 4** | **P6** | **Software SPI** |
| **MCP3008 CS** | Chip Select | **GPIO 5** | **P7** | **Software SPI** |
| **OLED SDA** | I2C Data | **GPIO 8** | **P11** | **I2C 0** (Shifted to resolve conflict) |
| **OLED SCL** | I2C Clock | **GPIO 9** | **P12** | **I2C 0** (Shifted to resolve conflict) |
| **GSM TX/RX**| UART 0 | **GPIO 0**, **GPIO 1** | **P1, P2** | **UART** Communication |
| **GPS RX** | GPS NMEA Input | **GPIO 6** | **P9** | **SoftwareSerial RX** (Connects to GPS module's TX pin) |
| **Buzzer** | Output Driver | **GPIO 27** | **P32** | **Digital OUT** |
| **AD8232** | Analog Out | $\rightarrow$ MCP3008 CH0 | **N/A** | Connect to $\text{ADC}$ input $\text{CH0}$. |

---

## 💻 5. Software Setup and Tuning

### Dependencies (Libraries)

Ensure the following libraries are installed in your Arduino IDE or PlatformIO environment:

* `Wire.h` (Standard $\text{I2C}$ Library)
* `Adafruit_SSD1306.h`
* `Adafruit_GFX.h`
* `TinyGPS++.h`
* `SoftwareSerial.h`

### Tuning Constants

You may adjust these values for calibration or testing purposes:

* `BPM_ANOMALY_LOW`: 60 (Bradycardia threshold)
* `BPM_ANOMALY_HIGH`: 120 (Tachycardia threshold)
* `ANOMALY_WINDOW_MS`: 5000 (5 seconds of persistent anomaly before triggering)
* `CALL_AFTER_SMS_DELAY`: 5000 (5 seconds delay after SMS before initiating the call)

**Note:** Update the phone numbers (`EMERGENCY1`, `EMERGENCY2`) at the top of the file before compiling.
