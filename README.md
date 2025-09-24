# CEDAR-sensor

This repository contains Arduino/ESP32 code for an **air and sound data sensor**.  
It can be used to collect **air quality data** (e.g., particulate matter, gas levels) and **sound data** (e.g., noise levels), and upload the results to a **cloud server** or **remote database** via **4G or Wi-Fi modules**.

---

## 🔧 Components
- External DC power input header + power switch  
- MP1584 step-down (to 5V) power module  
- ESP32 development board / MCU circuit  
- PMS5003 PM2.5 sensor (UART)  
- SHT30 temperature & humidity sensor (I²C)  
- INMP441 digital microphone (I²S)  
- 0.96" OLED display (I²C, SSD1306)  
- ATGM336H GPS module (UART)  
- A7670C 4G Cat.1 communication module (UART, core board model **FS-MCore-A7670CD**)  

---

## 🔋 Power Supply
The system is designed to support **flexible power options**:
- **External DC input (recommended)**: connect via the DC header. Input voltage is stepped down by the **MP1584 module** to provide a stable 5V supply for all components.  
- **Power switch**: included in the circuit to easily turn the system on/off.  
- **Lithium battery (optional)**: the board can also be powered from a lithium battery pack with a charging & protection circuit. This is useful for portable deployments but not required if DC power is available.  

---

## 📡 Wiring Diagram
![Wiring Diagram](Wiring%20Diagram.png)

---

## ⚙️ Setup & Usage

1. **Hardware Preparation**  
   - Assemble all components according to the wiring diagram.  
   - Ensure power is supplied either from an external DC source or from a lithium battery with a charging/protection circuit.  
   - Double-check ESP32 pin connections for UART / I²C / I²S.  

2. **Software Installation**  
   - Install Arduino IDE.  
   - Add ESP32 board support:  
     `File > Preferences > Additional Board Manager URLs` →  
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```  
   - Install required libraries:  
     - `Adafruit_Sensor`  
     - `Adafruit_SHT31`  
     - `Wire`  
     - `SSD1306` or `U8g2`  
     - `WiFi` (ESP32 core)  
     - `TinyGSM` (for 4G module)  

3. **Upload Code**  
   - Connect ESP32 via USB.  
   - Select **Board: ESP32 Dev Module** in Arduino IDE.  
   - Select the correct **Port**.  
   - Upload the code from this repository.  

4. **Running**  
   - Power on the ESP32.  
   - OLED will display PM2.5, temperature, humidity, and sound data.  
   - Data will be sent via **Wi-Fi or 4G** to the server.  
   - Optionally, data can be streamed via USB to Unity software on Mac for visualisation.  

---

## ⚠️ Notes
- Ensure 4G SIM card has an active data plan.  
- Set Wi-Fi SSID and password inside the Arduino code before uploading.  
- Use a charging & protection module if running on lithium battery.  
- Recommended: test each sensor individually before assembling the full system.  
