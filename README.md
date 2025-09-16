# CEDAR-sensor

This repository contains Arduino/ESP32 code for an **air and sound data sensor**.  
It can be used to collect **air quality data** (e.g., particulate matter, gas levels) and **sound data** (e.g., noise levels), and upload the results to a **cloud server** or **remote database** via **4G or Wi-Fi modules**.

---

## 🔧 Components
- ESP32 development board  
- PM2.5 sensor module (UART)  
- SHT30 temperature & humidity sensor (I²C)  
- Sound sensor (I²S / Analog)  
- OLED display (I²C)  
- 4G communication module (UART)  
- Power supply (Lithium battery + charging circuit + power switch)  

---

## 📡 Wiring Diagram
![Wiring Diagram](Wiring%20Diagram.png)

---


## ⚙️ Setup & Usage

1. **Hardware Preparation**  
   - Assemble all components according to the wiring diagram.  
   - Ensure lithium battery is charged and connected with a power switch.  
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
