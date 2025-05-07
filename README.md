# ESP32 Smart Curtain Control System

### Overview
This project implements an **automated curtain control system** using **ESP32** and **FreeRTOS**. The system adjusts curtains based on **temperature** and **light levels**, monitors **motion**, and ensures **system health** tracking.

### Features
-  **Automatic Curtain Control** (Based on temperature)
-  **Light-Based Control** (Lights turn on/off automatically)
-  **Motion Detection** (Alerts via LCD display)
-  **PID-Controlled Motor** (Smooth & efficient movement)
-  **System Health Monitoring** (Heap memory & task stack usage)

### Hardware Components
- ESP32 Development Board
- DHT11 Temperature Sensor
- LDR (Light Sensor)
- PIR Motion Sensor
- HD44780 LCD Display (I2C)
- Curtain Motor with H-Bridge Driver

### How It Works
1. Sensors collect temperature, light, and motion data.
2. Decision-making logic triggers curtain movement via **semaphores**.
3. **PID control** ensures smooth motor movement.
4. LCD updates system status in **real-time**.
5. A dedicated **System Health Task** monitors memory usage.

