# Fitness Tracker with Wi-Fi 6 Connectivity

An embedded system for real-time physical activity tracking with direct **Wi-Fi 6 (IEEE 802.11ax)** connectivity, fully independent of smartphones. Built on the **nRF7002DK** platform with **Zephyr RTOS**, this project integrates motion sensing, graphical interface, and secure cloud communication into a standalone wearable prototype.

---

## Features

- **Wi-Fi 6 Connectivity**: Direct internet access without a smartphone bridge.
- **Real-Time Motion Tracking**: Using a **3-axis MEMS accelerometer (MMA7361L)**.
- **Graphical User Interface**: Circular **GC9A01 TFT display** with **CST816D capacitive touch sensor**, powered by **LVGL**.
- **Secure Data Transmission**: Activity data published via **MQTT over TLS (port 8883)** with predefined topics.
- **Activity Recognition**: Step detection, distance estimation, calories, speed, and walk/run classification.
- **Modular Design**: Easy integration of additional sensors and future software extensions.

---

## System Architecture

- **Hardware**:  
  - Nordic Semiconductor **nRF7002DK** (nRF5340 dual-core MCU + nRF7002 Wi-Fi 6 companion IC)  
  - **MMA7361L accelerometer** (3-axis, analog output)  
  - **GC9A01 circular TFT LCD** (240x240, SPI)  
  - **CST816D capacitive touch controller** (I²C)  

- **Software**:  
  - **Zephyr RTOS** via **nRF Connect SDK**  
  - **MQTT client** with TLS encryption  
  - **LVGL graphics library** for GUI  
  - Real-time step detection and basic activity classification  

---

## Implementation Overview

1. **Hardware Integration**: All modules connected to the nRF7002DK via SPI, I²C, ADC, and QSPI interfaces.  
2. **Sensor Processing**: Accelerometer signals digitized, filtered, and processed for step detection.  
3. **GUI Development**: Two-screen LVGL interface (main + details) with animations and real-time data updates.  
4. **MQTT Communication**: Secure TLS channel on port 8883 for publishing sensor data and receiving control commands.  
5. **Testing**: Prototyped on a breadboard; verified live data transfer to a Mosquitto MQTT broker.  

---

## Example Data Flow

1. User moves → accelerometer captures acceleration → MCU detects step.  
2. Step count + activity metrics updated on GUI.  
3. Data published to MQTT broker (topic: `wifi/fund/nrf/publish/fitness/topic1`).  
4. Server/PC client subscribed to topic receives real-time updates.  

---

## Future Improvements

- Adaptive step length calibration and advanced activity recognition.  
- Digital MEMS sensors with integrated filtering.  
- Power optimization (display sleep, sampling rate control).  
- Robust MQTT features (QoS, Last Will, auto-reconnect).  
- Enhanced TLS certificate management.  

---

## Getting Started

### Requirements
- **Hardware**: nRF7002DK, MMA7361L, GC9A01 TFT LCD with CST816D touch.  
- **Software**: nRF Connect SDK (≥ 3.0.1), Zephyr RTOS, VS Code with nRF Connect extension.  
- **MQTT Broker**: e.g., [Eclipse Mosquitto](https://mosquitto.org).  

### Build & Flash
```bash
west build -b nrf7002dk_nrf5340_cpuapp .
west flash
```

### Run MQTT Subscriber on PC
```bash
mosquitto_sub -h <broker-address> -t wifi/fund/nrf/publish/fitness/topic1 --cafile ca.crt -p 8883
```

---

## Author

**Frane Krapić**  
Technical Faculty, University of Rijeka  
September 2025

---
