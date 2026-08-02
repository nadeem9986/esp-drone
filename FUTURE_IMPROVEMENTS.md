# ESP-Drone Future Implementations & Hardware Sensor Guide

This document outlines the planned future features, hardware integrations, exact sensor requirements, and software enhancements for the **ESP-Drone** project (`C:\Users\nad\Desktop\esp-drone-master`).

---

## 🚧 Active Obstacle Avoidance & Wall Auto-Brake Feature

### 🌟 How Wall Avoidance / Proximity Pause Works:
1. **Multi-Directional Laser Shield**: Mount **VL53L1X / VL53L0X ToF Laser Distance Sensors** pointing Front, Back, Left, and Right.
2. **Proximity Brake Override**:
   * When flying forward toward a wall, the Front Laser measures distance in real-time ($400\text{cm}$ range).
   * If distance drops below the safety threshold (e.g., $< 50\text{cm}$), the ESP32 **instantly overrides pilot joystick input**, executes a **$-15^\circ$ Reverse Pitch Active Brake** to halt momentum, and locks the drone in a stationary hover.
   * **Wall-Lock Guarantee**: Even if the pilot holds the forward joystick full speed, the flight controller refuses to hit the wall!

### 🔌 Multi-Laser Sensor Hardware Architecture:
* **TCA9548A 8-Channel I2C Multiplexer**: Since VL53L1X sensors share the same I2C address (`0x29`), a $1.00 TCA9548A multiplexer allows connecting up to 8 laser distance sensors simultaneously on `GPIO 21` (SDA) and `GPIO 22` (SCL).

---

## 📻 Long-Range RC Radio Options: ESP-NOW vs. nRF24L01+PA+LNA

### 1. ESP-NOW (Native ESP32 2.4GHz Radio)
* **Protocol**: Built-in ESP32 Wi-Fi hardware.
* **Range**: **150m - 300m** (PCB antenna) / **500m+** (External IPEX antenna).
* **Latency**: Ultra-fast **< 2ms**.
* **Pros**: No extra radio module hardware required on the drone.

### 2. nRF24L01+PA+LNA (1.5km - 2km Long Range External Radio)
* **Protocol**: Nordic nRF24 2.4GHz RF over SPI (`SCK 18`, `MISO 19`, `MOSI 23`, `CSN 5`, `CE 4`).
* **Range**: **1,500m – 2,000m (1.5km - 2.0km Line-of-Sight)**.
* **Power Output**: **+20 dBm (100mW)** boosted by Power Amplifier (PA) & Low-Noise Amplifier (LNA) with SMA antenna.

---

## 🛒 Exact Sensor Shopping List & Technical Specifications

| Sensor Category | Exact Sensor Module Part Number | Bus Interface | Function / Purpose |
| :--- | :--- | :--- | :--- |
| **1. Primary IMU (Attitude)** | **MPU-6050 (GY-521)** or **BMI270** | I2C (`GPIO 21`, `GPIO 22`) | 6-Axis Gyro + Accel for 250Hz Mahony Quaternion stabilization |
| **2. Altitude Hold (Pressure)** | **SPL06-001** (or BMP280) | I2C (`GPIO 21`, `GPIO 22`) | Air pressure sensor for altitude estimation ($\pm 5\text{cm}$ resolution) |
| **3. Obstacle Avoidance Lasers**| **VL53L1X** (Time-of-Flight Laser) | I2C (`TCA9548A`) | Front, Back, Left, Right laser rangefinders for active Wall Auto-Brake |
| **4. I2C Bus Multiplexer** | **TCA9548A I2C Multiplexer** | I2C (`GPIO 21`, `GPIO 22`) | Connects up to 8 laser distance sensors on one I2C bus |
| **5. Position Hold (2D Lock)** | **PMW3901 Optical Flow Module** | SPI (`GPIO 18, 19, 23, 5`) | 100FPS downward camera tracking ground patterns to freeze 2D drift |
| **6. Compass (Absolute Heading)** | **QMC5883L / HMC5883L** | I2C (`GPIO 21`, `GPIO 22`) | 3-Axis Magnetometer for true North compass orientation lock |
| **7. Long Range Radio** | **nRF24L01+PA+LNA Module** | SPI (`GPIO 18, 19, 23, 5, 4`) | 1.5km - 2km Long Range RC control |

---

## 🔮 Future Feature Roadmap

### 🛠️ 1. Advanced Flight Control & Sensor Fusion
- **Barometer & Laser Altitude Hold (Z-Axis Lock)**:
  - Fuse **SPL06-001** pressure + **VL53L1X** laser distance into the Extended Kalman Filter (EKF) for auto-hover height lock when throttle is released.
- **Optical Flow 2D Position Hold (X/Y-Axis Lock)**:
  - Integrate **PMW3901** optical flow camera deck to track ground displacement vectors and freeze 2D position indoors without GPS.

### 🧠 2. Autonomous Flight & Safety Systems
- **Active Obstacle Avoidance & Auto-Brake**: Front/Back/Left/Right laser proximity detection with auto-reverse pitch braking.
- **Auto Takeoff & Auto Landing**: One-touch automated takeoff to $1.0\text{m}$ hover and smooth proximity-controlled landing.
- **Wireless & Low-Voltage Fail-Safe**: Automated emergency descent on Radio loss ($>1.5\text{s}$) or low battery ($<3.3\text{V}$).
- **Geofencing & 360° Acro Flips**: Programmable height/distance limits and automated barrel roll / backflip assistance.

### 🎛️ 3. Ground Control Station (GCS) & Tuning
- **Live Wireless PID Tuning**: Adjust Roll, Pitch, and Yaw PID gains live via Web UI sliders over Wi-Fi.
- **On-Board Blackbox Data Logger**: Log 50Hz flight telemetry to SPI Flash (`LittleFS`) and play back 3D trajectories.

### 📷 4. Hardware Payloads & Enhancements
- **FPV Video Stream**: Stream live 60FPS video directly into the Web GCS using an **ESP32-CAM** module.
- **Addressable RGB Status LEDs & Buzzer**: **WS2812B NeoPixel** strips for flight status and piezo acoustic "Find My Drone" beacon.
