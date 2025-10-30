# Autonomous Robot for Borobudur Temple Relief Documentation

**Status:** 🚧 Phase 1 ✅ Complete | Phase 2 🔄 In Progress

**Project Goal:** Autonomous mobile robot untuk dokumentasi relief Candi Borobudur dengan navigasi otomatis dan image capture sistematis.

**Current Phase:** Communication System (STM32 ↔ ESP32-CAM) using Modbus RTU protocol

---

## 🎯 The Big Picture

This capstone project aims to build an **autonomous documentation robot** that can:
- Navigate narrow corridors of Borobudur Temple relief galleries
- Capture high-quality images systematically
- Avoid obstacles with ultrasonic sensors
- Upload images to server automatically

### System Components
- **STM32F407** (Master) - Central control, navigation, sensor processing
- **ESP32-CAM** (Slave) - Image capture and WiFi upload
- **HC-SR04 x4** - Ultrasonic obstacle detection (Phase 2)
- **Motor Driver + DC Motors** - Movement control (Phase 2)
- **Flask Server** - Image storage and management

---

## ✅ Phase 1: Communication System (COMPLETED)

A master-slave communication system using Modbus RTU protocol between STM32F407 (Master) and ESP32-CAM (Slave) for remote camera control and image upload.

---

## 🚀 Quick Start

### Hardware Connections

```
STM32F407VG              ESP32-CAM
PB10 (TX)  ─────────►    GPIO13 (RX)
PB11 (RX)  ◄─────────    GPIO12 (TX)
GND        ─────────     GND
```

### Software Setup

**1. Build STM32 (STM32CubeIDE):**
```bash
Open: stm32/semua_inti.ioc
Build Project → Flash to STM32
```

**2. Upload ESP32-CAM (Arduino IDE):**
```bash
Install library: modbus-esp8266 by emelianov
Configure WiFi in esp32cam.ino
Upload to ESP32-CAM
```

**3. Run Server (Python):**
```bash
cd server
pip install flask
python server.py
```

**4. Test:**
- Power on ESP32-CAM → Wait for WiFi connection
- Power on STM32 → Starts sending commands
- Monitor: Images appear in `server/uploaded_images/`

---

## 📁 Project Structure

```
context_capstone/
├── stm32/              # STM32F407 Master
│   ├── Core/
│   │   ├── Inc/modbus_master.h
│   │   └── Src/modbus_master.c
│   └── semua_inti.ioc
├── esp32cam/           # ESP32-CAM Slave
│   └── esp32cam.ino
├── server/             # Flask Upload Server
│   ├── server.py
│   └── uploaded_images/
├── CLAUDE.md           # Complete documentation
└── README.md           # This file
```

---

## 🔧 Key Features

- ✅ **Modbus RTU Protocol** - Industry standard, reliable
- ✅ **CRC16 Error Detection** - Data integrity validation
- ✅ **Camera Control** - Remote trigger via UART
- ✅ **HTTP Upload** - Images sent to server via WiFi
- ✅ **Status Reporting** - SUCCESS/ERROR feedback to master
- ✅ **Error Recovery** - Timeout and retry mechanisms

---

## 📊 Register Map

| Register | Address | Description | Values |
|----------|---------|-------------|---------|
| Command  | 0x0001  | Trigger capture | 0=IDLE, 1=CAPTURE |
| Status   | 0x0002  | Task status | 0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR |
| Error    | 0x0003  | Error details | 0=NONE, 1=CAM, 2=WIFI, 3=UPLOAD |

---

## 🔍 Configuration

### ESP32-CAM (`esp32cam.ino`)

```cpp
// WiFi Settings
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Server URL (update with your IP)
const char* serverURL = "http://192.168.1.9:5000/upload";

// Debug mode (skip upload for testing)
#define SKIP_UPLOAD_FOR_TEST  false
```

### STM32 (Configure via STM32CubeMX)

- **USART1:** 115200 baud (Debug console)
- **USART3:** 9600 baud (Modbus RTU to ESP32)
- **Modbus Timeout:** 1000ms (adjustable in code)

### Server (`server.py`)

```python
# Server port (5000 = no admin needed on Windows)
app.run(host='0.0.0.0', port=5000)

# Images saved to: uploaded_images/image_TIMESTAMP.jpg
```

---

## 🐛 Common Issues

### CRC Error
```
❌ [Modbus] ERROR: CRC Error!
✅ Fix: Check wiring (TX-RX cross, GND connected)
✅ Verify baud rate (both 9600)
```

### Connection Refused
```
❌ [Upload] ERROR: connection refused
✅ Fix: Server running? Windows Firewall allow port 5000?
✅ Test: Open http://192.168.1.9:5000 in browser
```

### Timeout
```
❌ [Modbus] TIMEOUT! ESP32-CAM tidak selesai
✅ Fix: Check ESP32 Serial - task running?
✅ Enable test mode: SKIP_UPLOAD_FOR_TEST = true
```

**Full troubleshooting guide:** See `CLAUDE.md`

---

## 📈 Communication Flow

```
1. STM32 → Write Reg 0x0001 = 1 (CAPTURE)
2. ESP32 → Set Status = BUSY
3. ESP32 → Capture image (OV2640)
4. ESP32 → Upload to server (HTTP POST)
5. ESP32 → Set Status = SUCCESS
6. STM32 → Poll Status every 500ms
7. STM32 → Detect SUCCESS → Continue
```

---

## 🤖 Phase 2: Robot Navigation (IN PROGRESS)

### Next Steps: Autonomous Navigation System

**Objective:** Build autonomous navigation with 4x HC-SR04 ultrasonic sensors and motor control.

### Robot Operation Flow

```
1. Check sensors (A,B,C,D) for obstacles
   ├─ Sensor A close? → Turn Right
   └─ Sensor B close? → Turn Left

2. Path clear? → Capture Image (Modbus command)
   └─ Wait for SUCCESS from ESP32-CAM

3. Move Forward X cm (e.g., 20cm)
   └─ Use encoder for precision

4. Stabilize (delay for vibration)

5. REPEAT → Document entire relief corridor
```

### Components to Add (Phase 2)

| Component | Purpose | Status |
|-----------|---------|--------|
| HC-SR04 x4 | Obstacle detection | 🔄 Testing next |
| L298N Motor Driver | Motor control | ⏳ Planned |
| DC Motors x2 | Movement | ⏳ Planned |
| Encoders | Distance measurement | ⏳ Planned |
| Robot Chassis | Physical structure | ⏳ Planned |
| LiPo Battery | Power supply | ⏳ Planned |

### Sensor Configuration

```
    [Sensor A]     [Sensor B]
    (Front-Left)   (Front-Right)
           \         /
            \       /
         [ ESP32-CAM ]
            /       \
           /         \
    [Sensor C]     [Sensor D]
    (Side-Left)    (Side-Right)
```

### Timeline

- **Week 1-2:** HC-SR04 sensor testing & integration ← **YOU ARE HERE**
- **Week 3-4:** Motor driver & movement control
- **Week 5-6:** Navigation logic implementation
- **Week 7-8:** Full system integration & testing

**Full technical details:** See `CLAUDE.md` - Section "Next Phase: Robot Integration"

---

## 🔮 Future Enhancements (Phase 3+)

- [ ] Machine learning for relief classification
- [ ] Multi-robot coordination
- [ ] Real-time path planning (dynamic obstacles)
- [ ] SLAM (Simultaneous Localization and Mapping)
- [ ] Cloud-based image analysis
- [ ] Mobile app for monitoring

---

## 📚 Documentation

- **Quick Start:** This file (README.md)
- **Complete Guide:** CLAUDE.md (architecture, protocol details, troubleshooting)
- **Code Comments:** Inline documentation in all files

---

## 🛠️ Requirements

**Hardware:**
- STM32F407VGT6 development board
- AI-Thinker ESP32-CAM module
- USB-TTL adapter (for ESP32 programming)
- ST-Link V2 (for STM32 programming)
- Jumper wires

**Software:**
- STM32CubeIDE 1.8.0+
- Arduino IDE 2.x with ESP32 board support
- Python 3.7+ with Flask
- modbus-esp8266 library

---

## 📊 Specifications

| Parameter | Value |
|-----------|-------|
| Communication Protocol | Modbus RTU |
| Physical Layer | UART (RS-232) |
| Baud Rate | 9600 bps |
| Data Format | 8N1 (8 data, no parity, 1 stop) |
| Error Detection | CRC16-Modbus |
| Camera Resolution | VGA (640x480) |
| Image Format | JPEG |
| Upload Protocol | HTTP POST |
| Server Port | 5000 (configurable) |

---

## 🎓 Capstone Project

**Institution:** [Your University]
**Program:** [Your Program]
**Semester:** [Semester]
**Year:** 2025

**Team:**
- [Your Name] - Hardware & Software Engineer
- Claude AI Assistant - Architecture & Documentation

**Development Time:** 3 weeks (October 2025)

---

## 📜 License

Educational use only. Free to use, modify, and extend.

---

## 🙏 Acknowledgments

Special thanks to:
- STMicroelectronics for HAL library
- Espressif for ESP32 platform
- emelianov for modbus-esp8266 library
- Open-source community

---

**For detailed documentation, see:** `CLAUDE.md`

**Status:** ✅ Project completed and working successfully! 🎉
