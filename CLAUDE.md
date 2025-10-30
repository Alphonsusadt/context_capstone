# Autonomous Robot for Borobudur Temple Relief Documentation

**Project Status:** 🚧 **PHASE 1 COMPLETED** (Communication System) | **PHASE 2 IN PROGRESS** (Robot Integration)

---

## 📋 Table of Contents

1. [Complete Project Overview](#complete-project-overview)
2. [Current Phase: Communication System](#current-phase-communication-system)
3. [System Architecture](#system-architecture)
4. [Hardware Setup](#hardware-setup)
5. [Software Implementation](#software-implementation)
6. [Communication Protocol](#communication-protocol)
7. [Deployment Guide](#deployment-guide)
8. [Troubleshooting](#troubleshooting)
9. [Next Phase: Robot Integration](#next-phase-robot-integration)
10. [Future Development](#future-development)

---

## 🎯 Complete Project Overview

### **🏛️ The Big Picture: Borobudur Temple Relief Documentation Robot**

**Final Goal:**
Membangun **autonomous mobile robot** yang dapat bergerak secara otomatis di gang-gang relief Candi Borobudur, mengambil foto relief secara sistematis, dan mengupload ke server untuk dokumentasi digital.

### **Problem Statement**
Candi Borobudur memiliki ribuan panel relief yang tersebar di gang-gang sempit dengan belokan-belokan. Dokumentasi manual memakan waktu lama dan tidak konsisten. Dibutuhkan sistem otomatis untuk:
- Navigasi di gang sempit dengan obstacle avoidance
- Capture gambar relief secara konsisten (jarak, angle, lighting)
- Upload dan organize data secara otomatis
- Coverage area yang luas dengan minimal human intervention

### **Solution: 3-Subsystem Robot**

```
┌─────────────────────────────────────────────────────────┐
│         AUTONOMOUS RELIEF DOCUMENTATION ROBOT           │
│                                                          │
│  ┌──────────────┐   ┌──────────────┐   ┌─────────────┐│
│  │  Navigation  │   │    Vision    │   │   Upload    ││
│  │   Subsystem  │   │   Subsystem  │   │  Subsystem  ││
│  │              │   │              │   │             ││
│  │ • Ultrasonic │   │ • ESP32-CAM  │   │ • WiFi      ││
│  │ • Motor      │   │ • OV2640     │   │ • HTTP      ││
│  │ • Encoder    │   │ • Trigger    │   │ • Server    ││
│  └──────┬───────┘   └──────┬───────┘   └──────┬──────┘│
│         │                  │                   │       │
│         └──────────────────┼───────────────────┘       │
│                            │                           │
│                   ┌────────▼────────┐                  │
│                   │   STM32F407     │                  │
│                   │ (Master Control)│                  │
│                   └─────────────────┘                  │
└─────────────────────────────────────────────────────────┘
```

### **Robot Operation Flow**

```
START
  │
  ├─► Check Ultrasonic Sensors (A, B, C, D)
  │
  ├─► Path Clear? → Continue
  │
  ├─► Send CAPTURE command to ESP32-CAM (Modbus RTU)
  │   └─ Wait for SUCCESS response
  │
  ├─► Move Forward X cm (Motor Control)
  │   └─ Use encoder for precise distance
  │
  ├─► Delay for stabilization
  │
  └─► REPEAT from start
```

### **Project Phases**

#### **✅ Phase 1: Communication System (COMPLETED - October 2025)**
**Goal:** Establish reliable communication between STM32 and ESP32-CAM
- ✅ Modbus RTU protocol implementation
- ✅ Camera trigger and capture
- ✅ Image upload to server
- ✅ Status reporting and error handling
- ✅ Testing and validation

**Deliverables:**
- Working Modbus communication
- ESP32-CAM capturing images on command
- HTTP upload to Flask server
- Complete documentation

#### **🚧 Phase 2: Navigation System (IN PROGRESS)**
**Goal:** Implement autonomous navigation with obstacle avoidance
- 🔄 HC-SR04 ultrasonic sensor integration (4 sensors)
- ⏳ Motor driver control (L298N or similar)
- ⏳ Precise movement control (encoder feedback)
- ⏳ Hardcoded navigation logic for Borobudur relief corridors
- ⏳ Testing in simulated environment

**Components:**
- 4x HC-SR04 Ultrasonic Sensors (Front-Left, Front-Right, Side-Left, Side-Right)
- 2x DC Motors with encoders
- 1x Motor Driver (L298N or TB6612FNG)
- Power management system

#### **⏳ Phase 3: System Integration (PLANNED)**
**Goal:** Integrate all subsystems into working robot
- Robot chassis assembly
- Power distribution
- Sensor calibration
- Movement testing
- End-to-end workflow testing


---

## 📍 Current Phase: Communication System

### **Objective**
Membangun sistem komunikasi antara **STM32F407VG (Master)** dan **ESP32-CAM (Slave)** untuk kontrol kamera dan upload gambar ke server via WiFi, dengan fokus pada protokol komunikasi yang reliable dan terstandarisasi.

**This is the foundation for the robot's vision subsystem.**

### **Use Case**
STM32 mengirim perintah "CAPTURE" ke ESP32-CAM. ESP32-CAM akan:
1. Mengambil gambar dengan kamera OV2640
2. Upload gambar ke server via WiFi
3. Mengembalikan status (SUCCESS/ERROR) ke STM32
4. STM32 melanjutkan ke perintah berikutnya setelah menerima konfirmasi

### **Key Features**
- ✅ Modbus RTU communication protocol (industri standard)
- ✅ Master-slave architecture dengan STM32 sebagai master
- ✅ Camera capture dengan ESP32-CAM (VGA 640x480 JPEG)
- ✅ HTTP upload ke server via WiFi
- ✅ Error handling dan status reporting
- ✅ Built-in CRC16 error detection
- ✅ Automatic polling dan timeout mechanism

---

## 🏗️ System Architecture

### **High-Level Architecture**

```
┌─────────────┐         Modbus RTU          ┌──────────────┐
│             │◄────────(UART 9600)─────────►│              │
│  STM32F407  │      TX: PB10 → RX: GPIO13   │  ESP32-CAM   │
│   (Master)  │      RX: PB11 ← TX: GPIO12   │   (Slave)    │
│             │         Common GND           │              │
└─────────────┘                              └──────┬───────┘
      │                                             │
      │ USART1 (Debug)                             │ WiFi
      │ 115200 baud                                 │
      ▼                                             ▼
┌─────────────┐                              ┌──────────────┐
│  PC Serial  │                              │ HTTP Server  │
│   Monitor   │                              │  (Flask)     │
└─────────────┘                              └──────────────┘
```

### **Communication Flow**

```
[STM32 Master]                          [ESP32-CAM Slave]
      │                                        │
      ├─ Write Reg 0x0001 = 1 (CAPTURE) ────►│
      │◄──────── Modbus ACK ───────────────────┤
      │                                        ├─ Set status = BUSY (0x0001)
      │                                        ├─ Capture image
      │                                        ├─ Upload to server (HTTP POST)
      │                                        └─ Set status = SUCCESS (0x0002)
      │                                            or ERROR (0x0003)
      ├─ Poll: Read Reg 0x0002 (Status) ─────►│
      │◄──────── Return Status ─────────────────┤
      │  (Every 500ms, max 30s)                │
      │                                        │
      ├─ Detect SUCCESS ─────────────────────►│
      └─ Continue to next task                │
```

### **Register Mapping (Modbus Holding Registers)**

| Register | Address | Access | Description | Values |
|----------|---------|--------|-------------|---------|
| Command  | 0x0001  | R/W    | Command register | 0=IDLE, 1=CAPTURE |
| Status   | 0x0002  | R      | Status register  | 0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR |
| Error Code | 0x0003 | R     | Error details    | 0=NONE, 1=CAM_FAIL, 2=WIFI_FAIL, 3=UPLOAD_FAIL |

---

## 🔌 Hardware Setup


### **Pin Configuration**

#### STM32F407VG

| Pin | Function | Connected To | Notes |
|-----|----------|--------------|-------|
| PA9 | USART1 TX | PC Serial (Debug) | 115200 baud |
| PA10 | USART1 RX | PC Serial (Debug) | 115200 baud |
| PB10 | USART3 TX | ESP32 GPIO13 (RX) | Modbus RTU, 9600 baud |
| PB11 | USART3 RX | ESP32 GPIO12 (TX) | Modbus RTU, 9600 baud |
| PC13 | GPIO Output | LED | Status indicator |
| GND | Ground | ESP32 GND | Common ground |

#### ESP32-CAM

| Pin | Function | Connected To | Notes |
|-----|----------|--------------|-------|
| GPIO13 | UART RX (Serial2) | STM32 PB10 (TX) | Modbus RTU input |
| GPIO12 | UART TX (Serial2) | STM32 PB11 (RX) | Modbus RTU output |
| GPIO0 | Camera XCLK | OV2640 | 20MHz clock |
| GPIO4 | Flash LED | LED | Camera flash (optional) |
| 5V | Power | Power supply | Via voltage regulator |
| GND | Ground | STM32 GND | Common ground |
| 3V3 | Power output | - | Internal 3.3V for camera |


---

## 💻 Software Implementation

### **Project Structure**

```
D:\context_capstone\
│
├── stm32/                          # STM32 Master Project (Phase 1 & 2)
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── modbus_master.h     # ✅ Phase 1: Modbus Master header
│   │   │   ├── hcsr04.h            # 🔄 Phase 2: HC-SR04 sensor driver (NEXT)
│   │   │   ├── motor_driver.h      # ⏳ Phase 2: Motor control (PLANNED)
│   │   │   └── navigation.h        # ⏳ Phase 2: Navigation logic (PLANNED)
│   │   └── Src/
│   │       ├── main.c              # Main application logic
│   │       ├── modbus_master.c     # ✅ Phase 1: Modbus RTU implementation
│   │       ├── hcsr04.c            # 🔄 Phase 2: HC-SR04 implementation (NEXT)
│   │       ├── motor_driver.c      # ⏳ Phase 2: Motor control (PLANNED)
│   │       ├── navigation.c        # ⏳ Phase 2: Navigation logic (PLANNED)
│   │       └── stm32f4xx_hal_msp.c # HAL peripheral init
│   ├── Drivers/                    # HAL drivers
│   ├── semua_inti.ioc              # STM32CubeMX project file
│   └── STM32F407VGTX_FLASH.ld      # Linker script
│
├── esp32cam/                       # ESP32-CAM Slave Project (Phase 1)
│   ├── esp32cam.ino                # ✅ Main Arduino sketch (Modbus Slave + Camera)
│   ├── board_config.h              # Board selection (AI-THINKER)
│   ├── camera_pins.h               # Camera pin definitions
│   ├── app_httpd.cpp               # HTTP server (available, not used)
│   └── partitions.csv              # Memory partitions
│
├── server/                         # Flask Upload Server (Phase 1)
│   ├── server.py                   # ✅ Flask application
│   ├── test_upload.py              # ✅ Connectivity test script
│   └── uploaded_images/            # Image storage folder (auto-created)
│       └── image_YYYYMMDD_HHMMSS.jpg  # Captured images
│
├── docs/                           # Documentation (auto-organized)
│   ├── CLAUDE.md                   # ✅ Complete technical documentation
│   ├── README.md                   # ✅ Quick start guide
│   ├── PHASE2_HCSR04_TESTING.md    # ✅ HC-SR04 sensor testing guide
│   └── images/                     # Screenshots, diagrams (if any)
│
└── (root files)
    ├── CLAUDE.md                   # ✅ Master documentation (this file)
    ├── README.md                   # ✅ Project overview & quick start
    └── PHASE2_HCSR04_TESTING.md    # ✅ Sensor testing reference (NEXT TASK)
```

**Legend:**
- ✅ = Completed and working
- 🔄 = In progress / Next task
- ⏳ = Planned for future

### **Key Software Components**

#### 1. STM32 Modbus Master (`modbus_master.c`)

**Features:**
- Modbus RTU protocol implementation
- Function Code 0x03 (Read Holding Registers)
- Function Code 0x06 (Write Single Register)
- CRC16-Modbus calculation and validation
- Timeout handling (default 1000ms)
- High-level API functions

**Key Functions:**
```c
// Initialize Modbus Master
void ModbusMaster_Init(ModbusMaster_t *modbus, UART_HandleTypeDef *huart, uint32_t timeout_ms);

// Send CAPTURE command
int ModbusMaster_SendCaptureCommand(ModbusMaster_t *modbus);

// Read status register
int ModbusMaster_ReadStatus(ModbusMaster_t *modbus, uint16_t *status);

// Wait for task completion (with polling)
int ModbusMaster_WaitForCompletion(ModbusMaster_t *modbus, uint32_t max_wait_ms);

// CRC16 calculation
uint16_t ModbusMaster_CRC16(uint8_t *buffer, uint16_t length);
```

**Error Codes:**
```c
#define MODBUS_OK                   0
#define MODBUS_ERR_TIMEOUT          -1
#define MODBUS_ERR_CRC              -2
#define MODBUS_ERR_EXCEPTION        -3
#define MODBUS_ERR_INVALID_RESPONSE -4
```

#### 2. ESP32-CAM Modbus Slave (`esp32cam.ino`)

**Features:**
- Modbus RTU slave implementation (using modbus-esp8266 library)
- Camera capture with OV2640 (VGA 640x480 JPEG)
- HTTP POST upload to server
- Non-blocking task execution
- Status and error reporting
- Test mode for debugging (skip upload)

**Key Variables:**
```cpp
// Modbus slave ID
#define MODBUS_SLAVE_ID     0x01

// Register values
volatile uint16_t reg_command = CMD_IDLE;
volatile uint16_t reg_status = STATUS_IDLE;
volatile uint16_t reg_error_code = ERR_NONE;

// Modbus instance
ModbusRTU mb;
```

**Callback Functions:**
```cpp
// Called when Master writes to register
uint16_t cbWrite(TRegister* reg, uint16_t val);

// Called when Master reads from register
uint16_t cbRead(TRegister* reg, uint16_t val);
```

**Task Flow:**
```cpp
void captureAndUpload() {
    // 1. Set status to BUSY
    // 2. Capture image with esp_camera_fb_get()
    // 3. Upload to server via HTTPClient
    // 4. Set status to SUCCESS or ERROR
    // 5. Update error code if failed
    // 6. Process Modbus requests during task (mb.task())
}
```

**CRITICAL:** `mb.task()` must be called frequently during blocking operations to allow Master polling!

#### 3. Flask Upload Server (`server.py`)

**Features:**
- Root endpoint (/) for status check
- Upload endpoint (/upload) for POST requests
- Automatic timestamp-based filename
- Folder auto-creation
- Request logging with client IP
- Error handling

**Endpoints:**

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Status page (shows server info) |
| `/upload` | POST | Image upload endpoint (returns "OK") |

**Example Usage:**
```bash
# Start server
python server.py

# Test upload (curl)
curl -X POST http://192.168.1.9:5000/upload --data-binary @image.jpg
```

---

## 🔄 Communication Protocol

### **Modbus RTU Frame Format**

**Request Frame (Master → Slave):**
```
┌────────┬──────────┬─────────────────┬─────────┬─────────┐
│ Slave  │ Function │   Data Bytes    │ CRC Low │ CRC High│
│  ID    │   Code   │   (N bytes)     │         │         │
└────────┴──────────┴─────────────────┴─────────┴─────────┘
  1 byte    1 byte      Variable         1 byte    1 byte
```

**Response Frame (Slave → Master):**
```
┌────────┬──────────┬─────────────────┬─────────┬─────────┐
│ Slave  │ Function │   Data Bytes    │ CRC Low │ CRC High│
│  ID    │   Code   │   (N bytes)     │         │         │
└────────┴──────────┴─────────────────┴─────────┴─────────┘
  1 byte    1 byte      Variable         1 byte    1 byte
```

### **Function Code 0x06: Write Single Register**

**Request Example (CAPTURE command):**
```
01 06 00 01 00 01 [CRC]
│  │  │  │  │  │
│  │  │  │  │  └─ Value Low (0x01 = CAPTURE)
│  │  │  │  └──── Value High (0x00)
│  │  │  └──────── Register Low (0x01 = Command Register)
│  │  └─────────── Register High (0x00)
│  └────────────── Function Code (0x06 = Write Single Register)
└───────────────── Slave Address (0x01)
```

**Response (Echo of request if successful):**
```
01 06 00 01 00 01 [CRC]
```

### **Function Code 0x03: Read Holding Registers**

**Request Example (Read Status):**
```
01 03 00 02 00 01 [CRC]
│  │  │  │  │  │
│  │  │  │  │  └─ Quantity Low (1 register)
│  │  │  │  └──── Quantity High (0)
│  │  │  └──────── Start Address Low (0x02 = Status Register)
│  │  └─────────── Start Address High (0x00)
│  └────────────── Function Code (0x03 = Read Holding Registers)
└───────────────── Slave Address (0x01)
```

**Response Example (Status = SUCCESS):**
```
01 03 02 00 02 [CRC]
│  │  │  │  │
│  │  │  │  └──── Data Low (0x02 = SUCCESS)
│  │  │  └──────── Data High (0x00)
│  │  └─────────── Byte Count (2 bytes)
│  └────────────── Function Code (0x03)
└───────────────── Slave Address (0x01)
```

### **CRC16-Modbus Calculation**

**Algorithm:**
```c
uint16_t ModbusMaster_CRC16(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;  // Polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;  // LSB first: Low byte, then High byte
}
```

**Properties:**
- Polynomial: 0xA001 (reversed 0x8005)
- Initial value: 0xFFFF
- RefIn: true, RefOut: true
- Transmitted: LSB first

---

## 🚀 Deployment Guide


#### **2. Build and Flash STM32**

**Build Project:**
```
1. Open STM32CubeIDE
2. Open project: D:\context_capstone\stm32
3. Right-click project → Build Project
4. Wait for compilation (should complete without errors)
```

**Flash to STM32:**
```
1. Connect ST-Link to STM32 (SWDIO, SWCLK, GND, 3.3V)
2. Connect ST-Link to PC via USB
3. Right-click project → Debug As → STM32 C/C++ Application
4. Or: Right-click project → Run As → STM32 C/C++ Application
```

**Verify:**
```
1. Open Serial Terminal (115200 baud, 8N1)
2. Connect to STM32 USART1 (PA9 TX)
3. Reset STM32
4. Should see: "Modbus RTU Master Initialized"
```

#### **3. Build and Flash ESP32-CAM**

**Configure WiFi:**
```cpp
// Edit esp32cam.ino line 23-24
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

**Configure Server URL:**
```cpp
// Edit esp32cam.ino line 27
const char* serverURL = "http://YOUR_SERVER_IP:5000/upload";
```

**Upload to ESP32-CAM:**
```
1. Open Arduino IDE
2. File → Open → D:\context_capstone\esp32cam\esp32cam.ino
3. Tools → Board → ESP32 Arduino → AI Thinker ESP32-CAM
4. Tools → Port → Select COM port
5. Connect GPIO0 to GND (programming mode)
6. Click Upload
7. Wait for "Hard resetting via RTS pin..."
8. Disconnect GPIO0 from GND
9. Press Reset button on ESP32-CAM
```

**Verify:**
```
1. Open Serial Monitor (115200 baud)
2. Reset ESP32-CAM
3. Should see:
   - "Kamera OK!"
   - "WiFi Connected!"
   - "Modbus Slave initialized"
```

#### **4. Wire Hardware Connections**

**Safety first:**
```
1. POWER OFF both boards before wiring
2. Double-check voltage levels (both 3.3V logic)
3. Ensure proper polarity
```

**Connections:**
```
STM32 PB10 (TX) ────► ESP32 GPIO13 (RX)
STM32 PB11 (RX) ◄──── ESP32 GPIO12 (TX)
STM32 GND ──────────── ESP32 GND

Optional:
STM32 PA9 (TX) ─────► USB-TTL RX (for debug)
STM32 GND ──────────── USB-TTL GND
```

**Wire Management:**
- Use color-coded wires (red=power, black=GND, others=signals)
- Keep wires short (<30cm) for reliability
- Avoid parallel routing with power cables
- Use heat shrink or tape to prevent shorts

#### **5. Setup Upload Server**

**Prepare Server:**
```bash
# Navigate to server folder
cd D:\context_capstone\server

# Check Python installation
python --version

# Install dependencies
pip install flask requests

# Optional: Test server first
python test_upload.py
```

**Run Server:**
```bash
# Windows (Command Prompt)
python server.py

# Expected output:
# ============================================================
# ESP32-CAM Upload Server Starting...
# Local IP: 192.168.1.X
# Server will listen on: http://0.0.0.0:5000
# ============================================================
#  * Running on http://192.168.1.X:5000
```

**Windows Firewall (IMPORTANT):**
```powershell
# Open PowerShell as Administrator
New-NetFirewallRule -DisplayName "Python Flask 5000" -Direction Inbound -LocalPort 5000 -Protocol TCP -Action Allow
```

**Test Server:**
```
1. Open browser
2. Navigate to: http://localhost:5000
3. Should see: "ESP32-CAM Upload Server is running!"
```

#### **6. System Integration Test**

**Power Up Sequence:**
```
1. Start Flask server first (python server.py)
2. Power on ESP32-CAM (wait for WiFi connection)
3. Power on STM32 (will start sending commands)
```

**Monitor All Three:**
```
Terminal 1: Flask server console
Terminal 2: ESP32-CAM Serial Monitor (115200)
Terminal 3: STM32 Serial Monitor (115200)
```

**Expected Behavior:**

**STM32 Console:**
```
[Modbus] Mengirim perintah CAPTURE ke ESP32-CAM...
[Modbus] Command berhasil dikirim!
[Modbus] Polling status ESP32-CAM...
[Modbus] SUCCESS! ESP32-CAM selesai capture & upload.
```

**ESP32-CAM Console:**
```
[Modbus] Write to register 0x0001, value: 0x0001
[Task] CAPTURE command received!
[Camera] Image captured successfully (8420 bytes, 640x480)
[Upload] Uploading to server...
[Upload] Success! Response code: 200
[Task] Task completed successfully!
```

**Server Console:**
```
[14:30:15] Upload request from: 192.168.1.15
  -> Receiving 8420 bytes...
  -> SUCCESS! Image saved: uploaded_images/image_20251029_143015.jpg
```

**Verify Images:**
```
Check folder: D:\context_capstone\server\uploaded_images\
Should contain: image_YYYYMMDD_HHMMSS.jpg files
```

---

## 🐛 Troubleshooting

### **Common Issues and Solutions**

#### **Issue 1: CRC Error at STM32**

**Symptoms:**
```
[Modbus] ERROR: Gagal mengirim command (Error code: -2)
  -> CRC Error! Data corrupted atau noise di line.
```

**Possible Causes:**
1. Poor wire connections
2. Baud rate mismatch
3. No common ground
4. EMI/noise interference

**Solutions:**
```
1. Check wiring:
   - Verify TX-RX crossover (PB10→GPIO13, PB11←GPIO12)
   - Ensure GND is connected
   - Test continuity with multimeter

2. Check baud rate:
   - STM32: USART3 = 9600 baud (in CubeMX)
   - ESP32: #define MODBUS_BAUDRATE 9600

3. Reduce baud rate if needed:
   - Change both to 4800 baud
   - Recompile and flash both

4. Improve signal integrity:
   - Use shielded cables
   - Add 100Ω termination resistors
   - Reduce wire length (<20cm)
```

#### **Issue 2: ESP32 Connection Refused**

**Symptoms:**
```
[Upload] ERROR: Failed! Error code: -1
[Upload] Error: connection refused
```

**Possible Causes:**
1. Server not running
2. Wrong IP address
3. Windows Firewall blocking
4. ESP32 and server on different networks

**Solutions:**
```
1. Verify server running:
   python server.py
   # Should show "Running on http://192.168.1.X:5000"

2. Test server from browser:
   http://localhost:5000
   http://192.168.1.X:5000

3. Check Windows Firewall:
   PowerShell (Admin):
   New-NetFirewallRule -DisplayName "Python Flask 5000"
     -Direction Inbound -LocalPort 5000 -Protocol TCP -Action Allow

4. Verify IP addresses:
   - Server IP: ipconfig (look for IPv4)
   - ESP32 IP: Check Serial Monitor WiFi connection
   - Must be same network: 192.168.1.X

5. Ping test:
   ping 192.168.1.9  # From computer
   # Should reply if network OK

6. Test with script:
   python test_upload.py
```

#### **Issue 3: Timeout at STM32**

**Symptoms:**
```
[Modbus] TIMEOUT! ESP32-CAM tidak selesai dalam 30 detik.
```

**Possible Causes:**
1. ESP32 task stuck
2. Upload taking too long
3. ESP32 not updating status register
4. Modbus polling too slow

**Solutions:**
```
1. Check ESP32 Serial:
   - Is task running?
   - Any errors during capture/upload?
   - Status register updated?

2. Enable test mode (skip upload):
   // In esp32cam.ino
   #define SKIP_UPLOAD_FOR_TEST  true
   // This will simulate upload (fast)

3. Increase timeout:
   // In STM32 main.c
   ModbusMaster_WaitForCompletion(&modbus_master, 60000); // 60s

4. Check server response time:
   curl -X POST http://192.168.1.9:5000/upload --data-binary @test.jpg
   # Should respond within seconds
```

#### **Issue 4: Camera Initialization Failed**

**Symptoms:**
```
[Camera] GAGAL init kamera: 0x105
```

**Possible Causes:**
1. Camera module not connected properly
2. Power supply insufficient
3. Wrong board configuration
4. Camera module defective

**Solutions:**
```
1. Check physical connections:
   - Camera ribbon cable seated properly
   - All pins making contact
   - No bent pins

2. Check power supply:
   - Use quality 5V adapter (min 1A)
   - Measure voltage at ESP32-CAM: should be 4.8-5.2V
   - Check for voltage drops during operation

3. Verify board config:
   // In board_config.h
   #define CAMERA_MODEL_AI_THINKER
   // Must match your actual board

4. Test camera separately:
   - Upload simple camera test sketch
   - Verify camera works before Modbus integration
```

#### **Issue 5: WiFi Connection Failed**

**Symptoms:**
```
[WiFi] Failed to connect! Continuing without WiFi...
```

**Possible Causes:**
1. Wrong SSID/password
2. Weak WiFi signal
3. Router settings (MAC filter, hidden SSID)
4. 5GHz WiFi (ESP32 only supports 2.4GHz)

**Solutions:**
```
1. Verify credentials:
   const char* ssid = "woi";
   const char* password = "123456781";
   // Check for typos, spaces, case-sensitivity

2. Move closer to router:
   - ESP32-CAM WiFi antenna is weak
   - Test within 2-3 meters first

3. Check router settings:
   - Disable MAC filtering temporarily
   - Use 2.4GHz network (not 5GHz)
   - Unhide SSID if hidden
   - Check DHCP is enabled

4. Test with phone hotspot:
   - Create hotspot on phone
   - Connect ESP32 to hotspot
   - If works, issue is with router
```

#### **Issue 6: Images Corrupted or Zero Bytes**

**Symptoms:**
- Image files saved but cannot open
- File size is 0 bytes or very small

**Possible Causes:**
1. HTTP request interrupted
2. Memory allocation failed
3. Camera capture failed but no error
4. Content-Type mismatch

**Solutions:**
```
1. Check ESP32 Serial for errors:
   - "Image captured successfully" with size > 0?
   - Upload response code 200?

2. Verify PSRAM:
   - ESP32-CAM has PSRAM for larger images
   - Check Board setting includes PSRAM support

3. Reduce image quality/size:
   // In esp32cam.ino setup()
   config.frame_size = FRAMESIZE_QVGA;  // Smaller (320x240)
   config.jpeg_quality = 15;            // Lower quality, smaller file

4. Test server upload manually:
   curl -X POST http://192.168.1.9:5000/upload
        --data-binary @known_good_image.jpg
   # Verify saved image is valid
```

### **Diagnostic Tools**

#### **Modbus Analyzer (PC Software)**

For deep debugging, use Modbus analysis tools:

1. **Modbus Poll** (Windows, paid but trial available)
   - Monitor Modbus communication in real-time
   - Manually read/write registers
   - Log all frames with timestamps

2. **QModMaster** (Free, cross-platform)
   - Open-source Modbus master simulator
   - Test ESP32-CAM slave independently
   - Verify register values

**Usage:**
```
1. Connect USB-TTL to ESP32-CAM UART (not Modbus UART!)
2. Use as Modbus bridge
3. Configure tool:
   - Port: COM port of USB-TTL
   - Baud: 9600
   - Slave ID: 0x01
4. Read/Write registers manually
5. Verify responses
```

#### **Logic Analyzer (Hardware)**

For ultimate debugging:

1. **Saleae Logic Analyzer** (recommended but expensive)
2. **DSLogic Plus** (cheaper alternative)

**Setup:**
```
1. Connect probes:
   - CH0: STM32 TX (PB10)
   - CH1: ESP32 RX (GPIO13)
   - CH2: STM32 RX (PB11)
   - CH3: ESP32 TX (GPIO12)
   - GND: Common ground

2. Configure analyzer:
   - Protocol: Async Serial (UART)
   - Baud rate: 9600
   - Data bits: 8, Parity: None, Stop: 1

3. Add Modbus decoder:
   - Import Modbus RTU decoder
   - Analyze frames, CRC, timing

4. Identify issues:
   - Frame collisions
   - CRC mismatches
   - Timing violations
   - Signal quality
```

---

## 🤖 Next Phase: Robot Integration

### **Phase 2 Overview: Navigation System**

**Status:** 🔄 IN PROGRESS (Next: HC-SR04 Ultrasonic Sensor Testing)

**Goal:** Implement autonomous navigation with obstacle avoidance untuk bergerak di gang-gang relief Candi Borobudur.

### **Robot Components & Architecture**


#### **STM32 Pin Allocation (Phase 2 Extensions)**

| STM32 Pin | Function | Connected To | Notes |
|-----------|----------|--------------|-------|
| **Already Used (Phase 1)** |
| PA9 | USART1 TX | Debug console | 115200 baud |
| PA10 | USART1 RX | Debug console | 115200 baud |
| PB10 | USART3 TX | ESP32 GPIO13 | Modbus RTU |
| PB11 | USART3 RX | ESP32 GPIO12 | Modbus RTU |
| PC13 | GPIO Out | LED | Status indicator |
| **Phase 2 Additions** |


### **Robot Operation Logic (Hardcoded Navigation)**

#### **Main Control Loop (Pseudocode)**

```c
void robot_main_loop() {
    while (1) {
        // === STEP 1: Obstacle Detection ===
        uint16_t distance_A = hcsr04_read(SENSOR_A);  // Front-Left
        uint16_t distance_B = hcsr04_read(SENSOR_B);  // Front-Right
        uint16_t distance_C = hcsr04_read(SENSOR_C);  // Side-Left
        uint16_t distance_D = hcsr04_read(SENSOR_D);  // Side-Right

        // === STEP 2: Collision Avoidance ===
        if (distance_A < THRESHOLD_CLOSE) {
            // Obstacle on front-left → Turn Right
            printf("Obstacle detected Front-Left! Turning right...\n");
            robot_turn_right(90);  // Turn 90 degrees
            continue;  // Re-check sensors
        }

        if (distance_B < THRESHOLD_CLOSE) {
            // Obstacle on front-right → Turn Left
            printf("Obstacle detected Front-Right! Turning left...\n");
            robot_turn_left(90);
            continue;
        }

        if (distance_C < THRESHOLD_SIDE) {
            // Too close to left wall → Adjust Right
            printf("Too close to left wall! Adjusting right...\n");
            robot_adjust_right(5);  // Small adjustment
        }

        if (distance_D < THRESHOLD_SIDE) {
            // Too close to right wall → Adjust Left
            printf("Too close to right wall! Adjusting left...\n");
            robot_adjust_left(5);
        }

        // === STEP 3: Image Capture ===
        printf("Path clear. Capturing image...\n");

        // Send CAPTURE command via Modbus
        int result = ModbusMaster_SendCaptureCommand(&modbus_master);
        if (result != MODBUS_OK) {
            printf("ERROR: Failed to send capture command!\n");
            continue;
        }

        // Wait for capture completion
        result = ModbusMaster_WaitForCompletion(&modbus_master, 30000);
        if (result != MODBUS_OK) {
            printf("ERROR: Capture failed or timeout!\n");
            // Retry or log error
            continue;
        }

        printf("Image captured and uploaded successfully!\n");

        // === STEP 4: Move Forward ===
        printf("Moving forward %d cm...\n", DISTANCE_PER_STEP);
        robot_move_forward(DISTANCE_PER_STEP);  // e.g., 20 cm

        // === STEP 5: Stabilization Delay ===
        HAL_Delay(500);  // Wait for vibration to settle

        // === STEP 6: Loop Continue ===
        // Repeat from obstacle detection
    }
}
```

#### **Distance Thresholds (Configurable)**

```c
// Distance thresholds in centimeters
#define THRESHOLD_CLOSE     30   // Front obstacle detection (stop & turn)
#define THRESHOLD_SIDE      15   // Side wall distance (adjust)
#define THRESHOLD_SAFE      50   // Safe distance to proceed
#define DISTANCE_PER_STEP   20   // Distance to move after each capture
```

### **Expected Challenges & Solutions**

| Challenge | Solution |
|-----------|----------|
| **Multiple sensors interfere** | Use time-division multiplexing (read one sensor at a time with delay) |
| **Inaccurate distance readings** | Average multiple readings, add noise filtering |
| **Motors not moving straight** | Implement encoder feedback + PID control |
| **Robot vibration affects camera** | Add stabilization delay after movement |
| **Power supply voltage drop** | Use adequate LiPo battery, voltage regulators |
| **Sensor readings during movement** | Stop before reading sensors |

### **Testing Checklist**

**Phase 2A: Sensor Testing**
- [ ] Single HC-SR04 sensor working
- [ ] All 4 sensors working independently
- [ ] Sensors working simultaneously without interference
- [ ] Accurate distance readings (±2cm)
- [ ] Obstacle detection logic working

**Phase 2B: Motor Testing**
- [ ] Motors spin correctly (forward/backward)
- [ ] PWM speed control working
- [ ] Encoder counting working
- [ ] Precise distance movement (±1cm)
- [ ] Turning angles accurate (±5°)

**Phase 2C: Integration Testing**
- [ ] Obstacle avoidance working
- [ ] Navigation logic correct
- [ ] Camera capture during navigation
- [ ] Upload successful during navigation
- [ ] Complete capture-move cycle working


---

## 📚 References and Resources

### **Documentation**

- **STM32F407 Datasheet:** https://www.st.com/resource/en/datasheet/stm32f407vg.pdf
- **STM32 HAL User Manual:** https://www.st.com/resource/en/user_manual/dm00105879.pdf
- **ESP32 Technical Reference:** https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- **OV2640 Datasheet:** https://www.uctronics.com/download/cam_module/OV2640DS.pdf
- **Modbus Protocol Specification:** https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
- **Flask Documentation:** https://flask.palletsprojects.com/

### **Libraries**

- **modbus-esp8266:** https://github.com/emelianov/modbus-esp8266
- **ESP32 Camera Driver:** https://github.com/espressif/esp32-camera
- **Arduino ESP32:** https://github.com/espressif/arduino-esp32

### **Tools**

- **STM32CubeIDE:** https://www.st.com/en/development-tools/stm32cubeide.html
- **Arduino IDE:** https://www.arduino.cc/en/software
- **Modbus Poll:** https://www.modbustools.com/modbus_poll.html
- **QModMaster:** https://sourceforge.net/projects/qmodmaster/

### **Community Support**

- **STM32 Community:** https://community.st.com/
- **ESP32 Forum:** https://www.esp32.com/
- **Arduino Forum:** https://forum.arduino.cc/
- **Stack Overflow:** https://stackoverflow.com/questions/tagged/modbus

---

## 📝 Change Log

### Version 1.0.0 (October 29, 2025)
- ✅ Initial release
- ✅ Modbus RTU communication working
- ✅ Camera capture functional
- ✅ HTTP upload to Flask server working
- ✅ Complete documentation

### Version 0.9.0 (October 2025)
- 🔧 Implemented Modbus Master (STM32)
- 🔧 Implemented Modbus Slave (ESP32-CAM)
- 🔧 Added HTTP upload server
- 🐛 Fixed CRC error issues
- 🐛 Fixed connection refused issues
- 🐛 Fixed Windows Firewall blocking

### Version 0.5.0 (Initial Development)
- 📝 Text-based UART protocol (deprecated)
- 📝 Simple command/response system
- 📝 Basic camera functionality
