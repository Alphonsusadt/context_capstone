# Autonomous Robot for Borobudur Temple Relief Documentation

**Project Status:** ✅ **PHASE 1 COMPLETED** (Communication System + Error Handling + Button Control) | ⏳ **PHASE 2 NEXT** (Robot Navigation)

**Last Updated:** 2025-11-08

---

## 🎯 Project Goal

Build an **autonomous mobile robot** that can:
- Navigate automatically through Borobudur Temple relief corridors
- Capture relief photos systematically
- Upload images and metadata to Azure cloud storage
- Operate with minimal human intervention

---

## 📊 System Architecture

```
┌─────────────┐    Modbus RTU     ┌──────────────┐    WiFi/HTTPS    ┌──────────────┐
│  STM32F407  │◄──────9600────────►│  ESP32-CAM   │◄────────────────►│ Azure Cloud  │
│   (Master)  │      UART3         │   (Slave)    │     Vercel       │ Blob Storage │
│             │   PB10 ↔ GPIO13    │   OV2640     │                  │   + MongoDB  │
│  • Control  │   PB11 ↔ GPIO12    │   • Camera   │                  │              │
│  • Sensors  │      GND           │   • Upload   │                  │              │
│  • Motors   │                    │              │                  │              │
└─────────────┘                    └──────────────┘                  └──────────────┘
```

**3 Subsystems:**
1. **Navigation** (STM32) - Ultrasonic sensors, motor control, path planning
2. **Vision** (ESP32-CAM) - Camera capture, Modbus slave
3. **Upload** (ESP32-CAM) - WiFi, HTTPS upload to Azure

---

## ✅ Phase 1 Status: COMPLETED (November 2025)

### What Works:
- ✅ Modbus RTU communication (STM32 ↔ ESP32-CAM)
- ✅ Camera capture on command (VGA 640x480 JPEG)
- ✅ Image + metadata upload to Azure Blob Storage
- ✅ Session-based organization (session_001, session_002, ...)
- ✅ Status reporting and error handling
- ✅ Testing mode (configurable photo limits)

### Recent Fixes (2025-11-04 to 2025-11-08):
- ✅ **Initialization race condition** - STM32 waits for ESP32 READY flag
- ✅ **Timeout bug** - Now properly waits 90 seconds (was exiting after 1s)
- ✅ **Backend optimization** - Azure responds in <1s (was 3-15s)
- ✅ **Idempotency protection** - Prevents duplicate captures on retry
- ✅ **UART buffer flush** - Prevents CRC errors from stale data
- ✅ **Button control responsiveness** - RED button checked every 200ms during upload (was once per cycle)

### Success Rate:
- **Before fixes:** ~50% (3/6 attempts, many CRC/timeout errors)
- **After fixes:** **100%** (tested with 3 consecutive photos, no errors)

---

## 🎮 Button Control System (November 2025)

### Hardware:
- **GREEN button (START)**: PC0 → GND (Active LOW with internal pull-up)
- **RED button (STOP)**: PC2 → GND (Active LOW with internal pull-up)

### Button Logic:
- **Active LOW** with internal pull-up resistor
- **Debounce**: 50ms
- **GREEN**: Blocking wait at startup (must press to start capture)
- **RED**: Checked every 100-200ms (responsive abort anytime during operation)

### System Flow with Buttons:
```
[Power ON STM32]
    ↓
[Init all peripherals + WiFi esp-link]
    ↓
[Wait ESP32-CAM READY flag 0x0006]
    ↓
[Set Group ID once]
    ↓
[LED ON solid → System ready]
    ↓
[Wait GREEN BUTTON press] ← User must press to start
    ↓ (button pressed)
[Start CAPTURE LOOP]
    ↓
    while (1) {
      [Check RED BUTTON] ← Press anytime to stop
        ↓ (if pressed)
        [Send END_SESSION to ESP32]
        [ESP32 hits /session/end API]
        [Exit loop → Idle mode]

      [Check ENABLE_PHOTO_LIMIT] ← Testing mode
        ↓ (if limit reached)
        [Exit loop → Idle mode]

      [Capture & Upload photo]
      [Increment counter]
      [Delay 5s]
    }
    ↓
[Idle Mode: LED blink slow]
[Print: "Press RESET to restart"]
```

### Button Response Time:
- **GREEN button**: Instant (blocking wait)
- **RED button**: < 200ms (checked during upload, delay, and all operations)
  - Responsive even during 90-second Azure upload
  - Safe abort - sends END_SESSION command before stopping

### Modbus Register for Session Control:
- **0x0007: SESSION_CONTROL**
  - Write `2`: END_SESSION command
  - ESP32 calls `POST /session/end` API
  - Marks session as completed in backend

---

## 🔧 Hardware Setup

### Pin Connections:
```
STM32F407VG ──────────────────── ESP32-CAM
  PB10 (USART3 TX) ───────► GPIO13 (Serial2 RX)
  PB11 (USART3 RX) ◄────── GPIO12 (Serial2 TX)
  GND ──────────────────── GND

  PA9 (USART1 TX)  ───────► USB-TTL RX (Debug, 115200)
  PA10 (USART1 RX) ◄────── USB-TTL TX

STM32F407VG ──────────────────── Buttons
  PC0 (GPIO Input)  ───────► GREEN button → GND
  PC2 (GPIO Input)  ───────► RED button → GND
  (Internal pull-up enabled)
```

### Components:
- **STM32F407VG Discovery Board** - Master controller
- **ESP32-CAM (AI-Thinker)** - Camera + WiFi module
- **OV2640 Camera Module** - 2MP camera (VGA 640x480 used)
- **Power:** 5V 2A adapter for ESP32-CAM, USB for STM32

---

## 💻 Software Configuration

### Important Files:

**ESP32-CAM:** `esp32cam/esp32cam.ino`
- Backend selection: `USE_PRODUCTION_BACKEND` (true=Azure, false=Flask)
- WiFi credentials: `ssid` and `password`
- Modbus slave ID: `0x01`, Baud: `9600`

**STM32:** `stm32/Core/Src/main.c`
- Testing mode: `ENABLE_PHOTO_LIMIT` (1=limited photos, 0=infinite)
- Max photos: `MAX_PHOTOS` (e.g., 3 for testing)
- Group ID: `current_group` (manually set, e.g., 1)
- Timeout: `90000` ms (90 seconds)

**Backend:** `backend_azure/routes/upload.js`
- Immediate HTTP response (<1s)
- Background upload to Azure Blob Storage
- Session ID caching

### Modbus Register Map:

| Address | Name | R/W | Values | Description |
|---------|------|-----|--------|-------------|
| 0x0001 | COMMAND | R/W | 0=IDLE, 1=CAPTURE | Trigger capture |
| 0x0002 | STATUS | R | 0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR | Task status |
| 0x0003 | ERROR_CODE | R | 0-6 | Error details |
| 0x0004 | PHOTO_ID | R/W | 1, 2, 3... | Current photo number |
| 0x0005 | GROUP_ID | R/W | 1, 2, 3... | Group classification |
| 0x0006 | ESP32_READY | R | 0=not ready, 1=ready | Initialization flag |
| 0x0007 | SESSION_CONTROL | R/W | 0=IDLE, 2=END_SESSION | Session control (RED button) |

---

## 🚀 Quick Start

### 1. Setup Backend (Azure Vercel)
```bash
cd backend_azure
git push origin main  # Auto-deploys to Vercel
```

### 2. Flash ESP32-CAM
```
1. Open Arduino IDE
2. Configure WiFi: ssid = "YOUR_WIFI", password = "YOUR_PASSWORD"
3. Set: USE_PRODUCTION_BACKEND = true (for Azure)
4. Upload to ESP32-CAM
5. Serial Monitor (115200): Wait for "ESP32-CAM FULLY INITIALIZED AND READY!"
```

### 3. Flash STM32
```
1. Open STM32CubeIDE
2. Set: ENABLE_PHOTO_LIMIT = 1, MAX_PHOTOS = 3 (for testing)
3. Build and flash
4. Serial Monitor (115200): System will wait for ESP32 READY
```

### 4. Run Test
```
Power on ESP32 first → Wait for READY message (15-20s)
Power on STM32 second → Capture starts automatically
Monitor both serial consoles
Expected: 3 photos captured successfully
Check Azure Blob Storage: session_001/images/{001,002,003}.jpg
```

---

## 🛡️ Error Handling

### 1. Initialization Handshake
**Problem:** STM32 boots faster than ESP32, sends commands before ready.
**Solution:** ESP32 sets READY flag after WiFi + Session + Camera init. STM32 waits.

### 2. Timeout Protection
**Problem:** Timeout exited after 1s instead of 90s due to immediate error return.
**Solution:** Allow 5 consecutive errors, reset counter on success. Now waits full 90s.

### 3. Idempotency Protection
**Problem:** Timeout + retry could cause duplicate capture with same Photo ID.
**Solution:** ESP32 tracks last completed Photo ID. If duplicate → return SUCCESS, skip capture.

### 4. Backend Optimization
**Problem:** Azure waited for Blob upload (3-15s) before HTTP response.
**Solution:** Backend responds immediately (<1s), uploads in background.

### 5. UART Buffer Flush
**Problem:** Stale data in UART buffer causes CRC errors.
**Solution:** Flush RX buffer before critical Modbus commands.

---

## 📈 System Flow

```
[Startup Phase]
1. ESP32: Boot → WiFi → Session ID → Camera → Set READY=1 (15-20s)
2. STM32: Boot → Wait for ESP32 READY → Set Group ID once

[Capture Loop]
3. STM32: Set Photo ID → Flush buffer → Send CAPTURE
4. ESP32: Check idempotency → Capture → Upload → SUCCESS
5. STM32: Poll status (200ms interval, 90s timeout) → Detect SUCCESS
6. STM32: Increment counter → Repeat or Stop (if limit reached)
```

**Timing:**
- Initialization: ~15-20 seconds (WiFi + Session fetch)
- Per capture: ~3-5 seconds (Azure fast response)
- 3 photos test: ~30 seconds total

---

## 🐛 Common Issues & Fixes

### "ESP32-CAM not ready after 60 seconds"
**Cause:** WiFi connection failed or session fetch timeout.
**Fix:** Check ESP32 serial for WiFi/HTTP errors. Verify WiFi credentials.

### "CRC Error" on STM32
**Cause:** Poor UART connections, EMI noise, or baud rate mismatch.
**Fix:** Check wiring (TX↔RX crossover), verify 9600 baud on both sides.

### "TIMEOUT" on STM32
**Cause:** Azure upload >90s (very rare), or ESP32 stuck.
**Fix:** Idempotency protection handles retry automatically. Check ESP32 serial logs.

### Duplicate photos in Azure
**Should not happen.** Idempotency protection prevents this.
**Debug:** Check ESP32 log for "Updated last_completed_photo_id" messages.

---

## ⏳ Phase 2: Navigation System (NEXT)

**Goal:** Add autonomous navigation with obstacle avoidance.

**Components Needed:**
- 4× HC-SR04 Ultrasonic Sensors (Front-L, Front-R, Side-L, Side-R)
- 2× DC Motors with encoders
- 1× Motor Driver (L298N or TB6612FNG)
- Robot chassis + power distribution

**STM32 Extensions:**
- HC-SR04 driver (trigger + echo timing)
- Motor control (PWM + direction)
- Encoder feedback (precise distance)
- Navigation logic (hardcoded corridor path)

**Status:** Not started yet. Phase 1 must be stable first.

---

## 📁 Repository Structure

```
context_capstone/
├── stm32/                      # STM32 Master Project
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c          # Main control loop
│   │   │   └── modbus_master.c # Modbus RTU implementation
│   │   └── Inc/
│   │       └── modbus_master.h # Modbus definitions
│   └── semua_inti.ioc          # STM32CubeMX config
│
├── esp32cam/
│   └── esp32cam.ino            # ESP32-CAM slave + camera
│
├── backend_azure/              # Azure Vercel backend
│   ├── routes/upload.js        # Image/metadata endpoints
│   └── config/azure.js         # Blob Storage config
│
├── server/
│   └── server.py               # Flask dev server (local testing)
│
├── docs/
│   └── azure backend/          # Backend documentation (user-provided)
│
├── CLAUDE.md                   # This file (project documentation)
├── README.md                   # Quick start guide
└── QUICK_REFERENCE.md          # Configuration reference
```

---

## 🎯 Testing Checklist

### Quick Test (3 Photos):
```
☐ Backend deployed to Vercel
☐ ESP32-CAM: USE_PRODUCTION_BACKEND = true
☐ STM32: ENABLE_PHOTO_LIMIT = 1, MAX_PHOTOS = 3
☐ Flash both devices
☐ Power on ESP32 → wait for READY (15-20s)
☐ Power on STM32 → capture starts
☐ Monitor both serial consoles
☐ Verify: 3 photos, no errors, program stops
☐ Check Azure: session_001/images/{001,002,003}.jpg exists
```

### Production Mode:
```
☐ STM32: ENABLE_PHOTO_LIMIT = 0
☐ Flash STM32
☐ System captures continuously (infinite loop)
```

---

## 📚 Documentation Files

- **CLAUDE.md** (this file) - Complete project documentation
- **QUICK_REFERENCE.md** - Configuration and error handling reference
- **README.md** - Quick start guide
- **docs/azure backend/** - Backend API documentation (user-provided)

---

## 🔗 References

### Datasheets:
- STM32F407VG: https://www.st.com/resource/en/datasheet/stm32f407vg.pdf
- ESP32: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- OV2640: https://www.uctronics.com/download/cam_module/OV2640DS.pdf

### Libraries:
- modbus-esp8266: https://github.com/emelianov/modbus-esp8266
- ESP32 Camera: https://github.com/espressif/esp32-camera

### Tools:
- STM32CubeIDE: https://www.st.com/en/development-tools/stm32cubeide.html
- Arduino IDE: https://www.arduino.cc/en/software

---

**Project Status:** Phase 1 complete and tested. Ready for Phase 2 (navigation).
