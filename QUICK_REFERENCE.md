# Quick Reference - ESP32-CAM + STM32 System

**Last Updated:** 2025-11-04

---

## 📍 Important Configurations

### ESP32-CAM (`esp32cam/esp32cam.ino`)

```cpp
// Line 23: Backend Selection
#define USE_PRODUCTION_BACKEND  true   // true = Azure, false = Local Flask

// Line 24-27: WiFi & Server Configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Backend URLs (auto-selected based on USE_PRODUCTION_BACKEND)
// Azure (Production):
//   - https://borobudur-capture-api.vercel.app/session/current
//   - https://borobudur-capture-api.vercel.app/upload/image
//   - https://borobudur-capture-api.vercel.app/upload/meta
// Flask (Development):
//   - http://10.32.207.112:5000/session/current
//   - http://10.32.207.112:5000/upload/image
//   - http://10.32.207.112:5000/upload/meta

// Line 49-58: Modbus Configuration
#define MODBUS_SLAVE_ID     0x01      // ESP32-CAM Modbus slave address
#define MODBUS_BAUDRATE     9600      // UART baud rate
#define REG_COMMAND         0x0001    // Command register
#define REG_STATUS          0x0002    // Status register
#define REG_ERROR_CODE      0x0003    // Error code register
#define REG_PHOTO_ID        0x0004    // Photo ID register
#define REG_GROUP_ID        0x0005    // Group ID register
#define REG_ESP32_READY     0x0006    // Ready flag (handshake)

// Line 79-81: UART Pins for Modbus
#define STM32_RXD 13   // Connect to STM32 PB10 (TX)
#define STM32_TXD 12   // Connect to STM32 PB11 (RX)
```

---

### STM32 (`stm32/Core/Src/main.c`)

```c
// Line 33-37: Testing Mode Configuration
#define ENABLE_PHOTO_LIMIT  1    // 1 = testing (limited photos), 0 = production (infinite)
#define MAX_PHOTOS          3    // Maximum photos to capture in testing mode

// Line 34: Current group ID (manually set)
uint16_t current_group = 1;      // Group ID untuk foto (1, 2, 3, ...)

// Line 413: Timeout Configuration
ModbusMaster_WaitForCompletion(&modbus_master, 90000); // 90 seconds timeout
```

**STM32 UART Configuration (CubeMX):**
- USART1 (Debug): 115200 baud, PA9 (TX), PA10 (RX)
- USART3 (Modbus): 9600 baud, PB10 (TX), PB11 (RX)

---

## 🔧 Hardware Connections

```
STM32 PB10 (TX) ──────► ESP32 GPIO13 (RX)
STM32 PB11 (RX) ◄────── ESP32 GPIO12 (TX)
STM32 GND ────────────── ESP32 GND
```

---

## 🛡️ Error Handling Implemented

### 1. **Initialization Race Condition (FIXED)**
**Problem:** STM32 boots faster than ESP32, sends commands before ESP32 ready.

**Solution:**
- ESP32 sets `REG_ESP32_READY = 1` after full initialization (WiFi + Session + Camera)
- STM32 waits for READY flag before starting capture loop
- STM32 checks every 2 seconds, timeout after 60 seconds

**Log Messages:**
```
[STM32] Waiting for ESP32-CAM initialization...
[STM32] ESP32-CAM not ready yet (READY=0), check #X, waiting 2s...
[STM32] ✅ ESP32-CAM is READY! Starting capture sequence...
```

---

### 2. **Timeout Bug (FIXED)**
**Problem:** STM32 timeout exited after 1 second instead of 60 seconds.

**Solution:**
- Allow up to 5 consecutive communication errors before giving up
- Reset error counter on successful communication
- Now properly waits full 90 seconds

**Result:** Tolerates transient errors during ESP32 initialization/upload.

---

### 3. **Backend Azure Slow Response (FIXED)**
**Problem:** Azure backend waited for Blob upload (3-15s) before HTTP response.

**Solution:**
- Backend responds immediately (200-1000ms)
- Moves Azure Blob upload to background
- Session ID caching to reduce MongoDB queries

**Impact:** Response time dropped from 3-15s to 200-1000ms.

---

### 4. **Idempotency Protection (FIXED)**
**Problem:** STM32 timeout + retry could trigger duplicate capture with same Photo ID.

**Solution:**
- ESP32 tracks `last_completed_photo_id`
- If Photo ID ≤ last completed → return SUCCESS immediately (skip capture)
- Prevents duplicate captures even after timeout/retry

**Log Messages (ESP32):**
```
[Idempotency] Photo ID X already completed!
[Idempotency] Returning SUCCESS immediately (no re-capture)
```

---

### 5. **UART Buffer Corruption (FIXED)**
**Problem:** Stale data in UART buffer causes CRC errors.

**Solution:**
- Flush UART RX buffer before critical Modbus commands
- `ModbusMaster_FlushRxBuffer()` drains buffer

**Usage:** Called before:
- Reading ESP32 READY flag
- Setting Photo ID
- Sending CAPTURE command

---

## 📊 System Flow

```
[Startup]
ESP32: Boot → WiFi → Fetch Session ID → Camera Init → Set READY=1
STM32: Boot → Wait for ESP32 READY → Set Group ID once

[Capture Loop]
STM32: Set Photo ID → Send CAPTURE → Poll status (max 90s)
ESP32: Check idempotency → Capture → Upload → Set SUCCESS
STM32: Detect SUCCESS → Increment counter → Next photo
```

---

## 🔍 Modbus Register Map

| Address | Name | Access | Values | Description |
|---------|------|--------|--------|-------------|
| 0x0001 | COMMAND | R/W | 0=IDLE, 1=CAPTURE | Command from STM32 |
| 0x0002 | STATUS | R | 0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR | Current status |
| 0x0003 | ERROR_CODE | R | 0-6 | Error details (if STATUS=3) |
| 0x0004 | PHOTO_ID | R/W | 1, 2, 3, ... | Current photo ID |
| 0x0005 | GROUP_ID | R/W | 1, 2, 3, ... | Group ID (set once) |
| 0x0006 | ESP32_READY | R | 0=not ready, 1=ready | Handshake flag |

---

## ⚙️ Testing Checklist

### Quick Test (3 Photos):
```
1. Set: ENABLE_PHOTO_LIMIT = 1, MAX_PHOTOS = 3
2. Flash ESP32-CAM
3. Flash STM32
4. Power on ESP32 first → wait for READY message
5. Power on STM32 second → capture starts
6. Verify: 3 photos captured, program stops
```

### Production Mode:
```
1. Set: ENABLE_PHOTO_LIMIT = 0
2. Flash STM32
3. Power on both devices
4. System captures continuously (infinite loop)
```

---

## 🚨 Common Issues

### Issue: "ESP32-CAM not ready after 60 seconds"
**Cause:** WiFi connection slow/failed, or session fetch failed.
**Fix:** Check ESP32 serial monitor for WiFi/session errors.

### Issue: "CRC Error" on STM32
**Cause:** UART noise, poor connections, or baud rate mismatch.
**Fix:** Check wiring, verify both sides using 9600 baud.

### Issue: "TIMEOUT" on STM32
**Cause:** Azure upload taking >90 seconds (rare), or ESP32 stuck.
**Fix:** Check ESP32 serial for upload progress. Idempotency protection should handle retry.

### Issue: Duplicate photos in Azure
**Cause:** Idempotency not working (check logs).
**Fix:** Verify `last_completed_photo_id` updates after each success (ESP32 logs).

---

## 📁 File Structure

```
stm32/
  Core/Src/main.c              → Main control, capture loop
  Core/Src/modbus_master.c     → Modbus RTU master implementation
  Core/Inc/modbus_master.h     → Modbus definitions

esp32cam/
  esp32cam.ino                 → Main sketch, Modbus slave + camera

backend_azure/
  routes/upload.js             → Image/metadata upload endpoints (immediate response)

docs/
  azure backend/               → Backend documentation (user-provided)
```

---

## 🎯 Current Status

- ✅ Communication system fully working (Modbus RTU)
- ✅ Camera capture and upload to Azure
- ✅ Handshake mechanism (ESP32 READY flag)
- ✅ Idempotency protection (no duplicate captures)
- ✅ All error handling implemented
- ✅ Testing mode for quick iterations

**Next Phase:** Navigation system (HC-SR04 sensors + motor control)
