# STM32F407VG Pinout Configuration

**Project:** Borobudur Temple Relief Documentation Robot
**Board:** STM32F407VG Discovery
**Last Updated:** 2025-11-08

---

## 📌 Pin Summary

| Pin | Function | Direction | Config | Connected To |
|-----|----------|-----------|--------|--------------|
| **PA9** | USART1 TX | Output | 115200 8N1 | USB-TTL RX (Debug) |
| **PA10** | USART1 RX | Input | 115200 8N1 | USB-TTL TX (Debug) |
| **PA2** | USART2 TX | Output | 115200 8N1 | ESP-01 RX (WiFi) |
| **PA3** | USART2 RX | Input | 115200 8N1 | ESP-01 TX (WiFi) |
| **PB10** | USART3 TX | Output | 9600 8N1 | ESP32-CAM GPIO13 (Modbus) |
| **PB11** | USART3 RX | Input | 9600 8N1 | ESP32-CAM GPIO12 (Modbus) |
| **PC0** | GPIO Input | Input | Pull-up, Active LOW | GREEN Button → GND |
| **PC2** | GPIO Input | Input | Pull-up, Active LOW | RED Button → GND |
| **PC13** | GPIO Output | Output | Push-Pull, Active HIGH | LED Indicator |

---

## 🔌 Detailed Pinout

### 1. USART1 - USB-TTL Debug Serial
**Purpose:** Debug output and serial monitoring
**Baud Rate:** 115200
**Format:** 8 data bits, No parity, 1 stop bit (8N1)

```
STM32F407VG          USB-TTL Adapter
  PA9 (TX) ──────────► RX
  PA10 (RX) ◄──────── TX
  GND ────────────────► GND
```

**Notes:**
- Used for `printf()` debug output
- Serial monitor at 115200 baud
- All system messages printed here

---

### 2. USART2 - WiFi ESP-01 (esp-link)
**Purpose:** Wireless serial monitoring via ESP-01 with esp-link firmware
**Baud Rate:** 115200
**Format:** 8 data bits, No parity, 1 stop bit (8N1)

```
STM32F407VG          ESP-01 (esp-link)
  PA2 (TX) ──────────► RX
  PA3 (RX) ◄──────── TX
  GND ────────────────► GND
  3.3V ───────────────► VCC / CH_PD
```

**Notes:**
- Mirrors all debug output from USART1
- Allows wireless serial monitoring via web browser
- ESP-01 must be flashed with esp-link firmware
- Connect to ESP-01 WiFi AP or configure as station mode

---

### 3. USART3 - ESP32-CAM Modbus RTU
**Purpose:** Modbus RTU communication with ESP32-CAM for camera control
**Baud Rate:** 9600
**Format:** 8 data bits, No parity, 1 stop bit (8N1)
**Protocol:** Modbus RTU

```
STM32F407VG          ESP32-CAM
  PB10 (TX) ─────────► GPIO13 (RX2)
  PB11 (RX) ◄───────── GPIO12 (TX2)
  GND ────────────────► GND
```

**Modbus Configuration:**
- **Master:** STM32F407VG
- **Slave Address:** 0x01 (ESP32-CAM)
- **Function Codes:** 0x03 (Read), 0x06 (Write)
- **Timeout:** 1000ms

**Register Map:**
| Register | Name | Access | Description |
|----------|------|--------|-------------|
| 0x0001 | COMMAND | R/W | 0=IDLE, 1=CAPTURE |
| 0x0002 | STATUS | R | 0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR |
| 0x0003 | ERROR_CODE | R | Error details (0-6) |
| 0x0004 | PHOTO_ID | R/W | Photo sequence number |
| 0x0005 | GROUP_ID | R/W | Group/section identifier |
| 0x0006 | ESP32_READY | R | 0=not ready, 1=ready |
| 0x0007 | SESSION_CONTROL | R/W | 0=IDLE, 2=END_SESSION |

---

### 4. GPIO - User Control Buttons

#### GREEN Button (START)
**Pin:** PC0
**Function:** Start capture sequence
**Config:** GPIO Input, Internal Pull-up, Active LOW
**Debounce:** 50ms

```
STM32F407VG          Push Button
  PC0 ────────────────► Terminal 1
  GND ◄──────────────── Terminal 2
```

**Behavior:**
- Blocking wait at startup
- System waits for GREEN button press before starting capture loop
- Press once to begin operation

---

#### RED Button (STOP)
**Pin:** PC2
**Function:** Stop/abort capture session
**Config:** GPIO Input, Internal Pull-up, Active LOW
**Debounce:** 50ms

```
STM32F407VG          Push Button
  PC2 ────────────────► Terminal 1
  GND ◄──────────────── Terminal 2
```

**Behavior:**
- Non-blocking check every 100-200ms
- Can abort operation anytime during:
  - Photo upload (checked every 200ms)
  - Delay between captures (checked every 100ms)
  - Error recovery delays (checked every 100ms)
- Sends END_SESSION command to ESP32-CAM before stopping
- Response time: < 200ms

---

### 5. GPIO - LED Indicator

**Pin:** PC13
**Function:** System status indicator
**Config:** GPIO Output, Push-Pull, No Pull, Active HIGH

```
STM32F407VG          LED
  PC13 ───────────────► Anode (+)
  GND ◄───────────────── Cathode (-)
  (Add 220Ω resistor in series)
```

**LED States:**
| Pattern | Meaning |
|---------|---------|
| Solid ON | System ready, waiting for GREEN button |
| Blinking slow | Idle mode (after session ended) |
| OFF | System booting or error |

---

## 🔧 Wiring Checklist

### Minimum Setup (Core Functionality):
- [x] USART1 to USB-TTL (Debug - Required)
- [x] USART3 to ESP32-CAM (Camera Control - Required)
- [x] PC0 to GREEN Button (Start - Required)
- [x] PC2 to RED Button (Stop - Required)
- [x] PC13 to LED (Indicator - Recommended)

### Optional (Wireless Monitoring):
- [ ] USART2 to ESP-01 (WiFi Serial Monitor - Optional)

### Power:
- [x] STM32: 5V via USB or external
- [x] ESP32-CAM: 5V 2A adapter (stable power required!)
- [x] ESP-01: 3.3V (if used)

---

## ⚠️ Important Notes

### Pull-up Resistors:
- **PC0 and PC2:** Internal pull-up enabled in software (no external resistor needed)
- Buttons are **Active LOW** (pressed = GND)

### UART Cross-Connection:
- Always connect TX → RX and RX → TX
- **Never** connect TX → TX or RX → RX

### ESP32-CAM Power:
- **Critical:** Use dedicated 5V 2A power supply
- USB power often insufficient (causes brownouts)
- Shared GND with STM32 required

### Modbus Timing:
- 9600 baud requires ~1ms per byte transmission
- Total Modbus frame: ~10-15ms
- Polling interval: 200ms (upload status check)

### Debug Output:
- USART1 and USART2 both receive identical debug messages
- Use either wired (USART1) or wireless (USART2) monitoring
- Baud rate: 115200 for both

---

## 📊 Pin Allocation Summary

**Used Pins:** 9 total
- USART: 6 pins (3 channels × 2 pins)
- GPIO Input: 2 pins (buttons)
- GPIO Output: 1 pin (LED)

**Available for Phase 2 (Navigation):**
- TIM1_CH1-4 for ultrasonic sensors (Echo)
- TIM8_CH1-4 for ultrasonic sensors (Echo)
- Multiple GPIO for ultrasonic Trigger
- TIM2, TIM3, TIM9, TIM12 for motor PWM
- Plenty of GPIO for motor direction control

---

**End of Pinout Documentation**
