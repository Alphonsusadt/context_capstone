# Phase 2: HC-SR04 Ultrasonic Sensor Testing Guide

**Status:** 🔄 **NEXT TASK** - Ready to start

**Objective:** Test HC-SR04 ultrasonic sensor dengan STM32F407 untuk obstacle detection.

---

## 🎯 Testing Goals

1. ✅ Wire single HC-SR04 to STM32
2. ✅ Implement basic HC-SR04 driver (polling mode)
3. ✅ Get accurate distance readings
4. ✅ Test with different objects/distances
5. ✅ Wire and test 4 sensors simultaneously
6. ✅ Implement obstacle detection logic

---

## 🔌 Hardware Setup

### Single Sensor Test Wiring

```
HC-SR04          STM32F407VG
--------         ------------
VCC     ───────► 5V (or 3.3V via level shifter if needed)
GND     ───────► GND
TRIG    ───────► PD0 (GPIO Output)
ECHO    ───────► PD1 (GPIO Input / Timer Input Capture)
```

**⚠️ Important:**
- HC-SR04 works on 5V, but STM32 GPIO is 3.3V tolerant
- ECHO pin outputs 5V → **Use voltage divider** or level shifter for ECHO pin
- Simple voltage divider:
  ```
  ECHO ──┬── 2kΩ ──┬── PD1 (STM32)
         │         │
         └── 3kΩ ──┴── GND

  (This gives 3V from 5V signal)
  ```

### 4-Sensor Configuration

| Sensor | Position | TRIG Pin | ECHO Pin |
|--------|----------|----------|----------|
| A | Front-Left | PD0 | PD1 |
| B | Front-Right | PD2 | PD3 |
| C | Side-Left | PD4 | PD5 |
| D | Side-Right | PD6 | PD7 |

---

## 💻 STM32CubeMX Configuration

### Step 1: GPIO Configuration

**Trigger Pins (Output):**
- PD0, PD2, PD4, PD6 → GPIO_Output
- Label: TRIG_A, TRIG_B, TRIG_C, TRIG_D
- Mode: Output Push Pull
- Speed: High
- Initial: LOW

**Echo Pins (Input):**
- PD1, PD3, PD5, PD7 → GPIO_Input
- Label: ECHO_A, ECHO_B, ECHO_C, ECHO_D
- Mode: Input
- Pull-up/Pull-down: No pull

### Step 2: Timer Configuration (Optional, untuk Input Capture)

**TIM2 Configuration (for precise timing):**
- Clock Source: Internal Clock
- Prescaler: 72-1 (for 1µs resolution at 72MHz)
- Counter Period: 0xFFFF-1 (max)
- Auto-reload: Enable

**OR simpler: Use HAL_GetTick() + polling for testing**

---

## 📝 Test Code (Simple Polling Version)

### HC-SR04 Driver Header

```c
// hcsr04.h
#ifndef HCSR04_H_
#define HCSR04_H_

#include "stm32f4xx_hal.h"

// Sensor definitions
typedef enum {
    SENSOR_A = 0,  // Front-Left
    SENSOR_B = 1,  // Front-Right
    SENSOR_C = 2,  // Side-Left
    SENSOR_D = 3   // Side-Right
} HC_SR04_Sensor_t;

// Function prototypes
void HCSR04_Init(void);
uint16_t HCSR04_Read(HC_SR04_Sensor_t sensor);
void HCSR04_TestAll(void);

// Microsecond delay (for 10µs trigger pulse)
static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

#endif /* HCSR04_H_ */
```

### HC-SR04 Driver Implementation

```c
// hcsr04.c
#include "hcsr04.h"
#include <stdio.h>

// Pin definitions (adjust based on your wiring)
typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} HC_SR04_Config_t;

static HC_SR04_Config_t sensors[4] = {
    {GPIOD, GPIO_PIN_0, GPIOD, GPIO_PIN_1},  // Sensor A
    {GPIOD, GPIO_PIN_2, GPIOD, GPIO_PIN_3},  // Sensor B
    {GPIOD, GPIO_PIN_4, GPIOD, GPIO_PIN_5},  // Sensor C
    {GPIOD, GPIO_PIN_6, GPIOD, GPIO_PIN_7}   // Sensor D
};

void HCSR04_Init(void) {
    // Enable DWT for delay_us()
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    printf("HC-SR04 sensors initialized\n");
}

uint16_t HCSR04_Read(HC_SR04_Sensor_t sensor) {
    if (sensor > SENSOR_D) return 0xFFFF;

    HC_SR04_Config_t *s = &sensors[sensor];

    // 1. Send 10µs trigger pulse
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    // 2. Wait for echo pulse start (timeout 30ms)
    uint32_t timeout = HAL_GetTick() + 30;
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() > timeout) {
            return 0xFFFF;  // No echo = out of range or error
        }
    }

    // 3. Measure echo pulse width (microseconds)
    uint32_t start_time = DWT->CYCCNT;

    timeout = HAL_GetTick() + 30;
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() > timeout) {
            return 0xFFFF;  // Echo too long
        }
    }

    uint32_t end_time = DWT->CYCCNT;

    // 4. Calculate distance
    // pulse_width (µs) = (end - start) / (cycles per µs)
    uint32_t pulse_width_us = (end_time - start_time) / (SystemCoreClock / 1000000);

    // Distance (cm) = (pulse_width_us * speed_of_sound) / 2
    // Speed of sound = 343 m/s = 0.0343 cm/µs
    // Distance = (pulse_width * 0.0343) / 2 = pulse_width * 0.01715
    // To avoid float: Distance = (pulse_width * 343) / 20000
    uint16_t distance_cm = (pulse_width_us * 343) / 20000;

    return distance_cm;
}

void HCSR04_TestAll(void) {
    printf("\n=== HC-SR04 Sensor Test ===\n");

    while (1) {
        uint16_t dist_A = HCSR04_Read(SENSOR_A);
        HAL_Delay(60);  // Wait 60ms between sensors to avoid interference

        uint16_t dist_B = HCSR04_Read(SENSOR_B);
        HAL_Delay(60);

        uint16_t dist_C = HCSR04_Read(SENSOR_C);
        HAL_Delay(60);

        uint16_t dist_D = HCSR04_Read(SENSOR_D);
        HAL_Delay(60);

        // Print results
        printf("Sensor A (Front-L): ");
        if (dist_A == 0xFFFF) printf("OUT OF RANGE\n");
        else printf("%d cm\n", dist_A);

        printf("Sensor B (Front-R): ");
        if (dist_B == 0xFFFF) printf("OUT OF RANGE\n");
        else printf("%d cm\n", dist_B);

        printf("Sensor C (Side-L):  ");
        if (dist_C == 0xFFFF) printf("OUT OF RANGE\n");
        else printf("%d cm\n", dist_C);

        printf("Sensor D (Side-R):  ");
        if (dist_D == 0xFFFF) printf("OUT OF RANGE\n");
        else printf("%d cm\n", dist_D);

        printf("----------------------------\n");

        HAL_Delay(500);  // Repeat every 500ms
    }
}
```

### Main.c Integration

```c
// main.c
#include "main.h"
#include "hcsr04.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();  // For printf debug

    printf("\n=== STM32 HC-SR04 Test Program ===\n");

    // Initialize HC-SR04
    HCSR04_Init();

    // Test all sensors
    HCSR04_TestAll();

    while (1) {
        // Should not reach here (TestAll is infinite loop)
    }
}
```

---

## 🧪 Testing Procedure

### Test 1: Single Sensor Basic Test

1. **Wire only Sensor A** (Front-Left)
2. **Upload code** with only Sensor A active
3. **Open Serial Monitor** (115200 baud)
4. **Expected output:**
   ```
   === HC-SR04 Sensor Test ===
   Sensor A (Front-L): 25 cm
   ----------------------------
   Sensor A (Front-L): 25 cm
   ----------------------------
   ```
5. **Test different distances:**
   - Place hand at 10cm → Should read ~10cm
   - Place hand at 50cm → Should read ~50cm
   - Remove hand (>400cm) → Should read "OUT OF RANGE"

### Test 2: Distance Accuracy

1. **Use measuring tape**
2. **Place object at exact distances:**
   - 10cm, 20cm, 30cm, 50cm, 100cm
3. **Record sensor readings**
4. **Calculate error:** `Error = |Measured - Actual|`
5. **Acceptable error:** ±2cm

### Test 3: Multiple Sensors Simultaneously

1. **Wire all 4 sensors**
2. **Test interference:**
   - All sensors reading at same time? → Possible cross-talk
   - Use time-division (60ms delay between readings)
3. **Verify independent readings:**
   - Place objects at different distances for each sensor
   - Confirm each reads correctly

### Test 4: Obstacle Detection Logic

```c
void test_obstacle_detection() {
    while (1) {
        uint16_t dist_A = HCSR04_Read(SENSOR_A);
        uint16_t dist_B = HCSR04_Read(SENSOR_B);

        if (dist_A < 30 && dist_A != 0xFFFF) {
            printf("⚠️ OBSTACLE FRONT-LEFT! Turn Right!\n");
        } else if (dist_B < 30 && dist_B != 0xFFFF) {
            printf("⚠️ OBSTACLE FRONT-RIGHT! Turn Left!\n");
        } else {
            printf("✓ Path clear\n");
        }

        HAL_Delay(500);
    }
}
```

---

## 📊 Expected Results

### Normal Operation

| Distance | Expected Reading | Acceptable Range |
|----------|------------------|------------------|
| 10 cm | 10 cm | 8-12 cm |
| 30 cm | 30 cm | 28-32 cm |
| 50 cm | 50 cm | 48-52 cm |
| 100 cm | 100 cm | 97-103 cm |
| >400 cm | OUT OF RANGE | 0xFFFF |

### Common Issues

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| Always reads 0 | No echo received | Check wiring, voltage levels |
| Always OUT OF RANGE | Timeout too short | Increase timeout, check echo pin |
| Erratic readings | Interference | Add delay between sensors |
| Fixed value | Stuck echo pin | Check voltage divider, pin config |

---

## ✅ Success Criteria

Phase 2A (Sensor Testing) is complete when:
- [ ] Single sensor reads distance accurately (±2cm)
- [ ] All 4 sensors work independently
- [ ] No interference between sensors
- [ ] Obstacle detection logic works (threshold-based)
- [ ] Readings are stable and repeatable
- [ ] Can detect obstacles from 10cm to 200cm

---

## 🔜 Next Steps After Testing

Once sensors are working:
1. Integrate with existing Modbus code (don't break Phase 1!)
2. Add motor driver control
3. Implement movement functions
4. Combine: Sensor → Decision → Camera → Movement

**Timeline:** 1-2 weeks for sensor testing and integration

---

**For complete robot integration plan, see:** `CLAUDE.md` - Section "Next Phase: Robot Integration"
