/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 8 HC-SR04 Optimized (LOW LATENCY)
  ******************************************************************************
  * Informasi 8 sensor ultrasonik
  * Sensor 1: Echo_1 = PE9  (TIM1_CH1) Trig_1 = PE10 (Dp_Kn)
  * Sensor 2: Echo_2 = PE11 (TIM1_CH2) Trig_2 = PE12 (Dp_Kr)
  * Sensor 3: Echo_3 = PE13 (TIM1_CH3) Trig_3 = PE14 (Sp_Kr_Dp)
  * Sensor 4: Echo_4 = PA11 (TIM1_CH4) Trig_4 = PA12 (Sp_Kr_Bl)
  * Sensor 5: Echo_5 = PC9  (TIM8_CH4) Trig_5 = PD15 (Bl_Kr)
  * Sensor 6: Echo_6 = PC8  (TIM8_CH3) Trig_6 = PD14 (Bl_Kn)
  * Sensor 7: Echo_7 = PC7  (TIM8_CH2) Trig_7 = PD13 (Sp_Kn_Bl)
  * Sensor 8: Echo_8 = PC6  (TIM8_CH1) Trig_8 = PD12 (Sp_Kn_Dp)
  * UART: TX = PA9, RX = PA10 (USART1) --> ke laptop
  * Ket
  * Dp = depan		Kn = Kanan		Sp = Samping
  * Bl = Belakang	Kr = Kiri
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_SENSORS 8
#define FILTER_SIZE 3 // Median filter dengan 3 nilai
#define SENSOR_TIMEOUT_US 25000 // 25ms timeout (max echo ~23.2ms)
#define MEASUREMENT_INTERVAL_MS 15 // Interval antar pengukuran

// Struktur data untuk setiap sensor
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *TRIG_PORT;
    uint16_t TRIG_PIN;
    const char *name;

    volatile uint32_t ic_val1;
    volatile uint32_t ic_val2;
    volatile uint32_t difference;
    volatile bool is_first_captured;
    volatile bool is_captured;

    // Buffer untuk median filter
    float buffer[FILTER_SIZE];
    uint8_t buffer_index;
    bool buffer_full;

    float distance;
    float filtered_distance;
} HC_SR04_IC;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HC_SR04_IC sensors[NUM_SENSORS];
volatile uint8_t sensors_captured_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Sensors_Init(void);
void delay_us(uint32_t us);
void HC_SR04_Trigger_All(void);
float HC_SR04_Calculate_Distance(HC_SR04_IC *sensor);
float Median_Filter(float *buffer, uint8_t size);
void Update_Filter(HC_SR04_IC *sensor, float new_value);
void Print_Sensor_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== Fungsi Delay Microsecond ====================
void delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

// ==================== Median Filter ====================
float Median_Filter(float *buffer, uint8_t size) {
    if (size == 0) return 0.0f;
    if (size == 1) return buffer[0];

    float temp[FILTER_SIZE];
    memcpy(temp, buffer, size * sizeof(float));

    // Bubble sort untuk mencari median
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }

    // Return median value
    return temp[size / 2];
}

// ==================== Update Filter Buffer ====================
void Update_Filter(HC_SR04_IC *sensor, float new_value) {
    // Tambahkan nilai baru ke buffer
    sensor->buffer[sensor->buffer_index] = new_value;
    sensor->buffer_index++;

    // Reset index jika sudah penuh
    if (sensor->buffer_index >= FILTER_SIZE) {
        sensor->buffer_index = 0;
        sensor->buffer_full = true;
    }

    // Hitung median jika buffer sudah ada data
    if (sensor->buffer_full) {
        sensor->filtered_distance = Median_Filter(sensor->buffer, FILTER_SIZE);
    } else {
        // Jika buffer belum penuh, gunakan data yang ada
        sensor->filtered_distance = Median_Filter(sensor->buffer, sensor->buffer_index);
    }
}

// ==================== Inisialisasi Array Sensor ====================
void Sensors_Init(void) {
    // Sensor 1: TIM1_CH1 (PE9) - Trig PE10
    sensors[0].htim = &htim1;
    sensors[0].channel = TIM_CHANNEL_1;
    sensors[0].TRIG_PORT = GPIOE;
    sensors[0].TRIG_PIN = Trig_1_Pin;
    sensors[0].name = "US1";

    // Sensor 2: TIM1_CH2 (PE11) - Trig PE12
    sensors[1].htim = &htim1;
    sensors[1].channel = TIM_CHANNEL_2;
    sensors[1].TRIG_PORT = GPIOE;
    sensors[1].TRIG_PIN = Trig_2_Pin;
    sensors[1].name = "US2";

    // Sensor 3: TIM1_CH3 (PE13) - Trig PE14
    sensors[2].htim = &htim1;
    sensors[2].channel = TIM_CHANNEL_3;
    sensors[2].TRIG_PORT = GPIOE;
    sensors[2].TRIG_PIN = Trig_3_Pin;
    sensors[2].name = "US3";

    // Sensor 4: TIM1_CH4 (PA11) - Trig PA12
    sensors[3].htim = &htim1;
    sensors[3].channel = TIM_CHANNEL_4;
    sensors[3].TRIG_PORT = Trig_4_GPIO_Port;
    sensors[3].TRIG_PIN = Trig_4_Pin;
    sensors[3].name = "US4";

    // Sensor 5: TIM8_CH4 (PC9) - Trig PD15
    sensors[4].htim = &htim8;
    sensors[4].channel = TIM_CHANNEL_4;
    sensors[4].TRIG_PORT = GPIOD;
    sensors[4].TRIG_PIN = Trig_5_Pin;
    sensors[4].name = "US5";

    // Sensor 6: TIM8_CH3 (PC8) - Trig PD14
    sensors[5].htim = &htim8;
    sensors[5].channel = TIM_CHANNEL_3;
    sensors[5].TRIG_PORT = GPIOD;
    sensors[5].TRIG_PIN = Trig_6_Pin;
    sensors[5].name = "US6";

    // Sensor 7: TIM8_CH2 (PC7) - Trig PD13
    sensors[6].htim = &htim8;
    sensors[6].channel = TIM_CHANNEL_2;
    sensors[6].TRIG_PORT = GPIOD;
    sensors[6].TRIG_PIN = Trig_7_Pin;
    sensors[6].name = "US7";

    // Sensor 8: TIM8_CH1 (PC6) - Trig PD12
    sensors[7].htim = &htim8;
    sensors[7].channel = TIM_CHANNEL_1;
    sensors[7].TRIG_PORT = GPIOD;
    sensors[7].TRIG_PIN = Trig_8_Pin;
    sensors[7].name = "US8";

    // Inisialisasi semua sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].is_first_captured = false;
        sensors[i].is_captured = false;
        sensors[i].difference = 0;
        sensors[i].distance = 0.0f;
        sensors[i].filtered_distance = 0.0f;
        sensors[i].buffer_index = 0;
        sensors[i].buffer_full = false;

        // Inisialisasi buffer filter
        for (int j = 0; j < FILTER_SIZE; j++) {
            sensors[i].buffer[j] = 0.0f;
        }
    }
}

// ==================== Trigger Semua Sensor Bersamaan ====================
void HC_SR04_Trigger_All(void) {
    // 1. Reset semua flag dan counter
    sensors_captured_count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].is_captured = false;
        sensors[i].is_first_captured = false;
        // Pastikan polaritas kembali ke RISING
        __HAL_TIM_SET_CAPTUREPOLARITY(sensors[i].htim, sensors[i].channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }

    // 2. Set trigger HIGH untuk semua sensor bersamaan
    for (int i = 0; i < NUM_SENSORS; i++) {
        HAL_GPIO_WritePin(sensors[i].TRIG_PORT, sensors[i].TRIG_PIN, GPIO_PIN_SET);
    }

    delay_us(12); // 10-12us sesuai datasheet

    // 3. Set trigger LOW untuk semua sensor bersamaan
    for (int i = 0; i < NUM_SENSORS; i++) {
        HAL_GPIO_WritePin(sensors[i].TRIG_PORT, sensors[i].TRIG_PIN, GPIO_PIN_RESET);
    }

    // 4. Mulai timer timeout
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

// ==================== Fungsi Hitung Jarak ====================
float HC_SR04_Calculate_Distance(HC_SR04_IC *sensor) {
    if (!sensor->is_captured) {
        return -1.0f; // Tidak ada data valid
    }

    // Konversi durasi (us) ke jarak (cm)
    // Formula: distance = (time_in_us * speed_of_sound) / 2
    // speed_of_sound = 343 m/s = 0.0343 cm/us
    // distance = time_us * 0.0343 / 2 = time_us * 0.01715
    float distance = (float)sensor->difference * 0.01715f;

    // Reset flag setelah dibaca
    sensor->is_captured = false;

    // Filter out-of-range values (2cm - 400cm)
    if (distance < 2.0f || distance > 400.0f) {
        return -1.0f;
    }

    return distance;
}

// ==================== Fungsi Print Data Sensor ====================
void Print_Sensor_Data(void) {
    char buffer[256];
    int length = 0;

    // Format: US1:25.5, US2:30.2, US3:15.8, US4:0.0, US5:42.1, US6:18.3, US7:0.0, US8:35.6
    for (int i = 0; i < NUM_SENSORS; i++) {
        float distance = sensors[i].filtered_distance;

        // Jika distance invalid, set ke 0.0
        if (distance < 2.0f || distance > 400.0f) {
            distance = 0.0f;
        }

        // Format setiap sensor
        if (i == 0) {
            length = snprintf(buffer, sizeof(buffer), "%s:%.1f", sensors[i].name, distance);
        } else {
            length += snprintf(buffer + length, sizeof(buffer) - length, ", %s:%.1f", sensors[i].name, distance);
        }
    }

    // Tambahkan newline dan kirim via UART
    snprintf(buffer + length, sizeof(buffer) - length, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// ==================== Override printf untuk UART ====================
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  // Inisialisasi array sensor
  Sensors_Init();

  // Start Timer untuk delay microsecond & timeout
  HAL_TIM_Base_Start(&htim2);

  // Start Input Capture untuk semua channel TIM1
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

  // Start Input Capture untuk semua channel TIM8
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);

  printf("SYSTEM READY (OPTIMIZED LOW LATENCY)\r\n");
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t cycle_start = HAL_GetTick();

    // === 1. Trigger semua sensor & reset state ===
    HC_SR04_Trigger_All();

    // === 2. Tunggu data dengan timeout 25ms ===
    while (sensors_captured_count < NUM_SENSORS) {
        if (__HAL_TIM_GET_COUNTER(&htim2) > SENSOR_TIMEOUT_US) {
            break; // Timeout setelah 25ms
        }
    }

    // === 3. Baca dan update filter untuk semua sensor ===
    for (int i = 0; i < NUM_SENSORS; i++) {
        float raw_distance = -1.0f;

        // Cek jika sensor ini berhasil capture
        if(sensors[i].is_captured) {
            raw_distance = HC_SR04_Calculate_Distance(&sensors[i]);
        }

        // Hanya update filter jika data valid
        if (raw_distance > 0) {
            Update_Filter(&sensors[i], raw_distance);
            sensors[i].distance = raw_distance;
        } else {
            // Jika data invalid, set filtered_distance ke 0.0
            sensors[i].filtered_distance = 0.0f;
        }
    }

    // === 4. Kirim data via UART dengan format yang diinginkan ===
    Print_Sensor_Data();

    // === 5. Maintain minimum measurement interval ===
    while (HAL_GetTick() - cycle_start < MEASUREMENT_INTERVAL_MS) {
        // Tunggu sampai interval minimum tercapai
    }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15; // Untuk 1MHz (1 tick = 1us) dengan clock 16MHz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15; // Untuk 1MHz (1 tick = 1us) dengan clock 16MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  // HAL_TIM_MspPostInit dihapus karena tidak diperlukan untuk sensor ultrasonik
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{
  /* USER CODE BEGIN TIM8_Init 0 */
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 15; // Untuk 1MHz (1 tick = 1us) dengan clock 16MHz
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Trig_1_Pin|Trig_2_Pin|Trig_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Trig_8_Pin|Trig_7_Pin|Trig_6_Pin|Trig_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_4_GPIO_Port, Trig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Trig_1_Pin Trig_2_Pin Trig_3_Pin */
  GPIO_InitStruct.Pin = Trig_1_Pin|Trig_2_Pin|Trig_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_8_Pin Trig_7_Pin Trig_6_Pin Trig_5_Pin */
  GPIO_InitStruct.Pin = Trig_8_Pin|Trig_7_Pin|Trig_6_Pin|Trig_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_4_Pin */
  GPIO_InitStruct.Pin = Trig_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ==================== Input Capture Callback (OPTIMIZED) ====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    HC_SR04_IC *sensor = NULL;
    uint32_t capture_value;

    // Identifikasi sensor dengan lookup table yang lebih cepat
    if (htim->Instance == TIM1) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1: sensor = &sensors[0]; break;
            case HAL_TIM_ACTIVE_CHANNEL_2: sensor = &sensors[1]; break;
            case HAL_TIM_ACTIVE_CHANNEL_3: sensor = &sensors[2]; break;
            case HAL_TIM_ACTIVE_CHANNEL_4: sensor = &sensors[3]; break;
            default: return;
        }
    }
    else if (htim->Instance == TIM8) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1: sensor = &sensors[7]; break;
            case HAL_TIM_ACTIVE_CHANNEL_2: sensor = &sensors[6]; break;
            case HAL_TIM_ACTIVE_CHANNEL_3: sensor = &sensors[5]; break;
            case HAL_TIM_ACTIVE_CHANNEL_4: sensor = &sensors[4]; break;
            default: return;
        }
    }
    else {
        return;
    }

    capture_value = HAL_TIM_ReadCapturedValue(htim, sensor->channel);

    if (!sensor->is_first_captured) {
        // First capture (RISING edge)
        sensor->ic_val1 = capture_value;
        sensor->is_first_captured = true;

        // Change polarity to FALLING
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else {
        // Second capture (FALLING edge)
        sensor->ic_val2 = capture_value;

        // Calculate difference (handle overflow)
        sensor->difference = (sensor->ic_val2 >= sensor->ic_val1) ?
                           (sensor->ic_val2 - sensor->ic_val1) :
                           (0xFFFF - sensor->ic_val1 + sensor->ic_val2);

        // Validate pulse width (150us - 23200us = ~2.5cm - 400cm)
        sensor->is_captured = (sensor->difference >= 150 && sensor->difference <= 23200);

        // Reset for next measurement
        sensor->is_first_captured = false;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->channel, TIM_INPUTCHANNELPOLARITY_RISING);

        // Increment completion counter
        sensors_captured_count++;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
