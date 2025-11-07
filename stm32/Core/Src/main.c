/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "modbus_master.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// untuk wifi
#define WIFI_SSID "woi"
#define WIFI_PASS "123456781"
#define WIFI_RX_BUFFER_SIZE 512

// --- TESTING/DEVELOPMENT CONFIGURATION ---
// Set ENABLE_PHOTO_LIMIT to 1 for testing (captures MAX_PHOTOS then stops)
// Set ENABLE_PHOTO_LIMIT to 0 for production (infinite loop)
#define ENABLE_PHOTO_LIMIT  1      // 1 = testing mode, 0 = production mode
#define MAX_PHOTOS          3     // Maximum photos to capture (ignored if ENABLE_PHOTO_LIMIT = 0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//variabel wifi
uint8_t wifi_rx_buffer[WIFI_RX_BUFFER_SIZE];

// Modbus Master untuk komunikasi dengan ESP32-CAM
ModbusMaster_t modbus_master;

// Photo ID and Group ID counters
uint16_t photo_counter = 1;    // Photo ID counter (starts from 1)
uint16_t current_group = 1;    // Current group ID (changes when robot turns)

// OLD: UART receive buffer and state for STM32 <-> ESP32 comms (DEPRECATED - using Modbus now)
// #define RX_LINE_MAX 256
// static uint8_t rx_byte;                // single-byte receiver for interrupt
// static char rx_line[RX_LINE_MAX];      // accumulates a line
// static volatile uint16_t rx_index = 0;
// static volatile uint8_t line_ready = 0; // flag: full line received
// static volatile uint8_t waiting_for_response = 0;
// static const uint32_t COMMAND_TIMEOUT_MS = 5000; // 5s timeout, adjust as needed

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */

// --- FUNGSI BANTU UNTUK WIFI (DEPRECATED - esp-link handles WiFi now) ---
// bool send_and_receive(const char *cmd, uint32_t delay_ms);
// bool ESP01_ConnectWiFi(const char *ssid, const char *password);
// -----------------------------

// --- BUTTON CONTROL FUNCTIONS ---
uint8_t read_green_button(void);  // Returns 1 if pressed, 0 if not
uint8_t read_red_button(void);    // Returns 1 if pressed, 0 if not
// -----------------------------

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h" // Tambahkan ini untuk printf

// Deklarasikan huart1 agar dikenal di fungsi ini
extern UART_HandleTypeDef huart1;

// Fungsi _write untuk printf ke UART1 (USB-TTL) dan UART2 (WiFi ESP-01)
// Sekarang serial output keluar ke 2 tempat: USB debug + WiFi esp-link
int _write(int file, char *ptr, int len)
{
  (void)file; // Tidak digunakan
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100); // USB-TTL debug (115200 baud)
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100); // WiFi esp-link (115200 baud)
  return len;
}

// =================================================================
// == BUTTON CONTROL FUNCTIONS ==
// Active LOW buttons with internal pull-up (pressed = 0, not pressed = 1)
// =================================================================

// Read GREEN button (PC0)
// Returns 1 if pressed, 0 if not pressed
uint8_t read_green_button(void) {
    // Active LOW: Button pressed = GPIO_PIN_RESET (0)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
        HAL_Delay(50);  // Debounce 50ms
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
            return 1;  // Pressed
        }
    }
    return 0;  // Not pressed
}

// Read RED button (PC2)
// Returns 1 if pressed, 0 if not pressed
uint8_t read_red_button(void) {
    // Active LOW: Button pressed = GPIO_PIN_RESET (0)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) {
        HAL_Delay(50);  // Debounce 50ms
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) {
            return 1;  // Pressed
        }
    }
    return 0;  // Not pressed
}

// =================================================================
// == FUNGSI WIFI ESP01 (DEPRECATED - esp-link firmware handles all WiFi) ==
// =================================================================
/*
bool send_at_command(const char* cmd, const char* expected_response, uint32_t timeout)
{
    memset(wifi_rx_buffer, '\0', WIFI_RX_BUFFER_SIZE); // Kosongkan buffer utama
    uint32_t buffer_idx = 0; // Indeks untuk buffer utama

    printf("Mengirim: %s", cmd);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 1000); // Kirim perintah

    uint32_t start_time = HAL_GetTick(); // Catat waktu mulai

    // Loop polling sampai timeout total
    while ((HAL_GetTick() - start_time) < timeout)
    {
        uint8_t byte_received; // Terima satu byte saja

        // Coba terima SATU byte dengan timeout singkat (misal 50ms)
        if (HAL_UART_Receive(&huart2, &byte_received, 1, 50) == HAL_OK)
        {
            // Jika satu byte diterima
            if (buffer_idx < WIFI_RX_BUFFER_SIZE - 1) // Pastikan buffer tidak penuh
            {
                wifi_rx_buffer[buffer_idx++] = byte_received; // Tambahkan byte ke buffer
                wifi_rx_buffer[buffer_idx] = '\0'; // Jaga agar tetap null-terminated

                // Periksa apakah respons yang diharapkan ADA di buffer setiap kali byte baru masuk
                if (strstr((char*)wifi_rx_buffer, expected_response) != NULL) {
                    printf("Respon DITEMUKAN! >> %s\r\n", wifi_rx_buffer);
                    return true; // Berhasil!
                }
            }
            else {
                 printf("Respon >> (Buffer Penuh sebelum respons ditemukan)\r\n");
                 return false; // Buffer penuh, anggap gagal
            }
        }
        // Jika HAL_UART_Receive timeout (tidak ada byte baru), loop lanjut
        // Kita tidak perlu HAL_Delay di sini karena HAL_UART_Receive sudah memberi jeda
    }

    // Jika keluar dari loop (timeout total tercapai)
    printf("Respon >> (Timeout %lu ms, respons '%s' tidak ditemukan)\r\n", timeout, expected_response);
    printf("   Buffer terakhir: %s\r\n", wifi_rx_buffer);
    return false; // Gagal
}

// Fungsi ESP01_ConnectWiFi (bool version) biarkan seperti yang sudah ada di kode bool sebelumnya
bool ESP01_ConnectWiFi(const char *ssid, const char *password) {
    if (!send_at_command("AT\r\n", "OK", 2000)) {
       // printf("ESP8266 tidak merespon 'AT'. Cek kabel/daya.\r\n"); // Komentar ini bisa diaktifkan jika perlu
    }
    if (!send_at_command("AT+RST\r\n", "ready", 5000)) {
       // printf("Modul gagal reset.\r\n");
    }
    if (!send_at_command("AT+CWMODE=1\r\n", "OK", 2000)) {
      //  printf("Gagal set mode station.\r\n");
    }

    char wifiCommand[128];
    snprintf(wifiCommand, sizeof(wifiCommand), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    if (!send_at_command(wifiCommand, "WIFI GOT IP", 20000)) { // Timeout 20 detik
        printf("Gagal konek ke WiFi. Cek SSID/Password/Sinyal.\r\n");
        return false;
    }

    printf("WiFi Terhubung dan Mendapat IP!\r\n");
    return true;
}
*/
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  // Initialize Modbus Master untuk komunikasi dengan ESP32-CAM via UART3
  ModbusMaster_Init(&modbus_master, &huart3, MODBUS_TIMEOUT_MS);
  printf("=== Modbus RTU Master Initialized (UART3, 9600 baud) ===\r\n");

  // OLD: UART interrupt mode (deprecated)
  // HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  // =================================================================
  // == ESP-01 WIFI SETUP (Using esp-link firmware) ==
  // == WiFi configuration handled by esp-link web UI ==
  // == STM32 just sends serial data to UART2, esp-link forwards via WiFi ==
  // =================================================================
  printf("=== ESP-01 (esp-link) Serial-WiFi Bridge Active ===\r\n");
  printf("=== Access via browser: http://<esp-link-IP> ===\r\n");
  printf("=== Or Telnet: telnet <esp-link-IP> 23 ===\r\n\r\n");

  // LED indicator: System ready
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED NYALA (system ready)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Print configuration mode
#if ENABLE_PHOTO_LIMIT
  printf("\r\n========================================\r\n");
  printf("[Config] TESTING MODE ACTIVE\r\n");
  printf("[Config] Will capture maximum %d photos\r\n", MAX_PHOTOS);
  printf("========================================\r\n\r\n");
#else
  printf("\r\n========================================\r\n");
  printf("[Config] PRODUCTION MODE ACTIVE\r\n");
  printf("[Config] Infinite capture loop enabled\r\n");
  printf("========================================\r\n\r\n");
#endif

  // ===================================================================
  // FIX 2: Wait for ESP32-CAM to be ready (Handshake mechanism)
  // ESP32 sets READY flag (0x0006) after WiFi + Session + Camera init
  // ===================================================================
  printf("\r\n[Startup] Waiting for ESP32-CAM initialization...\r\n");
  printf("[Startup] Checking READY flag (Register 0x0006) every 2 seconds...\r\n\r\n");

  uint8_t ready_check_count = 0;
  uint16_t esp32_ready = 0;

  while (1) {
      // Flush RX buffer before critical read (Fix 4)
      ModbusMaster_FlushRxBuffer(modbus_master.huart);

      // Read ESP32 READY register (0x0006)
      int result = ModbusMaster_ReadHoldingRegisters(&modbus_master,
                                                      MODBUS_SLAVE_ADDR,
                                                      MODBUS_REG_ESP32_READY,
                                                      1,
                                                      &esp32_ready);

      if (result == MODBUS_OK && esp32_ready == 1) {
          printf("[Startup] ESP32-CAM is READY! Starting capture sequence...\r\n\r\n");
          break;  // ESP32 ready, exit wait loop
      }

      ready_check_count++;

      if (result == MODBUS_OK) {
          printf("[Startup] ESP32-CAM not ready yet (READY=%d), check #%d, waiting 2s...\r\n",
                 esp32_ready, ready_check_count);
      } else {
          printf("[Startup] Cannot read READY register (error %d), check #%d, waiting 2s...\r\n",
                 result, ready_check_count);
      }

      HAL_Delay(2000);  // Check every 2 seconds

      // Optional: Timeout after 60 seconds (30 checks)
      if (ready_check_count >= 30) {
          printf("\r\n[Startup] WARNING: ESP32-CAM tidak ready setelah 60 detik!\r\n");
          printf("[Startup] Proceeding anyway, but first capture may fail.\r\n\r\n");
          break;
      }
  }

  // ===================================================================
  // FIX 6: Set Group ID once at startup (not every capture)
  // ===================================================================
  printf("[Startup] Setting Group ID to %d (once at startup)...\r\n", current_group);

  // Flush buffer before critical command (Fix 4)
  ModbusMaster_FlushRxBuffer(modbus_master.huart);

  int group_result = ModbusMaster_WriteSingleRegister(&modbus_master,
                                                       MODBUS_SLAVE_ADDR,
                                                       0x0005,  // REG_GROUP_ID
                                                       current_group);
  if (group_result == MODBUS_OK) {
      printf("[Startup] Group ID set successfully.\r\n\r\n");
  } else {
      printf("[Startup] WARNING: Failed to set Group ID (error %d)\r\n\r\n", group_result);
  }

  printf("========================================\r\n");
  printf("[System] READY TO START CAPTURE\r\n");
  printf("[System] Press GREEN button to begin...\r\n");
  printf("========================================\r\n\r\n");

  // ===================================================================
  // WAIT FOR GREEN BUTTON PRESS (Blocking wait)
  // User must press GREEN button to start capture sequence
  // ===================================================================
  while (!read_green_button()) {
      HAL_Delay(100);  // Check every 100ms
  }

  printf("[System] GREEN button pressed! Starting capture sequence...\r\n\r\n");

while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Check photo limit (if enabled)
#if ENABLE_PHOTO_LIMIT
    if (photo_counter > MAX_PHOTOS) {
        printf("\r\n========================================\r\n");
        printf("[System] MAXIMUM PHOTOS REACHED (%d)\r\n", MAX_PHOTOS);
        printf("[System] Total captured: %d photos\r\n", photo_counter - 1);
        printf("[System] Program stopped.\r\n");
        printf("[System] Press RESET button to restart.\r\n");
        printf("========================================\r\n\r\n");
        break;  // Exit main loop
    }
#endif

    // =================================================================
    // CHECK RED BUTTON FOR MANUAL STOP (Non-blocking)
    // User can press RED button anytime to end session
    // =================================================================
    if (read_red_button()) {
        printf("\r\n========================================\r\n");
        printf("[System] RED button pressed! Stopping session...\r\n");
        printf("========================================\r\n\r\n");

        // Send END_SESSION command to ESP32-CAM
        printf("[Modbus] Sending END_SESSION command to ESP32-CAM...\r\n");

        // Flush buffer before command
        ModbusMaster_FlushRxBuffer(modbus_master.huart);

        int result = ModbusMaster_WriteSingleRegister(&modbus_master,
                                                       MODBUS_SLAVE_ADDR,
                                                       0x0007,  // SESSION_CONTROL register
                                                       2);      // END_SESSION command

        if (result == MODBUS_OK) {
            printf("[Modbus] END_SESSION command sent successfully.\r\n");
            printf("[Modbus] Waiting for ESP32 to complete API call...\r\n");
            HAL_Delay(3000);  // Wait for ESP32 to call /session/end API
            printf("[Modbus] Session ended.\r\n");
        } else {
            printf("[Modbus] WARNING: Failed to send END_SESSION (Error: %d)\r\n", result);
            printf("[Modbus] Proceeding to stop anyway.\r\n");
        }

        printf("\r\n========================================\r\n");
        printf("[System] SESSION ENDED\r\n");
        printf("[System] Total photos captured: %d\r\n", photo_counter - 1);
        printf("[System] Press RESET button to restart.\r\n");
        printf("========================================\r\n\r\n");

        break;  // Exit capture loop
    }

    // =================================================================
    // STEP 1: Send Photo ID and Group ID to ESP32-CAM
    // =================================================================
#if ENABLE_PHOTO_LIMIT
    printf("\r\n[Capture #%d / %d] === Starting new capture sequence ===\r\n", photo_counter, MAX_PHOTOS);
#else
    printf("\r\n[Capture #%d] === Starting new capture sequence ===\r\n", photo_counter);
#endif
    printf("[Modbus] Photo ID: %d, Group ID: %d\r\n", photo_counter, current_group);

    // Flush RX buffer before critical command (Fix 4)
    ModbusMaster_FlushRxBuffer(modbus_master.huart);

    // Send Photo ID to ESP32-CAM (Register 0x0004)
    printf("[Modbus] Setting Photo ID to %d (Reg 0x0004)...\r\n", photo_counter);
    int result = ModbusMaster_WriteSingleRegister(&modbus_master,
                                                   MODBUS_SLAVE_ADDR,
                                                   0x0004,  // REG_PHOTO_ID
                                                   photo_counter);
    if (result != MODBUS_OK) {
        printf("[Modbus] ERROR: Gagal set Photo ID (Error code: %d)\r\n", result);
        HAL_Delay(5000);
        continue;
    }
    printf("[Modbus] Photo ID set successfully.\r\n");

    // NOTE: Group ID set once at startup (Fix 6 - optimization)

    // =================================================================
    // STEP 2: Send CAPTURE command to ESP32-CAM
    // =================================================================
    printf("[Modbus] Mengirim perintah CAPTURE ke ESP32-CAM (Reg 0x0001 = 1)...\r\n");

    result = ModbusMaster_SendCaptureCommand(&modbus_master);

    if (result != MODBUS_OK) {
        printf("[Modbus] ERROR: Gagal mengirim command (Error code: %d)\r\n", result);
        if (result == MODBUS_ERR_TIMEOUT) {
            printf("  -> Timeout! Cek koneksi UART atau ESP32-CAM tidak merespon.\r\n");
        } else if (result == MODBUS_ERR_CRC) {
            printf("  -> CRC Error! Data corrupted atau noise di line.\r\n");
        } else if (result == MODBUS_ERR_EXCEPTION) {
            printf("  -> Modbus Exception! ESP32-CAM menolak request.\r\n");
        }
        HAL_Delay(5000);
        continue; // Skip ke iterasi berikutnya
    }

    printf("[Modbus] Command berhasil dikirim! ESP32-CAM acknowledge.\r\n");

    // =================================================================
    // STEP 3: Poll status sampai selesai (max 90 detik for Azure upload)
    // =================================================================
    printf("[Modbus] Polling status ESP32-CAM (max 90 detik)...\r\n");

    result = ModbusMaster_WaitForCompletion(&modbus_master, 90000); // 90 detik timeout (Azure HTTPS + idempotency buffer)

    if (result == MODBUS_OK) {
        printf("[Modbus] SUCCESS! ESP32-CAM selesai capture & upload.\r\n");
        printf("[Modbus] Photo #%03d uploaded successfully (Group %d).\r\n", photo_counter, current_group);

        // Increment photo counter untuk foto berikutnya
        photo_counter++;
        printf("[Counter] Photo counter incremented to: %d\r\n", photo_counter);

    } else if (result == MODBUS_ERR_TIMEOUT) {
        printf("[Modbus] TIMEOUT! ESP32-CAM tidak selesai dalam 90 detik.\r\n");
    } else if (result == MODBUS_ERR_EXCEPTION) {
        printf("[Modbus] ERROR! ESP32-CAM melaporkan kegagalan (status=ERROR).\r\n");

        // Read error code from register 0x0003
        uint16_t error_code = 0;
        if (ModbusMaster_ReadHoldingRegisters(&modbus_master, MODBUS_SLAVE_ADDR,
                                               MODBUS_REG_ERROR_CODE, 1, &error_code) == MODBUS_OK) {
            printf("  -> Error Code: 0x%04X ", error_code);

            // Decode error code
            switch(error_code) {
                case 0x0001: printf("(Camera capture failed)\r\n"); break;
                case 0x0002: printf("(WiFi disconnected)\r\n"); break;
                case 0x0003: printf("(Image upload failed)\r\n"); break;
                case 0x0004: printf("(Metadata upload failed)\r\n"); break;
                case 0x0005: printf("(Session ID retrieval failed)\r\n"); break;
                case 0x0006: printf("(Invalid Photo ID)\r\n"); break;
                default: printf("(Unknown error)\r\n"); break;
            }
        }

        // Optional: Retry logic bisa ditambahkan di sini
        // Untuk sekarang, skip dan lanjut ke foto berikutnya
        printf("[Recovery] Skipping failed capture, moving to next photo...\r\n");
        photo_counter++;  // Increment juga untuk skip photo_id yang gagal
    } else {
        printf("[Modbus] ERROR: Communication error (Error code: %d)\r\n", result);
    }

    // =================================================================
    // STEP 4: Check for group change (navigation logic - Phase 2)
    // =================================================================
    // TODO (Phase 2): Implement group change detection based on ultrasonic sensors
    // When robot turns (detected by navigation logic), increment current_group
    //
    // Example logic:
    // if (robot_turned) {  // Will be detected by HC-SR04 + motor encoder
    //     current_group++;
    //     printf("[Navigation] Robot turned! Group ID incremented to: %d\r\n", current_group);
    // }
    //
    // For now, group_id stays constant at 1 (single straight corridor)

    // Tunggu 5 detik sebelum perintah berikutnya
    printf("\r\nMenunggu 5 detik sebelum capture berikutnya...\r\n");
    HAL_Delay(5000);
}

  // Idle mode after photo limit reached (only executed if ENABLE_PHOTO_LIMIT = 1)
  printf("\r\n[System] Entering idle mode...\r\n");
  printf("[System] LED will blink slowly to indicate idle state.\r\n");
  while (1) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // Blink LED slowly
      HAL_Delay(1000);  // 1 second on, 1 second off
  }

  /* USER CODE END 3 */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  /*Configure GPIO pins : PC0 (GREEN button) and PC2 (RED button) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Active LOW with pull-up
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// OLD CALLBACK - DEPRECATED (Using Modbus RTU polling now)
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) { // Callback utk USART3 (ESP32)
        // ignore carriage return '\r', gunakan '\n' sebagai terminator
        if (rx_byte == '\r') {
            // skip
        } else if (rx_byte == '\n') {
            rx_line[rx_index] = '\0';   // terminate string
            line_ready = 1;             // tandai line siap diproses
            rx_index = 0;               // reset untuk line berikutnya
        } else {
            if (rx_index < (RX_LINE_MAX - 1)) {
                rx_line[rx_index++] = (char)rx_byte;
            } else {
                // overflow: reset index dan flag (jangan crash)
                rx_index = 0;
            }
        }
        // restart interrupt receive untuk byte berikutnya
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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
