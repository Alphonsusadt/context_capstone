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

// --- FUNGSI BANTU UNTUK WIFI (LOGIKA ASLI ANDA) ---
bool send_and_receive(const char *cmd, uint32_t delay_ms);
bool ESP01_ConnectWiFi(const char *ssid, const char *password);
// -----------------------------

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h" // Tambahkan ini untuk printf

// Deklarasikan huart1 agar dikenal di fungsi ini
extern UART_HandleTypeDef huart1;

// Fungsi _write untuk printf ke UART1
int _write(int file, char *ptr, int len)
{
  (void)file; // Tidak digunakan
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}



// =================================================================
// == FUNGSI WIFI ESP01 (Menggunakan UART2) -- VERSI bool (POLLING AKTIF) ==
// =================================================================
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
  // == LOGIKA SETUP WIFI DENGAN PENGECEKAN HASIL (bool) ==
  // =================================================================
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Mulai dengan LED MATI

  printf("=== Memulai koneksi WiFi (ESP-01) ===\r\n");

  // Panggil fungsi koneksi WiFi dan CEK HASILNYA
  if (ESP01_ConnectWiFi(WIFI_SSID, WIFI_PASS) == true)
  {
      // Jika KONEKSI BERHASIL
      printf("=== WiFi (ESP-01) Berhasil Terhubung! ===\r\n");
      // Nyalakan LED sebagai indikator sukses
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED NYALA
      printf("Melanjutkan ke program utama (Modbus komunikasi ke ESP32-CAM)...\r\n\r\n");
  }
  else
  {
      // Jika KONEKSI GAGAL
      printf("=== WiFi (ESP-01) GAGAL Terhubung! ===\r\n");
      printf("Program berhenti. Cek SSID/Password/Sinyal/Koneksi ESP-01.\r\n");

      // Hentikan program jika WiFi STM32 gagal.
      // LED akan berkedip sebagai tanda error.
      while(1)
      {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Blink LED error
          HAL_Delay(100); // Kedip cepat
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // =================================================================
    // MODBUS RTU: Send CAPTURE command to ESP32-CAM
    // =================================================================
    printf("\r\n[Modbus] Mengirim perintah CAPTURE ke ESP32-CAM (Reg 0x0001 = 1)...\r\n");

    int result = ModbusMaster_SendCaptureCommand(&modbus_master);

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
    // MODBUS RTU: Poll status sampai selesai (max 30 detik)
    // =================================================================
    printf("[Modbus] Polling status ESP32-CAM (max 30 detik)...\r\n");

    result = ModbusMaster_WaitForCompletion(&modbus_master, 30000); // 30 detik timeout

    if (result == MODBUS_OK) {
        printf("[Modbus] SUCCESS! ESP32-CAM selesai capture & upload.\r\n");
    } else if (result == MODBUS_ERR_TIMEOUT) {
        printf("[Modbus] TIMEOUT! ESP32-CAM tidak selesai dalam 30 detik.\r\n");
    } else if (result == MODBUS_ERR_EXCEPTION) {
        printf("[Modbus] ERROR! ESP32-CAM melaporkan kegagalan (status=ERROR).\r\n");

        // Optional: Read error code from register 0x0003
        uint16_t error_code = 0;
        if (ModbusMaster_ReadHoldingRegisters(&modbus_master, MODBUS_SLAVE_ADDR,
                                               MODBUS_REG_ERROR_CODE, 1, &error_code) == MODBUS_OK) {
            printf("  -> Error Code: 0x%04X\r\n", error_code);
        }
    } else {
        printf("[Modbus] ERROR: Communication error (Error code: %d)\r\n", result);
    }

    // Tunggu 5 detik sebelum perintah berikutnya
    printf("Menunggu 5 detik sebelum perintah berikutnya...\r\n");
    HAL_Delay(5000);
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
