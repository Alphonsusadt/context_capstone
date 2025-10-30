/**************************************************************
 * ESP32-CAM (MODBUS RTU SLAVE) - CAPSTONE PROJECT
 * Komunikasi dengan STM32 Master via Modbus RTU over UART
 * Pin RX di GPIO 13, Pin TX di GPIO 12
 *
 * LIBRARY REQUIREMENT:
 * Install "ModbusRTU by emelianov" from Arduino Library Manager
 * atau download dari: https://github.com/emelianov/modbus-esp8266
 *
 * REGISTER MAP:
 * - 0x0001: Command Register (1=CAPTURE, 0=IDLE)
 * - 0x0002: Status Register (0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR)
 * - 0x0003: Error Code Register
 **************************************************************/

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>       // For HTTP upload to server
#include "board_config.h"
#include <ModbusRTU.h>        // Modbus RTU library

// --- WIFI CONFIGURATION ---
const char* ssid = "woi";
const char* password = "123456781";

// --- SERVER CONFIGURATION ---
const char* serverURL = "http://10.32.207.112:5000/upload"; // Ganti dengan URL server Anda
// Port 5000 untuk Windows (port 80 perlu admin di Windows)

// --- DEBUG MODE ---
#define SKIP_UPLOAD_FOR_TEST  false  // Set ke false jika server sudah ready

// --- MODBUS CONFIGURATION ---
#define MODBUS_SLAVE_ID     0x01
#define MODBUS_BAUDRATE     9600

// Modbus Register Addresses
#define REG_COMMAND         0x0001
#define REG_STATUS          0x0002
#define REG_ERROR_CODE      0x0003

// Command Values
#define CMD_IDLE            0x0000
#define CMD_CAPTURE         0x0001

// Status Values
#define STATUS_IDLE         0x0000
#define STATUS_BUSY         0x0001
#define STATUS_SUCCESS      0x0002
#define STATUS_ERROR        0x0003

// Error Code Values
#define ERR_NONE            0x0000
#define ERR_CAMERA_FAIL     0x0001
#define ERR_WIFI_FAIL       0x0002
#define ERR_UPLOAD_FAIL     0x0003

// --- UART CONFIGURATION FOR MODBUS ---
#define STM32_RXD 13
#define STM32_TXD 12

// Modbus RTU instance
ModbusRTU mb;

// Register values (volatile karena bisa diakses dari callback)
volatile uint16_t reg_command = CMD_IDLE;
volatile uint16_t reg_status = STATUS_IDLE;
volatile uint16_t reg_error_code = ERR_NONE;

// Task flag
volatile bool task_pending = false;

// Callback untuk Modbus - dipanggil ketika Master menulis register
uint16_t cbWrite(TRegister* reg, uint16_t val) {
  // Convert TAddress ke uint16_t untuk comparison
  uint16_t addr = reg->address.address;

  Serial.printf("[Modbus] Write to register 0x%04X, value: 0x%04X\n", addr, val);

  // Handle command register write
  if (addr == REG_COMMAND) {
    if (val == CMD_CAPTURE) {
      Serial.println("[Task] CAPTURE command received!");
      reg_command = CMD_CAPTURE;
      task_pending = true;  // Set flag untuk execute task di loop()
    } else {
      reg_command = CMD_IDLE;
    }
  }

  return val; // Return value yang ditulis
}

// Callback untuk Modbus - dipanggil ketika Master membaca register
uint16_t cbRead(TRegister* reg, uint16_t val) {
  // Convert TAddress ke uint16_t untuk comparison
  uint16_t addr = reg->address.address;

  // Update register values sebelum dibaca Master
  if (addr == REG_STATUS) {
    return reg_status;
  } else if (addr == REG_ERROR_CODE) {
    return reg_error_code;
  } else if (addr == REG_COMMAND) {
    return reg_command;
  }

  return val;
}

void setup() {
  Serial.begin(115200); // Untuk Serial Monitor (Debug PC)
  Serial.println("\n=== ESP32-CAM Modbus RTU Slave ===");

  // === Initialize Camera ===
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] GAGAL init kamera: 0x%x\n", err);
    while(1) delay(1000); // Halt jika kamera gagal
  }
  Serial.println("[Camera] Kamera OK!");

  // === Connect to WiFi ===
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int wifi_retry = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retry < 20) {
    delay(500);
    Serial.print(".");
    wifi_retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Failed to connect! Continuing without WiFi...");
    // Set error flag tapi jangan halt, Modbus masih bisa jalan
  }

  // === Setup Modbus RTU Slave ===
  Serial.println("[Modbus] Initializing Modbus RTU Slave...");

  // Setup Serial2 untuk Modbus
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, STM32_RXD, STM32_TXD);

  // Initialize Modbus RTU dengan Serial2
  mb.begin(&Serial2);
  mb.slave(MODBUS_SLAVE_ID);

  // Add registers dengan callback
  mb.addHreg(REG_COMMAND, reg_command);     // Command register
  mb.addHreg(REG_STATUS, reg_status);       // Status register
  mb.addHreg(REG_ERROR_CODE, reg_error_code); // Error code register

  // Set callbacks
  mb.onSetHreg(REG_COMMAND, cbWrite);       // Callback saat Master write command
  mb.onGetHreg(REG_STATUS, cbRead);         // Callback saat Master read status
  mb.onGetHreg(REG_ERROR_CODE, cbRead);     // Callback saat Master read error code

  Serial.printf("[Modbus] Slave initialized (ID: 0x%02X, Baud: %d)\n", MODBUS_SLAVE_ID, MODBUS_BAUDRATE);
  Serial.println("[Modbus] Register Map:");
  Serial.println("  0x0001: Command (1=CAPTURE, 0=IDLE)");
  Serial.println("  0x0002: Status (0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR)");
  Serial.println("  0x0003: Error Code");
  Serial.println("\n[System] Ready to receive Modbus commands from STM32!\n");
}

// Function untuk capture dan upload image
void captureAndUpload() {
  Serial.println("\n[Task] Starting capture and upload process...");

  // Set status ke BUSY
  reg_status = STATUS_BUSY;
  reg_error_code = ERR_NONE;
  mb.Hreg(REG_STATUS, reg_status);
  mb.Hreg(REG_ERROR_CODE, reg_error_code);
  Serial.printf("[Status] Updated: BUSY (status=0x%04X)\n", reg_status);

  // Process Modbus during task - CRITICAL!
  for (int i = 0; i < 10; i++) {
    mb.task();
    delay(5);
  }

  // === Step 1: Capture Image ===
  Serial.println("[Camera] Capturing image...");
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("[Camera] ERROR: Failed to capture image!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_CAMERA_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    Serial.printf("[Status] Updated: ERROR (status=0x%04X, error=0x%04X)\n", reg_status, reg_error_code);
    return;
  }

  Serial.printf("[Camera] Image captured successfully (%zu bytes, %dx%d)\n",
                fb->len, fb->width, fb->height);

  // Process Modbus after capture
  for (int i = 0; i < 5; i++) {
    mb.task();
    delay(5);
  }

  // === Step 2: Upload to Server ===
  bool upload_success = false;

#if SKIP_UPLOAD_FOR_TEST
  // TEST MODE: Skip upload, simulate success
  Serial.println("[Upload] TEST MODE: Skipping upload, simulating success...");
  delay(500); // Simulate upload time

  // Process Modbus during simulated upload
  for (int i = 0; i < 10; i++) {
    mb.task();
    delay(50);
  }

  upload_success = true;
  Serial.println("[Upload] TEST MODE: Upload simulated successfully!");
#else
  // PRODUCTION MODE: Real upload
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[Upload] Uploading to server...");

    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "image/jpeg");
    http.setTimeout(10000); // 10 second timeout

    // Send POST request dengan image data
    int httpResponseCode = http.POST(fb->buf, fb->len);

    // Process Modbus after HTTP request
    for (int i = 0; i < 5; i++) {
      mb.task();
      delay(5);
    }

    if (httpResponseCode > 0) {
      Serial.printf("[Upload] Success! Response code: %d\n", httpResponseCode);
      String response = http.getString();
      Serial.println("[Upload] Server response: " + response);
      upload_success = true;
    } else {
      Serial.printf("[Upload] ERROR: Failed! Error code: %d\n", httpResponseCode);
      Serial.println("[Upload] Error: " + http.errorToString(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("[Upload] ERROR: WiFi not connected!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_WIFI_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    Serial.printf("[Status] Updated: ERROR (status=0x%04X, error=0x%04X)\n", reg_status, reg_error_code);
    esp_camera_fb_return(fb);
    return;
  }
#endif

  // Return frame buffer
  esp_camera_fb_return(fb);

  // === Step 3: Update Status ===
  if (upload_success) {
    Serial.println("[Task] Task completed successfully!\n");
    reg_status = STATUS_SUCCESS;
    reg_error_code = ERR_NONE;
  } else {
    Serial.println("[Task] Task failed: Upload error\n");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_UPLOAD_FAIL;
  }

  // Update Modbus registers
  mb.Hreg(REG_STATUS, reg_status);
  mb.Hreg(REG_ERROR_CODE, reg_error_code);
  Serial.printf("[Status] Updated: Final status=0x%04X, error=0x%04X\n", reg_status, reg_error_code);

  // Reset command register
  reg_command = CMD_IDLE;
  mb.Hreg(REG_COMMAND, reg_command);

  // Important: Process Modbus to ensure status is readable by Master
  for (int i = 0; i < 20; i++) {
    mb.task();
    delay(5);
  }

  Serial.println("[Task] Task finished, status ready for polling.\n");
}

void loop() {
  // === Process Modbus requests ===
  mb.task();  // PENTING: Harus dipanggil di setiap loop untuk handle Modbus communication

  // === Execute pending task ===
  if (task_pending) {
    task_pending = false;  // Clear flag
    captureAndUpload();    // Execute capture & upload
  }

  // Small delay untuk stabilitas
  delay(10);
}