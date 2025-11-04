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
#include <WiFiClientSecure.h> // For HTTPS support
#include "board_config.h"
#include <ModbusRTU.h>        // Modbus RTU library

// --- WIFI CONFIGURATION ---
const char* ssid = "woi";
const char* password = "123456781";

// --- SERVER CONFIGURATION ---
// Toggle between Development (local Flask) and Production (Azure/Vercel)
#define USE_PRODUCTION_BACKEND  true  // true = Azure, false = Local Flask

#if USE_PRODUCTION_BACKEND
  // PRODUCTION: Azure Backend (Vercel)
  const char* serverURL_session = "https://test-capstone-backend-azure.vercel.app/session/current";
  const char* serverURL_image = "https://test-capstone-backend-azure.vercel.app/upload/image";
  const char* serverURL_meta = "https://test-capstone-backend-azure.vercel.app/upload/meta";
  #define SERVER_MODE "PRODUCTION (Azure)"
#else
  // DEVELOPMENT: Local Flask Server
  const char* serverURL_session = "http://10.32.207.112:5000/session/current";
  const char* serverURL_image = "http://10.32.207.112:5000/upload/image";
  const char* serverURL_meta = "http://10.32.207.112:5000/upload/meta";
  #define SERVER_MODE "DEVELOPMENT (Local)"
#endif

// --- DEBUG MODE ---
#define SKIP_UPLOAD_FOR_TEST  false  // Set ke false jika server sudah ready

// --- MODBUS CONFIGURATION ---
#define MODBUS_SLAVE_ID     0x01
#define MODBUS_BAUDRATE     9600

// Modbus Register Addresses
#define REG_COMMAND         0x0001
#define REG_STATUS          0x0002
#define REG_ERROR_CODE      0x0003
#define REG_PHOTO_ID        0x0004  // Photo ID (integer: 1, 2, 3, ...)
#define REG_GROUP_ID        0x0005  // Group ID (integer: 1, 1, 2, 2, ...)
#define REG_ESP32_READY     0x0006  // ESP32 Ready flag (Fix 2: Handshake)

// Command Values
#define CMD_IDLE            0x0000
#define CMD_CAPTURE         0x0001

// Status Values
#define STATUS_IDLE         0x0000
#define STATUS_BUSY         0x0001
#define STATUS_SUCCESS      0x0002
#define STATUS_ERROR        0x0003

// Error Code Values
#define ERR_NONE                0x0000
#define ERR_CAMERA_FAIL         0x0001
#define ERR_WIFI_FAIL           0x0002
#define ERR_UPLOAD_IMAGE_FAIL   0x0003
#define ERR_UPLOAD_META_FAIL    0x0004
#define ERR_SESSION_FAIL        0x0005
#define ERR_INVALID_PHOTO_ID    0x0006

// --- UART CONFIGURATION FOR MODBUS ---
#define STM32_RXD 13
#define STM32_TXD 12

// Modbus RTU instance
ModbusRTU mb;

// Register values (volatile karena bisa diakses dari callback)
volatile uint16_t reg_command = CMD_IDLE;
volatile uint16_t reg_status = STATUS_IDLE;
volatile uint16_t reg_error_code = ERR_NONE;
volatile uint16_t reg_photo_id = 0;      // Photo ID dari STM32
volatile uint16_t reg_group_id = 0;      // Group ID dari STM32
volatile uint16_t reg_esp32_ready = 0;   // Ready flag (Fix 2: Handshake) - 0=not ready, 1=ready

// Session management
String current_session_id = "";          // Session ID dari server (diambil saat startup)

// Task flag
volatile bool task_pending = false;

// ===================================================================
// FIX: Idempotency - Track last completed photo ID
// Prevents duplicate capture if STM32 timeouts and retries
// ===================================================================
volatile uint16_t last_completed_photo_id = 0;  // Track last successfully completed photo

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
  // Handle photo_id register write
  else if (addr == REG_PHOTO_ID) {
    reg_photo_id = val;
    Serial.printf("[Modbus] Photo ID set to: %d\n", val);
  }
  // Handle group_id register write
  else if (addr == REG_GROUP_ID) {
    reg_group_id = val;
    Serial.printf("[Modbus] Group ID set to: %d\n", val);
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
  } else if (addr == REG_PHOTO_ID) {
    return reg_photo_id;
  } else if (addr == REG_GROUP_ID) {
    return reg_group_id;
  }

  return val;
}

// ===== HELPER FUNCTIONS =====

// Function untuk GET session_id dari server
bool getSessionID() {
  Serial.println("\n[Session] Retrieving session ID from server...");
  Serial.printf("[Session] Target URL: %s\n", serverURL_session);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Session] ERROR: WiFi not connected!");
    return false;
  }

  HTTPClient http;
  WiFiClientSecure *client = nullptr;  // Initialize to null

  // For HTTPS (production), use WiFiClientSecure
  #if USE_PRODUCTION_BACKEND
    client = new WiFiClientSecure;
    if (client) {
      client->setInsecure();  // Skip SSL certificate verification
      http.begin(*client, serverURL_session);
      Serial.println("[Session] Using HTTPS (SSL verification: DISABLED)");
    }
  #else
    http.begin(serverURL_session);
  #endif

  http.setTimeout(20000); // 20 second timeout (increased for HTTPS)

  int httpCode = http.GET();
  bool result = false;

  if (httpCode > 0) {
    Serial.printf("[Session] Response code: %d\n", httpCode);

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("[Session] Response payload:");
      Serial.println(payload);

      // Parse JSON response
      // Expected: {"data": {"next_session_id": "session_001", ...}, ...}
      // Simple string parsing (bisa pakai ArduinoJson untuk lebih robust)
      int idx_start = payload.indexOf("\"next_session_id\":\"");
      if (idx_start > 0) {
        idx_start += 19; // Skip "next_session_id":"
        int idx_end = payload.indexOf("\"", idx_start);
        if (idx_end > idx_start) {
          current_session_id = payload.substring(idx_start, idx_end);
          Serial.printf("[Session] SUCCESS! Session ID: %s\n", current_session_id.c_str());
          result = true;
        }
      }

      if (!result) {
        Serial.println("[Session] ERROR: Failed to parse session_id from response!");
      }
    }
  } else {
    Serial.printf("[Session] ERROR: HTTP request failed, code: %d\n", httpCode);
    Serial.println("[Session] Error: " + http.errorToString(httpCode));
  }

  // Cleanup
  http.end();
  #if USE_PRODUCTION_BACKEND
    if (client) {
      delete client;  // Free WiFiClientSecure memory
      client = nullptr;
    }
  #endif

  return result;
}

// Function untuk upload image (multipart form-data)
bool uploadImage(camera_fb_t *fb, const char* photo_id_str) {
  Serial.println("\n[Upload Image] Starting upload...");

  HTTPClient http;
  WiFiClientSecure *client = nullptr;  // Initialize to null

  // For HTTPS (production), use WiFiClientSecure
  #if USE_PRODUCTION_BACKEND
    client = new WiFiClientSecure;
    if (client) {
      client->setInsecure();  // Skip SSL certificate verification
      http.begin(*client, serverURL_image);
      Serial.println("[Upload Image] Using HTTPS connection");
    }
  #else
    http.begin(serverURL_image);
  #endif

  http.setTimeout(30000); // 30 second timeout (increased for HTTPS)
  http.setReuse(false);   // Don't reuse connection (avoid stale connection issues)

  // Create multipart boundary
  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  String content_type = "multipart/form-data; boundary=" + boundary;
  http.addHeader("Content-Type", content_type);

  // Build multipart body
  String head = "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"photo_id\"\r\n\r\n";
  head += String(photo_id_str) + "\r\n";
  head += "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"file\"; filename=\"" + String(photo_id_str) + ".jpg\"\r\n";
  head += "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";

  uint32_t total_len = head.length() + fb->len + tail.length();

  // Allocate buffer untuk full body
  uint8_t *full_body = (uint8_t*)malloc(total_len);
  if (!full_body) {
    Serial.println("[Upload Image] ERROR: Failed to allocate memory!");
    return false;
  }

  // Copy head
  memcpy(full_body, head.c_str(), head.length());
  // Copy image data
  memcpy(full_body + head.length(), fb->buf, fb->len);
  // Copy tail
  memcpy(full_body + head.length() + fb->len, tail.c_str(), tail.length());

  Serial.printf("[Upload Image] Total size: %d bytes\n", total_len);
  Serial.printf("[Upload Image] Photo ID: %s\n", photo_id_str);

  // Send POST request (BLOCKING - takes 5-10 seconds for HTTPS)
  Serial.println("[Upload Image] Sending POST request (this may take 5-10 seconds for HTTPS)...");
  int httpCode = http.POST(full_body, total_len);

  free(full_body); // Free memory

  // CRITICAL: Process Modbus extensively after POST completes
  // STM32 might have polled status multiple times during POST
  Serial.println("[Upload Image] POST complete, processing Modbus requests...");
  for (int i = 0; i < 50; i++) {  // Increased from 5 to 50
    mb.task();
    delay(10);  // Total: 500ms of Modbus processing
  }

  bool result = false;

  if (httpCode > 0) {
    Serial.printf("[Upload Image] Response code: %d\n", httpCode);

    if (httpCode >= 200 && httpCode < 300) {
      String response = http.getString();
      Serial.println("[Upload Image] Response: " + response);
      Serial.println("[Upload Image] SUCCESS!");
      result = true;
    } else {
      String response = http.getString();
      Serial.printf("[Upload Image] ERROR: Server returned code %d\n", httpCode);
      Serial.println("[Upload Image] Response: " + response);
      result = false;
    }
  } else {
    Serial.printf("[Upload Image] ERROR: Request failed, code: %d\n", httpCode);
    Serial.println("[Upload Image] Error: " + http.errorToString(httpCode));
    result = false;
  }

  // Cleanup
  http.end();
  #if USE_PRODUCTION_BACKEND
    if (client) {
      delete client;  // Free WiFiClientSecure memory
      client = nullptr;
    }
  #endif

  return result;
}

// Function untuk upload metadata (JSON body)
bool uploadMetadata(const char* photo_id_str, uint16_t group_id) {
  Serial.println("\n[Upload Meta] Starting metadata upload...");

  HTTPClient http;
  WiFiClientSecure *client = nullptr;  // Initialize to null

  // For HTTPS (production), use WiFiClientSecure
  #if USE_PRODUCTION_BACKEND
    client = new WiFiClientSecure;
    if (client) {
      client->setInsecure();  // Skip SSL certificate verification
      http.begin(*client, serverURL_meta);
      Serial.println("[Upload Meta] Using HTTPS connection");
    }
  #else
    http.begin(serverURL_meta);
  #endif

  http.addHeader("Content-Type", "application/json");
  http.setTimeout(20000); // 20 second timeout (increased for HTTPS)
  http.setReuse(false);   // Don't reuse connection

  // Build JSON payload
  String json_payload = "{";
  json_payload += "\"photo_id\":\"" + String(photo_id_str) + "\",";
  json_payload += "\"session_id\":\"" + current_session_id + "\",";
  json_payload += "\"group_id\":\"" + String(group_id) + "\"";
  json_payload += "}";

  Serial.println("[Upload Meta] Payload: " + json_payload);

  // Send POST request (BLOCKING - takes 1-3 seconds for HTTPS)
  Serial.println("[Upload Meta] Sending POST request...");
  int httpCode = http.POST(json_payload);

  // CRITICAL: Process Modbus extensively after POST completes
  Serial.println("[Upload Meta] POST complete, processing Modbus requests...");
  for (int i = 0; i < 50; i++) {  // Increased from 5 to 50
    mb.task();
    delay(10);  // Total: 500ms of Modbus processing
  }

  bool result = false;

  if (httpCode > 0) {
    Serial.printf("[Upload Meta] Response code: %d\n", httpCode);

    if (httpCode >= 200 && httpCode < 300) {
      String response = http.getString();
      Serial.println("[Upload Meta] Response: " + response);
      Serial.println("[Upload Meta] SUCCESS!");
      result = true;
    } else {
      String response = http.getString();
      Serial.printf("[Upload Meta] ERROR: Server returned code %d\n", httpCode);
      Serial.println("[Upload Meta] Response: " + response);
      result = false;
    }
  } else {
    Serial.printf("[Upload Meta] ERROR: Request failed, code: %d\n", httpCode);
    Serial.println("[Upload Meta] Error: " + http.errorToString(httpCode));
    result = false;
  }

  // Cleanup
  http.end();
  #if USE_PRODUCTION_BACKEND
    if (client) {
      delete client;  // Free WiFiClientSecure memory
      client = nullptr;
    }
  #endif

  return result;
}

void setup() {
  Serial.begin(115200); // Untuk Serial Monitor (Debug PC)
  Serial.println("\n=== ESP32-CAM Modbus RTU Slave ===");
  Serial.printf("[Config] Server Mode: %s\n", SERVER_MODE);

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

    // === GET Session ID dari server ===
    if (!getSessionID()) {
      Serial.println("[WiFi] WARNING: Failed to get session_id!");
      Serial.println("[WiFi] Will continue with empty session_id...");
      // Don't halt, masih bisa lanjut (tapi upload akan gagal)
    }
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
  mb.addHreg(REG_COMMAND, reg_command);       // Command register
  mb.addHreg(REG_STATUS, reg_status);         // Status register
  mb.addHreg(REG_ERROR_CODE, reg_error_code); // Error code register
  mb.addHreg(REG_PHOTO_ID, reg_photo_id);     // Photo ID register
  mb.addHreg(REG_GROUP_ID, reg_group_id);     // Group ID register
  mb.addHreg(REG_ESP32_READY, reg_esp32_ready); // ESP32 Ready flag (Fix 2)

  // Set callbacks
  mb.onSetHreg(REG_COMMAND, cbWrite);         // Callback saat Master write command
  mb.onSetHreg(REG_PHOTO_ID, cbWrite);        // Callback saat Master write photo_id
  mb.onSetHreg(REG_GROUP_ID, cbWrite);        // Callback saat Master write group_id
  mb.onGetHreg(REG_STATUS, cbRead);           // Callback saat Master read status
  mb.onGetHreg(REG_ERROR_CODE, cbRead);       // Callback saat Master read error code
  mb.onGetHreg(REG_PHOTO_ID, cbRead);         // Callback saat Master read photo_id
  mb.onGetHreg(REG_GROUP_ID, cbRead);         // Callback saat Master read group_id

  Serial.printf("[Modbus] Slave initialized (ID: 0x%02X, Baud: %d)\n", MODBUS_SLAVE_ID, MODBUS_BAUDRATE);
  Serial.println("[Modbus] Register Map:");
  Serial.println("  0x0001: Command (1=CAPTURE, 0=IDLE)");
  Serial.println("  0x0002: Status (0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR)");
  Serial.println("  0x0003: Error Code (0-6)");
  Serial.println("  0x0004: Photo ID (integer: 1, 2, 3, ...)");
  Serial.println("  0x0005: Group ID (integer: 1, 1, 2, 2, ...)");
  Serial.println("  0x0006: ESP32 Ready (0=not ready, 1=ready)");

  // ===================================================================
  // FIX 2: Set READY flag after all initialization complete
  // This allows STM32 to wait until ESP32 is fully initialized
  // ===================================================================
  reg_esp32_ready = 1;  // Signal to STM32 that we're ready
  mb.Hreg(REG_ESP32_READY, reg_esp32_ready);

  Serial.println("\n========================================");
  Serial.println("ESP32-CAM FULLY INITIALIZED AND READY!");
  Serial.println("========================================");
  Serial.println("[System] WiFi connected, Session ID fetched, Camera ready");
  Serial.println("[System] Modbus slave active - Ready to receive commands from STM32");
  Serial.println("========================================\n");
}

// Function untuk capture dan upload image + metadata
void captureAndUpload() {
  Serial.println("\n[Task] ===== Starting capture and upload process =====");

  // Read photo_id and group_id from Modbus registers
  uint16_t photo_id = reg_photo_id;
  uint16_t group_id = reg_group_id;

  // Validate photo_id (must be > 0)
  if (photo_id == 0) {
    Serial.println("[Task] ERROR: Invalid photo_id (0)! STM32 must set photo_id first.");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_INVALID_PHOTO_ID;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    return;
  }

  // ===================================================================
  // IDEMPOTENCY CHECK (Fix for duplicate command bug)
  // If this photo_id was already completed, return SUCCESS immediately
  // This prevents re-capturing if STM32 times out and retries
  // ===================================================================
  if (photo_id <= last_completed_photo_id) {
    Serial.println("\n========================================");
    Serial.printf("⚠️  [Idempotency] Photo ID %d already completed!\n", photo_id);
    Serial.printf("[Idempotency] Last completed ID: %d\n", last_completed_photo_id);
    Serial.println("[Idempotency] Returning SUCCESS immediately (no re-capture)");
    Serial.println("========================================\n");

    // Set SUCCESS status immediately
    reg_status = STATUS_SUCCESS;
    reg_error_code = ERR_NONE;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);

    // Process Modbus extensively to ensure STM32 can read status
    Serial.println("[Modbus] Processing Modbus for 2 seconds to ensure STM32 can read status...");
    for (int i = 0; i < 200; i++) {
      mb.task();
      delay(10);
    }

    Serial.println("[Idempotency] Status ready for polling. Exiting without capture.\n");
    return;  // Skip actual capture/upload
  }

  // Format photo_id ke string "001", "002", etc.
  char photo_id_str[8];
  sprintf(photo_id_str, "%03d", photo_id);

  Serial.printf("[Task] Photo ID: %d (%s) - NEW (last completed: %d)\n",
                photo_id, photo_id_str, last_completed_photo_id);
  Serial.printf("[Task] Group ID: %d\n", group_id);
  Serial.printf("[Task] Session ID: %s\n", current_session_id.c_str());

  // Set status ke BUSY
  reg_status = STATUS_BUSY;
  reg_error_code = ERR_NONE;
  mb.Hreg(REG_STATUS, reg_status);
  mb.Hreg(REG_ERROR_CODE, reg_error_code);
  Serial.printf("[Status] Updated: BUSY\n");

  // Process Modbus during task - CRITICAL!
  for (int i = 0; i < 10; i++) {
    mb.task();
    delay(5);
  }

  // === Step 1: Capture Image ===
  Serial.println("\n[Camera] Capturing image...");
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("[Camera] ERROR: Failed to capture image!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_CAMERA_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    Serial.printf("[Status] Updated: ERROR - Camera failed\n");
    return;
  }

  Serial.printf("[Camera] Image captured successfully (%zu bytes, %dx%d)\n",
                fb->len, fb->width, fb->height);

  // Process Modbus after capture
  for (int i = 0; i < 5; i++) {
    mb.task();
    delay(5);
  }

  // === Step 2: Check WiFi and Session ID ===
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Upload] ERROR: WiFi not connected!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_WIFI_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    esp_camera_fb_return(fb);
    return;
  }

  if (current_session_id.length() == 0) {
    Serial.println("[Upload] ERROR: Session ID is empty!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_SESSION_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    esp_camera_fb_return(fb);
    return;
  }

#if SKIP_UPLOAD_FOR_TEST
  // === TEST MODE: Simulate upload ===
  Serial.println("\n[Upload] TEST MODE: Simulating uploads...");
  delay(500);

  for (int i = 0; i < 10; i++) {
    mb.task();
    delay(50);
  }

  Serial.println("[Upload] TEST MODE: Image upload simulated (SUCCESS)");
  Serial.println("[Upload] TEST MODE: Metadata upload simulated (SUCCESS)");

  reg_status = STATUS_SUCCESS;
  reg_error_code = ERR_NONE;
#else
  // === PRODUCTION MODE: Real upload ===

  // Step 3: Upload Image (multipart)
  bool image_ok = uploadImage(fb, photo_id_str);

  if (!image_ok) {
    Serial.println("[Task] ERROR: Image upload failed!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_UPLOAD_IMAGE_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    esp_camera_fb_return(fb);
    return;
  }

  // Step 4: Upload Metadata (JSON) - ONLY if image upload success
  bool meta_ok = uploadMetadata(photo_id_str, group_id);

  if (!meta_ok) {
    Serial.println("[Task] ERROR: Metadata upload failed!");
    reg_status = STATUS_ERROR;
    reg_error_code = ERR_UPLOAD_META_FAIL;
    mb.Hreg(REG_STATUS, reg_status);
    mb.Hreg(REG_ERROR_CODE, reg_error_code);
    esp_camera_fb_return(fb);
    return;
  }

  // === Step 5: Success! ===
  Serial.println("\n[Task] SUCCESS! Both image and metadata uploaded.");
  reg_status = STATUS_SUCCESS;
  reg_error_code = ERR_NONE;
#endif

  // Return frame buffer
  esp_camera_fb_return(fb);

  // Update Modbus registers
  mb.Hreg(REG_STATUS, reg_status);
  mb.Hreg(REG_ERROR_CODE, reg_error_code);
  Serial.printf("[Status] Updated: Final status=0x%04X, error=0x%04X\n", reg_status, reg_error_code);

  // ===================================================================
  // UPDATE LAST COMPLETED PHOTO ID (Idempotency tracking)
  // Only update after successful upload to prevent duplicate captures
  // ===================================================================
  last_completed_photo_id = photo_id;
  Serial.printf("[Idempotency] Updated last_completed_photo_id: %d\n", last_completed_photo_id);

  // Reset command register
  reg_command = CMD_IDLE;
  mb.Hreg(REG_COMMAND, reg_command);

  // Important: Process Modbus to ensure status is readable by Master
  // Increased iterations for Azure production (HTTPS takes longer)
  Serial.println("[Modbus] Processing Modbus for 2 seconds to ensure STM32 can read status...");
  for (int i = 0; i < 200; i++) {
    mb.task();
    delay(10);
  }

  Serial.println("[Task] ===== Task finished, status ready for polling =====\n");
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