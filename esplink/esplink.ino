 /*
   * WiFi Serial Bridge untuk STM32
   * ESP-01 jadi wireless serial monitor via Telnet
   */

  #include <ESP8266WiFi.h>

  // ===== CONFIG WIFI =====
  const char* ssid = "woi";           // Ganti dengan WiFi Anda
  const char* password = "123456781"; // Ganti dengan password WiFi Anda

  // ===== CONFIG SERVER =====
  #define TELNET_PORT 23
  #define SERIAL_BAUD 115200

  WiFiServer server(TELNET_PORT);
  WiFiClient client;

  void setup() {
    // Init Serial (connect ke STM32 UART2)
    Serial.begin(SERIAL_BAUD);

    // Wait for bootloader to finish (1 second)
    delay(1000);

    // Clear any garbage from bootloader
    while(Serial.available() > 0) {
      Serial.read();
    }

    // Now print clean message
    Serial.println("\n\n=== ESP-01 WiFi Serial Bridge ===");

    // Connect WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // Wait for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ WiFi Connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Telnet Port: ");
      Serial.println(TELNET_PORT);
      Serial.println("================================");

      // Start Telnet server
      server.begin();
      server.setNoDelay(true);
    } else {
      Serial.println("\n❌ WiFi connection FAILED!");
      Serial.println("Check SSID/Password and reboot.");
    }
  }

  void loop() {
    // Check for new client connection
    if (server.hasClient()) {
      // Disconnect existing client if any
      if (client && client.connected()) {
        client.stop();
        Serial.println("Old client disconnected");
      }

      // Accept new client
      client = server.available();
      if (client) {
        Serial.println("New Telnet client connected!");
        client.println("=== ESP-01 Serial Bridge ===");
        client.print("Connected to: ");
        client.println(ssid);
        client.println("Ready to receive data...\n");
      }
    }

    // Forward data: Serial → WiFi
    if (Serial.available() > 0) {
      char c = Serial.read();

      // Send to telnet client if connected
      if (client && client.connected()) {
        client.write(c);
      }
    }

    // Forward data: WiFi → Serial (for bidirectional communication)
    if (client && client.connected() && client.available() > 0) {
      char c = client.read();
      Serial.write(c);
    }

    // Small delay to prevent WDT reset
    delay(1);
  }