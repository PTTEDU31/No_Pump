/**
 * @file BasicExample.ino
 * @brief Basic example of ATCommandLib usage
 * 
 * This example shows:
 * - Initialization
 * - Sending async commands
 * - Sending sync commands
 * - Registering URC handlers
 * - Using callbacks
 */

#include <ATCommandLib.h>

// Define serial for modem
#define MODEM_SERIAL Serial1

// Create AT manager instance
ATCommandManager* atManager = nullptr;

// Global variables for command results
int signalQuality = -1;
bool mqttConnected = false;

// ================== CALLBACK FUNCTIONS ==================

// Callback for AT command (basic check)
void onATResponse(ATResponseType type, const char* response, void* userData) {
  Serial.print("AT Response: ");
  
  switch (type) {
    case ATResponseType::OK:
      Serial.println("OK");
      break;
    case ATResponseType::ERROR:
      Serial.println("ERROR");
      break;
    case ATResponseType::TIMEOUT:
      Serial.println("TIMEOUT");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
}

// Callback for CSQ command (signal quality)
void onCSQResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    Serial.print("CSQ Response: ");
    Serial.println(response);
    
    // Parse +CSQ: <rssi>,<ber>
    int rssi, ber;
    if (sscanf(response, "+CSQ: %d,%d", &rssi, &ber) == 2) {
      signalQuality = rssi;
      Serial.print("Signal Quality (RSSI): ");
      Serial.println(rssi);
    }
  } else {
    Serial.println("CSQ failed");
  }
}

// Callback for MQTT connect
void onMQTTConnectResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println("MQTT Connected!");
    mqttConnected = true;
  } else {
    Serial.println("MQTT Connect failed");
    mqttConnected = false;
  }
}

// ================== URC HANDLERS ==================

// URC handler for incoming MQTT messages
void onMQTTMessageReceived(const char* prefix, const char* data, void* userData) {
  Serial.print("MQTT Message Received: ");
  Serial.println(data);
  
  // Parse the +SMSUB: format
  // Format: +SMSUB: "topic",<length>,"<payload>"
  // You would parse and process the message here
}

// URC handler for MQTT state changes
void onMQTTStateChanged(const char* prefix, const char* data, void* userData) {
  Serial.print("MQTT State Changed: ");
  Serial.println(data);
  
  // Check if disconnected
  if (strstr(data, "0") != nullptr) {
    Serial.println("MQTT Disconnected!");
    mqttConnected = false;
  }
}

// URC handler for PDP context changes
void onPDPContextChanged(const char* prefix, const char* data, void* userData) {
  Serial.print("PDP Context Changed: ");
  Serial.println(data);
  
  if (strstr(data, "DEACTIVE") != nullptr) {
    Serial.println("PDP Context Deactivated!");
    // You might want to trigger reconnection here
  }
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("ATCommandLib Basic Example");
  Serial.println("==========================");
  
  // Initialize modem serial
  MODEM_SERIAL.begin(115200);
  
  // Create AT manager
  atManager = new ATCommandManager(&MODEM_SERIAL);
  
  // Initialize
  if (!atManager->begin()) {
    Serial.println("ERROR: Failed to initialize AT manager");
    while(1) delay(1000);
  }
  
  // Enable debug output
  atManager->setDebug(true);
  
  // Register URC handlers
  atManager->registerURCHandler("+SMSUB:", onMQTTMessageReceived);
  atManager->registerURCHandler("+SMSTATE:", onMQTTStateChanged);
  atManager->registerURCHandler("+APP PDP:", onPDPContextChanged);
  
  Serial.println("AT Manager initialized");
  Serial.println("URC handlers registered");
  
  delay(1000);
  
  // Example 1: Check if modem is alive (sync)
  Serial.println("\n--- Example 1: Check modem alive (sync) ---");
  if (atManager->isAlive(2000)) {
    Serial.println("Modem is alive!");
  } else {
    Serial.println("Modem not responding");
  }
  
  delay(1000);
  
  // Example 2: Send async command with callback
  Serial.println("\n--- Example 2: Send async AT command ---");
  atManager->sendCommandAsync("AT", 2000, onATResponse);
  
  delay(2000);
  
  // Example 3: Query signal quality (async)
  Serial.println("\n--- Example 3: Query signal quality (async) ---");
  atManager->sendCommandAsync("AT+CSQ", 2000, onCSQResponse);
  
  delay(2000);
  
  // Example 4: Query network registration (sync)
  Serial.println("\n--- Example 4: Query network registration (sync) ---");
  char response[128] = {0};
  if (atManager->sendCommandSync("AT+CREG?", "+CREG:", 2000, response, sizeof(response))) {
    Serial.print("Network registration: ");
    Serial.println(response);
  } else {
    Serial.println("Failed to get network registration");
  }
  
  Serial.println("\n==========================");
  Serial.println("Setup complete. Entering main loop...");
  Serial.println("==========================\n");
}

// ================== LOOP ==================

void loop() {
  // CRITICAL: Must call loop() to process AT commands and URCs
  atManager->loop();
  
  // Your application logic here
  
  // Example: Periodically check signal quality
  static unsigned long lastCSQ = 0;
  if (millis() - lastCSQ > 30000) { // Every 30 seconds
    lastCSQ = millis();
    atManager->sendCommandAsync("AT+CSQ", 2000, onCSQResponse);
  }
  
  // Example: Print stats every 60 seconds
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 60000) {
    lastStats = millis();
    
    const ATStats& stats = atManager->getStats();
    Serial.println("\n--- AT Command Statistics ---");
    Serial.print("Commands Sent: "); Serial.println(stats.commandsSent);
    Serial.print("Commands OK: "); Serial.println(stats.commandsOK);
    Serial.print("Commands ERROR: "); Serial.println(stats.commandsError);
    Serial.print("Commands TIMEOUT: "); Serial.println(stats.commandsTimeout);
    Serial.print("URCs Received: "); Serial.println(stats.urcsReceived);
    Serial.print("Bytes Sent: "); Serial.println(stats.bytesSent);
    Serial.print("Bytes Received: "); Serial.println(stats.bytesReceived);
    Serial.print("Pending Commands: "); Serial.println(atManager->getPendingCommandCount());
    Serial.println("-----------------------------\n");
  }
  
  // Small delay to avoid tight loop
  delay(10);
}
