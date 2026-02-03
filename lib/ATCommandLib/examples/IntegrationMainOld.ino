/**
 * @file IntegrationMainOld.ino
 * @brief Example showing how to integrate ATCommandLib into main_old.cpp
 * 
 * This demonstrates replacing existing blocking AT commands with
 * non-blocking async commands and URC handlers.
 */

// Include config.h first to configure the library
#include "config.h"
#include <ATCommandLib.h>

// ==================== CONFIGURATION ====================
// Most configuration is now in config.h

const char* NODE_ID = "5a06bafb-e479-4dc3-87d9-d79734d71f13";
const char* mqtt_server = "broker.remotextr.com";
const int mqtt_port = 1883;

char topic_sub[96];
char topic_pub[96];
char clientID[64];
char mqtt_username[64];
char mqtt_password[64];

// ==================== GLOBAL VARIABLES ====================

ATCommandManager* atManager = nullptr;

// State flags
bool mqttConnected = false;
bool gprsConnected = false;
int signalQuality = -1;

// Statistics
uint32_t cntModemRestarts = 0;
uint32_t cntGprsConnects = 0;
uint32_t cntMqttConnects = 0;

// ==================== CALLBACK FUNCTIONS ====================

// Callback for AT alive check
void onATAliveResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println(F("[AT] Modem alive"));
  } else {
    Serial.println(F("[AT] Modem not responding"));
  }
}

// Callback for CSQ (signal quality)
void onCSQResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    int rssi, ber;
    if (sscanf(response, "+CSQ: %d,%d", &rssi, &ber) == 2) {
      signalQuality = rssi;
      Serial.print(F("[CSQ] Signal Quality: "));
      Serial.print(rssi);
      Serial.print(F(" ("));
      
      // Convert to dBm
      if (rssi >= 2 && rssi <= 30) {
        int dbm = -113 + (rssi * 2);
        Serial.print(dbm);
        Serial.print(F(" dBm)"));
      } else {
        Serial.print(F("unknown)"));
      }
      Serial.println();
    }
  } else {
    Serial.println(F("[CSQ] Query failed"));
    signalQuality = -1;
  }
}

// Callback for GPRS status check
void onGPRSCheckResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    Serial.print(F("[GPRS] Status: "));
    Serial.println(response);
    
    // Check if connected: +CNACT: 0,1,"xxx.xxx.xxx.xxx"
    if (strstr(response, ",1,") != nullptr) {
      gprsConnected = true;
      Serial.println(F("[GPRS] Connected"));
    } else {
      gprsConnected = false;
      Serial.println(F("[GPRS] Not connected"));
    }
  } else {
    gprsConnected = false;
    Serial.println(F("[GPRS] Check failed"));
  }
}

// Callback for GPRS activation
void onGPRSActivateResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    gprsConnected = true;
    cntGprsConnects++;
    Serial.println(F("[GPRS] Activated successfully"));
    
    // After GPRS connect, sync time (optional)
    // syncTimeUTC();
  } else if (type == ATResponseType::ERROR) {
    Serial.println(F("[GPRS] Activation ERROR"));
  } else {
    Serial.println(F("[GPRS] Activation TIMEOUT"));
  }
}

// Callback for MQTT state check
void onMQTTStateCheckResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    // Check +SMSTATE: 1 (connected) or 0 (disconnected)
    mqttConnected = (strstr(response, "+SMSTATE: 1") != nullptr);
    
    Serial.print(F("[MQTT] State: "));
    Serial.println(mqttConnected ? F("Connected") : F("Disconnected"));
  } else {
    mqttConnected = false;
  }
}

// Callback for MQTT connect
void onMQTTConnectResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println(F("[MQTT] Connected to broker!"));
    mqttConnected = true;
    cntMqttConnects++;
    
    // Subscribe to topic
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+SMSUB=\"%s\",1", topic_sub);
    atManager->sendCommandAsync(cmd, 5000, onMQTTSubscribeResponse);
    
  } else if (type == ATResponseType::ERROR) {
    Serial.println(F("[MQTT] Connect ERROR"));
    mqttConnected = false;
  } else {
    Serial.println(F("[MQTT] Connect TIMEOUT"));
    mqttConnected = false;
  }
}

// Callback for MQTT subscribe
void onMQTTSubscribeResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.print(F("[MQTT] Subscribed to: "));
    Serial.println(topic_sub);
  } else {
    Serial.println(F("[MQTT] Subscribe failed"));
  }
}

// ==================== URC HANDLERS ====================

// URC handler for incoming MQTT messages
// This replaces the handleMQTTMessages() polling logic
void onMQTTMessageReceived(const char* prefix, const char* data, void* userData) {
  Serial.println(F("\n========== MQTT MESSAGE RECEIVED =========="));
  Serial.print(F("Raw data: "));
  Serial.println(data);
  
  // Parse +SMSUB: "topic",<length>,"<hex_payload>"
  const char* payload = strchr(data, '"');
  if (payload) {
    payload = strchr(payload + 1, '"');
    if (payload) {
      payload = strchr(payload + 1, '"');
      if (payload) {
        payload++; // Skip opening quote
        
        const char* end = strchr(payload, '"');
        if (end) {
          size_t payloadLen = end - payload;
          char hexPayload[600];
          
          if (payloadLen < sizeof(hexPayload)) {
            strncpy(hexPayload, payload, payloadLen);
            hexPayload[payloadLen] = '\0';
            
            Serial.print(F("Hex payload (len="));
            Serial.print(strlen(hexPayload));
            Serial.println(F("):"));
            Serial.println(hexPayload);
            
            // HERE: Integrate your existing message processing logic
            // from handleMQTTMessages() in main_old.cpp:
            // 1. hexDecode() - convert hex to binary
            // 2. Extract nonce, tag, ciphertext
            // 3. Time validation
            // 4. decryptPayload()
            // 5. Process JSON command
            
            // For now, just print
            Serial.println(F("TODO: Decrypt and process command"));
          }
        }
      }
    }
  }
  
  Serial.println(F("==========================================\n"));
}

// URC handler for MQTT state changes
void onMQTTStateChanged(const char* prefix, const char* data, void* userData) {
  Serial.print(F("[URC] MQTT State: "));
  Serial.println(data);
  
  // +SMSTATE: 0 means disconnected
  if (strstr(data, "0") != nullptr) {
    Serial.println(F("[URC] MQTT DISCONNECTED!"));
    mqttConnected = false;
    
    // Trigger reconnection
    // You might want to set a flag here instead of direct reconnect
  } else if (strstr(data, "1") != nullptr) {
    Serial.println(F("[URC] MQTT CONNECTED!"));
    mqttConnected = true;
  }
}

// URC handler for PDP context changes
void onPDPContextChanged(const char* prefix, const char* data, void* userData) {
  Serial.print(F("[URC] PDP Context: "));
  Serial.println(data);
  
  // +APP PDP: 0,DEACTIVE means PDP deactivated
  if (strstr(data, "DEACTIVE") != nullptr) {
    Serial.println(F("[URC] PDP CONTEXT DEACTIVATED!"));
    gprsConnected = false;
    mqttConnected = false;
    
    // Trigger reconnection
  }
}

// ==================== HELPER FUNCTIONS ====================

// Apply boot configuration (non-blocking)
void applyBootConfig() {
  Serial.println(F("[CONFIG] Applying boot config..."));
  
  // Queue multiple commands
  atManager->sendCommandAsync("AT+CMEE=2", 500);
  atManager->sendCommandAsync("AT+CSOCKSETPN=1", 500);
  atManager->sendCommandAsync("AT+SMCONF=\"CONTEXTID\",1", 500);
  atManager->sendCommandAsync("AT+CSCLK=0", 500);
  
  Serial.println(F("[CONFIG] Boot config queued"));
}

// Connect to GPRS (non-blocking)
void gprsConnect() {
  Serial.println(F("[GPRS] Initiating connection..."));
  
  // First, check current status
  atManager->sendCommandAsync("AT+CNACT?", 2000, onGPRSCheckResponse);
  
  // If not connected (will be handled in callback), activate
  // For now, let's queue the activation command
  // atManager->sendCommandAsync("AT+CNACT=0,1", 10000, onGPRSActivateResponse);
}

// Connect to MQTT (non-blocking)
void mqttConnect() {
  Serial.println(F("[MQTT] Initiating connection..."));
  
  // First disconnect if needed
  atManager->sendCommandAsync("AT+SMDISC", 1000);
  
  // Configure MQTT
  char cmd[128];
  
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"URL\",\"%s\"", mqtt_server);
  atManager->sendCommandAsync(cmd, 1000);
  
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PORT\",%d", mqtt_port);
  atManager->sendCommandAsync(cmd, 1000);
  
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLIENTID\",\"%s\"", clientID);
  atManager->sendCommandAsync(cmd, 1000);
  
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"USERNAME\",\"%s\"", mqtt_username);
  atManager->sendCommandAsync(cmd, 1000);
  
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PASSWORD\",\"%s\"", mqtt_password);
  atManager->sendCommandAsync(cmd, 1000);
  
  atManager->sendCommandAsync("AT+SMCONF=\"CLEANSS\",1", 1000);
  atManager->sendCommandAsync("AT+SMCONF=\"QOS\",0", 1000);
  atManager->sendCommandAsync("AT+SMCONF=\"KEEPTIME\",60", 1000);
  
  // Connect (with callback)
  atManager->sendCommandAsync("AT+SMCONN", 30000, onMQTTConnectResponse);
}

// Check connections (replaces old blocking checkConnections)
void checkConnections() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 15000; // 15 seconds
  
  if ((millis() - lastCheck) < checkInterval) {
    return;
  }
  lastCheck = millis();
  
  Serial.println(F("\n--- Health Check ---"));
  
  // Check modem alive
  atManager->sendCommandAsync("AT", 1000, onATAliveResponse);
  
  // Check GPRS status
  atManager->sendCommandAsync("AT+CNACT?", 2000, onGPRSCheckResponse);
  
  // Check MQTT status
  atManager->sendCommandAsync("AT+SMSTATE?", 1500, onMQTTStateCheckResponse);
  
  // Query signal quality
  atManager->sendCommandAsync("AT+CSQ", 1500, onCSQResponse);
  
  Serial.println(F("--------------------\n"));
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println(F("\n\n"));
  Serial.println(F("==========================================="));
  Serial.println(F("  ATCommandLib Integration Example"));
  Serial.println(F("  For main_old.cpp"));
  Serial.println(F("===========================================\n"));
  
  // Initialize modem serial
  sim7070.begin(115200);
  delay(100);
  
  // Setup MQTT topics
  snprintf(topic_sub, sizeof(topic_sub), "xtr/server/%s", NODE_ID);
  snprintf(topic_pub, sizeof(topic_pub), "xtr/nodes/%s", NODE_ID);
  snprintf(clientID, sizeof(clientID), "%s", NODE_ID);
  snprintf(mqtt_username, sizeof(mqtt_username), "node_username");
  snprintf(mqtt_password, sizeof(mqtt_password), "node_password");
  
  Serial.print(F("Node ID: "));
  Serial.println(NODE_ID);
  Serial.print(F("Subscribe Topic: "));
  Serial.println(topic_sub);
  Serial.print(F("Publish Topic: "));
  Serial.println(topic_pub);
  Serial.println();
  
  // Create AT manager
  atManager = new ATCommandManager(&sim7070);
  
  if (!atManager->begin()) {
    Serial.println(F("ERROR: Failed to initialize AT manager"));
    while(1) {
      delay(1000);
    }
  }
  
  // Enable debug mode
  atManager->setDebug(true);
  
  // Register URC handlers
  Serial.println(F("Registering URC handlers..."));
  atManager->registerURCHandler("+SMSUB:", onMQTTMessageReceived);
  atManager->registerURCHandler("+SMSTATE:", onMQTTStateChanged);
  atManager->registerURCHandler("+APP PDP:", onPDPContextChanged);
  
  Serial.println(F("AT Manager initialized"));
  Serial.println(F("URC handlers registered\n"));
  
  delay(1000);
  
  // Apply boot config
  applyBootConfig();
  
  delay(2000);
  
  // Connect to GPRS
  gprsConnect();
  
  delay(5000);
  
  // Connect to MQTT
  mqttConnect();
  
  Serial.println(F("\n==========================================="));
  Serial.println(F("  Setup complete. Entering main loop..."));
  Serial.println(F("===========================================\n"));
}

// ==================== LOOP ====================

void loop() {
  // CRITICAL: Process AT commands and URCs
  atManager->loop();
  
  // Periodic health check
  checkConnections();
  
  // Print statistics every 60 seconds
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 60000) {
    lastStats = millis();
    
    const ATStats& stats = atManager->getStats();
    
    Serial.println(F("\n============ STATISTICS ============"));
    Serial.print(F("Commands Sent: "));
    Serial.println(stats.commandsSent);
    Serial.print(F("  - OK: "));
    Serial.println(stats.commandsOK);
    Serial.print(F("  - ERROR: "));
    Serial.println(stats.commandsError);
    Serial.print(F("  - TIMEOUT: "));
    Serial.println(stats.commandsTimeout);
    Serial.print(F("URCs Received: "));
    Serial.println(stats.urcsReceived);
    Serial.print(F("Pending Commands: "));
    Serial.println(atManager->getPendingCommandCount());
    Serial.println();
    Serial.print(F("Modem Restarts: "));
    Serial.println(cntModemRestarts);
    Serial.print(F("GPRS Connects: "));
    Serial.println(cntGprsConnects);
    Serial.print(F("MQTT Connects: "));
    Serial.println(cntMqttConnects);
    Serial.println();
    Serial.print(F("GPRS Connected: "));
    Serial.println(gprsConnected ? F("YES") : F("NO"));
    Serial.print(F("MQTT Connected: "));
    Serial.println(mqttConnected ? F("YES") : F("NO"));
    Serial.print(F("Signal Quality: "));
    Serial.println(signalQuality);
    Serial.println(F("====================================\n"));
  }
  
  // When MQTT connected, you can publish messages here
  if (mqttConnected) {
    // Example: Publish telemetry every 30 seconds
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 30000) {
      lastPublish = millis();
      
      Serial.println(F("[PUBLISH] Sending telemetry..."));
      
      // Build your message (same as publishMessage() in main_old.cpp)
      // For now, just a simple example
      char msg[256];
      snprintf(msg, sizeof(msg), "{\"uptime\":%lu,\"csq\":%d}", 
               millis() / 1000, signalQuality);
      
      // Publish (you'll need to adapt this for hex encoding if using encryption)
      char cmd[512];
      snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"%s\",%d,1,0", topic_pub, strlen(msg));
      
      if (atManager->sendCommandSync(cmd, ">", 5000)) {
        atManager->sendCommandSync(msg, "OK", 5000);
        Serial.println(F("[PUBLISH] Sent"));
      } else {
        Serial.println(F("[PUBLISH] Failed"));
      }
    }
  }
  
  // Small delay
  delay(10);
}

// ==================== NOTES ====================
/*
 * INTEGRATION STEPS FOR main_old.cpp:
 * 
 * 1. Replace sendAT() calls with atManager->sendCommandAsync()
 * 2. Replace sendCommandGetResponse() with callbacks
 * 3. Remove handleMQTTMessages() polling - use URC handlers instead
 * 4. Convert gprsConnect(), mqttConnect() to non-blocking versions
 * 5. Replace checkConnections() blocking checks with async queries
 * 6. Add atManager->loop() at top of main loop()
 * 7. Keep your existing crypto and pump control logic
 * 
 * BENEFITS:
 * - No more blocking delays
 * - URCs handled automatically (no missed messages)
 * - Better error handling with callbacks
 * - Easier to debug with statistics
 * - Command queue prevents overwhelming the modem
 */
