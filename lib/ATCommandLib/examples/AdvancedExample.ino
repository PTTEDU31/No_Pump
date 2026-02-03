/**
 * @file AdvancedExample.ino
 * @brief Advanced example showing MQTT connection flow with AT commands
 * 
 * This example demonstrates:
 * - State machine for GPRS/MQTT connection
 * - Chaining async commands
 * - Error handling and retry logic
 * - URC handling for real MQTT messages
 */

#include <ATCommandLib.h>

#define MODEM_SERIAL Serial1

// MQTT Configuration
const char* MQTT_BROKER = "broker.example.com";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32_client";
const char* MQTT_USERNAME = "username";
const char* MQTT_PASSWORD = "password";
const char* MQTT_TOPIC_SUB = "xtr/server/mynode";
const char* MQTT_TOPIC_PUB = "xtr/nodes/mynode";

// Connection states
enum ConnectionState {
  STATE_INIT,
  STATE_CHECK_AT,
  STATE_CHECK_GPRS,
  STATE_ACTIVATE_GPRS,
  STATE_CHECK_IP,
  STATE_MQTT_CONFIG,
  STATE_MQTT_CONNECT,
  STATE_MQTT_SUBSCRIBE,
  STATE_CONNECTED,
  STATE_ERROR
};

// Global variables
ATCommandManager* atManager = nullptr;
ConnectionState currentState = STATE_INIT;
unsigned long stateStartTime = 0;
uint8_t retryCount = 0;
bool gprsConnected = false;
bool mqttConnected = false;

// ================== HELPER FUNCTIONS ==================

void changeState(ConnectionState newState) {
  if (currentState != newState) {
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print(" -> ");
    Serial.println(newState);
    
    currentState = newState;
    stateStartTime = millis();
    retryCount = 0;
  }
}

// ================== CALLBACK FUNCTIONS ==================

void onATCheckResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println("Modem responding");
    changeState(STATE_CHECK_GPRS);
  } else {
    Serial.println("Modem not responding");
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    }
  }
}

void onGPRSCheckResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    Serial.print("GPRS Status: ");
    Serial.println(response);
    
    // Check if already connected: +CNACT: 0,1,"xxx.xxx.xxx.xxx"
    if (strstr(response, ",1,") != nullptr) {
      Serial.println("GPRS already connected");
      gprsConnected = true;
      changeState(STATE_CHECK_IP);
    } else {
      Serial.println("GPRS not connected, activating...");
      changeState(STATE_ACTIVATE_GPRS);
    }
  } else {
    Serial.println("GPRS check failed");
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    }
  }
}

void onGPRSActivateResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println("GPRS activated");
    gprsConnected = true;
    changeState(STATE_CHECK_IP);
  } else {
    Serial.println("GPRS activation failed");
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    } else {
      changeState(STATE_ACTIVATE_GPRS);
    }
  }
}

void onIPCheckResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK && response) {
    Serial.print("IP Check: ");
    Serial.println(response);
    
    // Parse IP address from response
    if (strstr(response, ".") != nullptr) {
      Serial.println("IP obtained, proceeding to MQTT");
      changeState(STATE_MQTT_CONFIG);
    } else {
      Serial.println("No IP yet");
      retryCount++;
      if (retryCount >= 5) {
        changeState(STATE_ERROR);
      }
    }
  } else {
    Serial.println("IP check failed");
    retryCount++;
    if (retryCount >= 5) {
      changeState(STATE_ERROR);
    }
  }
}

void onMQTTConfigResponse(ATResponseType type, const char* response, void* userData) {
  static uint8_t configStep = 0;
  
  if (type == ATResponseType::OK) {
    configStep++;
    
    // All config steps done (URL, PORT, CLIENT_ID, USERNAME, PASSWORD, etc.)
    if (configStep >= 7) {
      Serial.println("MQTT config complete");
      configStep = 0;
      changeState(STATE_MQTT_CONNECT);
    }
  } else {
    Serial.println("MQTT config failed");
    configStep = 0;
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    } else {
      changeState(STATE_MQTT_CONFIG);
    }
  }
}

void onMQTTConnectResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println("MQTT connected!");
    mqttConnected = true;
    changeState(STATE_MQTT_SUBSCRIBE);
  } else {
    Serial.println("MQTT connect failed");
    mqttConnected = false;
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    } else {
      // Retry connection
      changeState(STATE_MQTT_CONNECT);
    }
  }
}

void onMQTTSubscribeResponse(ATResponseType type, const char* response, void* userData) {
  if (type == ATResponseType::OK) {
    Serial.println("MQTT subscribed!");
    changeState(STATE_CONNECTED);
  } else {
    Serial.println("MQTT subscribe failed");
    retryCount++;
    if (retryCount >= 3) {
      changeState(STATE_ERROR);
    } else {
      changeState(STATE_MQTT_SUBSCRIBE);
    }
  }
}

// ================== URC HANDLERS ==================

void onMQTTMessageReceived(const char* prefix, const char* data, void* userData) {
  Serial.println("\n========== MQTT MESSAGE ==========");
  Serial.print("Prefix: ");
  Serial.println(prefix);
  Serial.print("Data: ");
  Serial.println(data);
  Serial.println("==================================\n");
  
  // Parse +SMSUB: "topic",<length>,"<hex_payload>"
  // Extract and process the hex payload here
  
  // Example: Extract payload between quotes
  const char* payloadStart = strchr(data, '"');
  if (payloadStart) {
    payloadStart = strchr(payloadStart + 1, '"');
    if (payloadStart) {
      payloadStart = strchr(payloadStart + 1, '"');
      if (payloadStart) {
        payloadStart++;
        const char* payloadEnd = strchr(payloadStart, '"');
        if (payloadEnd) {
          size_t payloadLen = payloadEnd - payloadStart;
          char payload[256];
          if (payloadLen < sizeof(payload)) {
            strncpy(payload, payloadStart, payloadLen);
            payload[payloadLen] = '\0';
            
            Serial.print("Hex Payload: ");
            Serial.println(payload);
            
            // Here you would decode hex and decrypt
            // See main_old.cpp handleMQTTMessages() for full implementation
          }
        }
      }
    }
  }
}

void onMQTTStateChanged(const char* prefix, const char* data, void* userData) {
  Serial.print("MQTT State URC: ");
  Serial.println(data);
  
  // +SMSTATE: 0 means disconnected
  if (strstr(data, "0") != nullptr) {
    Serial.println("MQTT Disconnected via URC!");
    mqttConnected = false;
    changeState(STATE_MQTT_CONNECT);
  }
}

void onPDPContextChanged(const char* prefix, const char* data, void* userData) {
  Serial.print("PDP Context URC: ");
  Serial.println(data);
  
  if (strstr(data, "DEACTIVE") != nullptr) {
    Serial.println("PDP Context Deactivated!");
    gprsConnected = false;
    changeState(STATE_ACTIVATE_GPRS);
  }
}

// ================== STATE MACHINE ==================

void runStateMachine() {
  switch (currentState) {
    case STATE_INIT:
      Serial.println("Initializing...");
      changeState(STATE_CHECK_AT);
      break;
      
    case STATE_CHECK_AT:
      Serial.println("Checking AT...");
      atManager->sendCommandAsync("AT", 2000, onATCheckResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
      
    case STATE_CHECK_GPRS:
      Serial.println("Checking GPRS status...");
      atManager->sendCommandAsync("AT+CNACT?", 2000, onGPRSCheckResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
      
    case STATE_ACTIVATE_GPRS:
      Serial.println("Activating GPRS...");
      atManager->sendCommandAsync("AT+CNACT=0,1", 10000, onGPRSActivateResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
      
    case STATE_CHECK_IP:
      Serial.println("Checking IP...");
      atManager->sendCommandAsync("AT+CNACT?", 2000, onIPCheckResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
      
    case STATE_MQTT_CONFIG: {
      Serial.println("Configuring MQTT...");
      
      // Send multiple config commands
      char cmd[128];
      
      atManager->sendCommandAsync("AT+SMDISC", 1000, onMQTTConfigResponse);
      
      snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"URL\",\"%s\"", MQTT_BROKER);
      atManager->sendCommandAsync(cmd, 1000, onMQTTConfigResponse);
      
      snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PORT\",%d", MQTT_PORT);
      atManager->sendCommandAsync(cmd, 1000, onMQTTConfigResponse);
      
      snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLIENTID\",\"%s\"", MQTT_CLIENT_ID);
      atManager->sendCommandAsync(cmd, 1000, onMQTTConfigResponse);
      
      snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"USERNAME\",\"%s\"", MQTT_USERNAME);
      atManager->sendCommandAsync(cmd, 1000, onMQTTConfigResponse);
      
      snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PASSWORD\",\"%s\"", MQTT_PASSWORD);
      atManager->sendCommandAsync(cmd, 1000, onMQTTConfigResponse);
      
      atManager->sendCommandAsync("AT+SMCONF=\"CLEANSS\",1", 1000, onMQTTConfigResponse);
      
      changeState(STATE_INIT); // Wait for callbacks
      break;
    }
      
    case STATE_MQTT_CONNECT:
      Serial.println("Connecting to MQTT...");
      atManager->sendCommandAsync("AT+SMCONN", 30000, onMQTTConnectResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
      
    case STATE_MQTT_SUBSCRIBE: {
      Serial.println("Subscribing to MQTT topic...");
      char cmd[128];
      snprintf(cmd, sizeof(cmd), "AT+SMSUB=\"%s\",1", MQTT_TOPIC_SUB);
      atManager->sendCommandAsync(cmd, 5000, onMQTTSubscribeResponse);
      changeState(STATE_INIT); // Wait for callback
      break;
    }
      
    case STATE_CONNECTED:
      // Normal operation - handled in main loop
      break;
      
    case STATE_ERROR:
      Serial.println("ERROR state - restarting in 15s...");
      delay(15000);
      changeState(STATE_CHECK_AT);
      break;
      
    default:
      break;
  }
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n\n");
  Serial.println("=================================");
  Serial.println("ATCommandLib Advanced Example");
  Serial.println("GPRS/MQTT Connection Flow");
  Serial.println("=================================\n");
  
  // Initialize modem serial
  MODEM_SERIAL.begin(115200);
  delay(100);
  
  // Create AT manager
  atManager = new ATCommandManager(&MODEM_SERIAL);
  
  if (!atManager->begin()) {
    Serial.println("ERROR: Failed to initialize AT manager");
    while(1) delay(1000);
  }
  
  // Enable debug
  atManager->setDebug(true);
  
  // Register URC handlers
  atManager->registerURCHandler("+SMSUB:", onMQTTMessageReceived);
  atManager->registerURCHandler("+SMSTATE:", onMQTTStateChanged);
  atManager->registerURCHandler("+APP PDP:", onPDPContextChanged);
  
  Serial.println("AT Manager initialized");
  Serial.println("Starting connection flow...\n");
  
  changeState(STATE_INIT);
}

// ================== LOOP ==================

void loop() {
  // CRITICAL: Process AT commands and URCs
  atManager->loop();
  
  // Run state machine (non-blocking)
  static unsigned long lastStateCheck = 0;
  if (millis() - lastStateCheck > 100) {
    lastStateCheck = millis();
    runStateMachine();
  }
  
  // When connected, handle normal operations
  if (currentState == STATE_CONNECTED && mqttConnected) {
    // Example: Publish telemetry every 30 seconds
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 30000) {
      lastPublish = millis();
      
      // Build and publish message
      char msg[256];
      snprintf(msg, sizeof(msg), "{\"temp\":25.5,\"uptime\":%lu}", millis() / 1000);
      
      Serial.print("Publishing: ");
      Serial.println(msg);
      
      char cmd[512];
      snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"%s\",%d,1,0", MQTT_TOPIC_PUB, strlen(msg));
      
      // Send publish command then payload
      if (atManager->sendCommandSync(cmd, ">", 5000)) {
        atManager->sendCommandSync(msg, "OK", 5000);
      }
    }
  }
  
  // Print stats every 60 seconds
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 60000) {
    lastStats = millis();
    
    const ATStats& stats = atManager->getStats();
    Serial.println("\n========== STATISTICS ==========");
    Serial.print("Commands: ");
    Serial.print(stats.commandsSent);
    Serial.print(" (OK: ");
    Serial.print(stats.commandsOK);
    Serial.print(", ERR: ");
    Serial.print(stats.commandsError);
    Serial.print(", TO: ");
    Serial.print(stats.commandsTimeout);
    Serial.println(")");
    Serial.print("URCs: ");
    Serial.println(stats.urcsReceived);
    Serial.print("Pending: ");
    Serial.println(atManager->getPendingCommandCount());
    Serial.println("================================\n");
  }
  
  delay(10);
}
