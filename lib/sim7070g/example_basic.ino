/*
 * SIM7070G Library - Basic Example
 * 
 * Basic example of how to use the SIM7070G asynchronous library
 */

#include <Sim7070G.h>

// Serial port for modem (change according to your board)
HardwareSerial* modemSerial = &Serial1;

// Create instance of SIM7070G
// Parameters: serial, powerPin, baudRate
Sim7070G modem(modemSerial, 12, 19200);

// Network credentials
const char* APN = "your.apn.name";
const char* APN_USER = "";
const char* APN_PASS = "";

// MQTT credentials
const char* MQTT_BROKER = "mqtt.broker.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "arduino_client";
const char* MQTT_USERNAME = "";
const char* MQTT_PASSWORD = "";

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println(F("=== SIM7070G Basic Example ==="));
  
  // Initialize modem
  if (!modem.begin()) {
    Serial.println(F("Failed to initialize modem!"));
    while (1) delay(1000);
  }
  
  Serial.println(F("Modem initialized"));
  
  // Power on modem
  modem.powerOn();
  delay(2000);
  
  // Set network state callback
  modem.setNetworkStateCallback([](Sim7070GState state) {
    Serial.print(F("Network state: "));
    switch (state) {
      case Sim7070GState::IDLE:
        Serial.println(F("IDLE"));
        break;
      case Sim7070GState::INITIALIZING:
        Serial.println(F("INITIALIZING"));
        break;
      case Sim7070GState::CHECKING_SIM:
        Serial.println(F("CHECKING_SIM"));
        break;
      case Sim7070GState::CONFIGURING_NETWORK:
        Serial.println(F("CONFIGURING_NETWORK"));
        break;
      case Sim7070GState::CONNECTING_NETWORK:
        Serial.println(F("CONNECTING_NETWORK"));
        break;
      case Sim7070GState::NETWORK_CONNECTED:
        Serial.println(F("NETWORK_CONNECTED"));
        break;
      case Sim7070GState::NETWORK_DISCONNECTED:
        Serial.println(F("NETWORK_DISCONNECTED"));
        break;
      case Sim7070GState::ERROR:
        Serial.println(F("ERROR"));
        break;
    }
  });
  
  // Set MQTT state callback
  modem.setMQTTStateCallback([](MQTTState state) {
    Serial.print(F("MQTT state: "));
    switch (state) {
      case MQTTState::DISCONNECTED:
        Serial.println(F("DISCONNECTED"));
        break;
      case MQTTState::CONNECTING:
        Serial.println(F("CONNECTING"));
        break;
      case MQTTState::CONNECTED:
        Serial.println(F("CONNECTED"));
        break;
      case MQTTState::DISCONNECTING:
        Serial.println(F("DISCONNECTING"));
        break;
      case MQTTState::ERROR:
        Serial.println(F("ERROR"));
        break;
    }
  });
  
  // Set MQTT message callback
  modem.setMQTTMessageCallback([](const char* topic, const uint8_t* payload, uint32_t len) {
    Serial.print(F("MQTT Message ["));
    Serial.print(topic);
    Serial.print(F("]: "));
    Serial.write(payload, len);
    Serial.println();
  });
  
  // Start connection process
  connectNetwork();
}

void loop() {
  // CRITICAL: Must call loop() frequently to process asynchronous operations
  modem.loop();
  
  // Your code here
  delay(10);
}

void connectNetwork() {
  Serial.println(F("\n=== Connecting to Network ==="));
  
  // Check if modem is alive
  if (!modem.isPoweredOn()) {
    Serial.println(F("Modem not responding, powering on..."));
    modem.powerOn();
    delay(2000);
  }
  
  // Check SIM card
  Serial.println(F("Checking SIM card..."));
  if (!modem.checkSIM()) {
    Serial.println(F("SIM card not ready!"));
    return;
  }
  Serial.println(F("SIM card OK"));
  
  // Set network mode (38 = LTE NB-IoT)
  Serial.println(F("Setting network mode..."));
  if (!modem.setNetworkMode(38)) {
    Serial.println(F("Failed to set network mode"));
    return;
  }
  delay(1000);
  
  // Get signal quality
  int8_t rssi;
  uint8_t ber;
  if (modem.getSignalQuality(&rssi, &ber)) {
    Serial.print(F("Signal quality - RSSI: "));
    Serial.print(rssi);
    Serial.print(F(", BER: "));
    Serial.println(ber);
  }
  
  // Activate PDP context
  Serial.println(F("Activating PDP context..."));
  if (modem.activatePDPContext(0, APN, APN_USER[0] ? APN_USER : nullptr, 
                               APN_PASS[0] ? APN_PASS : nullptr)) {
    Serial.println(F("PDP context activated"));
    
    // Get IP address
    char ip[16];
    if (modem.getIPAddress(ip, sizeof(ip))) {
      Serial.print(F("IP Address: "));
      Serial.println(ip);
    }
    
    // Connect to MQTT
    connectMQTT();
  } else {
    Serial.println(F("Failed to activate PDP context"));
  }
}

void connectMQTT() {
  Serial.println(F("\n=== Connecting to MQTT ==="));
  
  // Begin MQTT
  if (!modem.mqttBegin()) {
    Serial.println(F("Failed to begin MQTT"));
    return;
  }
  
  // Set MQTT configuration
  Serial.println(F("Configuring MQTT..."));
  if (!modem.mqttSetConfig(
    MQTT_CLIENT_ID,
    MQTT_BROKER,
    MQTT_PORT,
    MQTT_USERNAME[0] ? MQTT_USERNAME : nullptr,
    MQTT_PASSWORD[0] ? MQTT_PASSWORD : nullptr,
    60,  // Keepalive
    true // Clean session
  )) {
    Serial.println(F("Failed to configure MQTT"));
    return;
  }
  
  // Connect to MQTT broker
  Serial.println(F("Connecting to MQTT broker..."));
  if (modem.mqttConnect()) {
    Serial.println(F("MQTT connected!"));
    
    // Subscribe to topic
    const char* subscribeTopic = "test/subscribe";
    if (modem.mqttSubscribe(subscribeTopic, 0)) {
      Serial.print(F("Subscribed to: "));
      Serial.println(subscribeTopic);
    }
    
    // Publish a message
    const char* publishTopic = "test/publish";
    const char* message = "Hello from SIM7070G!";
    if (modem.mqttPublish(publishTopic, message, 0, false)) {
      Serial.print(F("Published to: "));
      Serial.print(publishTopic);
      Serial.print(F(" - "));
      Serial.println(message);
    }
  } else {
    Serial.println(F("Failed to connect to MQTT broker"));
  }
}
