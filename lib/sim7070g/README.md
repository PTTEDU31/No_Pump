# SIM7070G Asynchronous Library

Asynchronous (async) library for SIM7070G module on Arduino, fully replacing TinyGSM.

## Features

- ✅ **Fully asynchronous**: Non-blocking, event-driven
- ✅ **AT Command Engine**: Queue-based command system with timeout and callbacks
- ✅ **Network Management**: GPRS/NB-IoT connection management
- ✅ **MQTT Client**: Async MQTT publish/subscribe
- ✅ **HTTP Client**: Async HTTP(S) requests
- ✅ **TCP/UDP Socket**: Async socket operations
- ✅ **URC Support**: Unsolicited Result Code handling
- ✅ **State Machine**: Event-driven state management with callbacks

## Structure

```
lib/sim7070g/
├── Sim7070G_AT.h/cpp    # Base AT Command Engine
├── Sim7070G.h/cpp        # Main Library Class
└── README.md
```

## Basic usage

### 1. Initialization

```cpp
#include <Sim7070G.h>

HardwareSerial* modemSerial = &Serial1; // or Serial1, Serial2, etc.
Sim7070G modem(modemSerial, 12, 19200); // serial, powerPin, baudRate

void setup() {
  Serial.begin(115200);
  
  if (!modem.begin()) {
    Serial.println("Failed to initialize modem");
    return;
  }
  
  modem.powerOn();
  delay(2000);
}
```

### 2. Main Loop

```cpp
void loop() {
  modem.loop(); // Must be called regularly (non-blocking)
  
  // Your code here
  delay(10);
}
```

### 3. Network Connection

```cpp
// Check SIM
if (!modem.checkSIM()) {
  Serial.println("SIM not ready");
  return;
}

// Set network mode (38 = LTE NB-IoT)
modem.setNetworkMode(38);

// Activate PDP context
if (modem.activatePDPContext(0, "your.apn.name")) {
  Serial.println("Network connected!");
  
  char ip[16];
  if (modem.getIPAddress(ip, sizeof(ip))) {
    Serial.print("IP: ");
    Serial.println(ip);
  }
}
```

### 4. MQTT Client

```cpp
// Setup MQTT
modem.mqttBegin();
modem.mqttSetConfig(
  "client_id",           // Client ID
  "mqtt.broker.com",     // Server
  1883,                  // Port
  "username",            // Username (optional)
  "password",            // Password (optional)
  60,                    // Keepalive
  true                   // Clean session
);

// Connect
if (modem.mqttConnect()) {
  Serial.println("MQTT connected");
  
  // Subscribe
  modem.mqttSubscribe("topic/subscribe", 0);
  
  // Publish
  modem.mqttPublish("topic/publish", "Hello World", 0, false);
}

// Set callback for incoming messages
modem.setMQTTMessageCallback([](const char* topic, const uint8_t* payload, uint32_t len) {
  Serial.print("MQTT RX [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();
});
```

### 5. HTTP Client

```cpp
// Setup HTTP
modem.httpBegin();
modem.httpSetURL("https://api.example.com/data");
modem.httpAddHeader("Content-Type: application/json");

// GET request
modem.httpGET([](int statusCode, const char* response, size_t len, void* userData) {
  Serial.print("HTTP Status: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
}, nullptr);

// POST request
modem.httpSetBody("{\"key\":\"value\"}");
modem.httpPOST([](int statusCode, const char* response, size_t len, void* userData) {
  // Handle response
}, nullptr);
```

### 6. TCP/UDP Socket

```cpp
// Open TCP socket
int socketId = 0;
if (modem.socketOpen(socketId, "example.com", 80, true)) {
  Serial.println("Socket opened");
  
  // Send data
  const char* data = "GET / HTTP/1.1\r\nHost: example.com\r\n\r\n";
  modem.socketSend(socketId, (const uint8_t*)data, strlen(data));
  
  // Receive data
  uint8_t buffer[256];
  size_t bytesRead;
  if (modem.socketReceive(socketId, buffer, sizeof(buffer), &bytesRead)) {
    Serial.write(buffer, bytesRead);
  }
  
  // Close socket
  modem.socketClose(socketId);
}
```

## Callbacks

### Network State Callback

```cpp
modem.setNetworkStateCallback([](Sim7070GState state) {
  switch (state) {
    case Sim7070GState::NETWORK_CONNECTED:
      Serial.println("Network connected");
      break;
    case Sim7070GState::NETWORK_DISCONNECTED:
      Serial.println("Network disconnected");
      break;
    // ...
  }
});
```

### MQTT State Callback

```cpp
modem.setMQTTStateCallback([](MQTTState state) {
  switch (state) {
    case MQTTState::CONNECTED:
      Serial.println("MQTT connected");
      break;
    case MQTTState::DISCONNECTED:
      Serial.println("MQTT disconnected");
      break;
    // ...
  }
});
```

## Supported AT Commands

### Basic Commands
- `AT` - Test command
- `AT+CPIN?` - Check SIM card
- `AT+CNMP=38` - Choose LTE mode
- `AT+CMNB=2` - Choose NB network
- `AT+CSQ` - Check signal intensity
- `AT+CGREG?` - Check network status
- `AT+CGNAPN` - Query APN
- `AT+CPSI?` - Request UE system information
- `AT+CNACT=0,1` - Activate application network
- `AT+CNACT?` - Check network IP

### MQTT Commands
- `AT+SMCONF` - Set MQTT configuration
- `AT+CSSLCFG` - SSL configuration
- `AT+SMSSL` - Select SSL setting
- `AT+SMCONN` - MQTT connection
- `AT+SMPUB` - Send data packet
- `AT+SMSUB` - Subscribe to topics
- `AT+SMUNSUB` - Unsubscribe
- `AT+SMSTATE` - Request MQTT connection status
- `AT+SMPUBHEX` - Set data format to hexadecimal
- `AT+SMDISC` - Disconnecting MQTT

### HTTP Commands
- `AT+SHCONF` - Setting HTTP(S)
- `AT+SHSSL` - Select SSL setting
- `AT+SHBOD` - Set Body
- `AT+SHAHEAD` - Add Head
- `AT+SHPARA` - Set HTTP(S) Para
- `AT+SHCPARA` - Clear HTTP(S) Para
- `AT+SHCHEAD` - Clear Head
- `AT+SHSTATE` - Query HTTP(S) connecting status
- `AT+SHREQ` - Set respond type
- `AT+SHREAD` - Read respond data
- `AT+SHDISC` - Disconnect HTTP(S)

### TCP/UDP Commands
- `AT+CACID` - Set TCP/UDP index
- `AT+CASSLCFG` - Set SSL parameters
- `AT+CAOPEN` - Open a TCP/UDP connection
- `AT+CASEND` - Send data
- `AT+CARECV` - Receive data
- `AT+CACLOSE` - Close TCP/UDP connection
- `AT+CACFG` - Set transparent parameters
- `AT+CASWITCH` - Enable transparent mode

## Notes

1. **Call `loop()` regularly**: This function processes all asynchronous operations
2. **Non-blocking**: All functions are non-blocking; use callbacks to receive results
3. **Memory**: Library uses dynamic allocation for RX buffer; ensure sufficient RAM
4. **Timeout**: Each AT command has its own timeout, adjustable as needed

## Migration from TinyGSM

Replace:
```cpp
TinyGsm modem(Serial1);
```

With:
```cpp
Sim7070G modem(&Serial1, 12, 19200);
```

And replace blocking calls with async calls using callbacks.

## License

MIT License
