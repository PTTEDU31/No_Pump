#ifndef SIM7070G_H
#define SIM7070G_H

#include <Arduino.h>
#include "config.h"

// =========================================================================
// SIM7070G MODEM LIBRARY
// =========================================================================

class SIM7070G {
public:
  // Constructor
  SIM7070G(HardwareSerial& serial, uint8_t pwrPin);
  
  // Initialization
  void begin(unsigned long baudRate = 115200);
  void applyBootConfig();
  
  // Power management
  void restart();
  bool isAlive();
  void verifySIMConnected();
  
  // AT command helpers
  void sendAT(const char* command, unsigned long wait = 100);
  bool sendATwaitOK(const char* cmd, char* out, size_t outCap, unsigned long overallMs = 2500);
  void sendCommandGetResponse(const char* command, char* response, size_t maxLen, unsigned long timeout = 1200);
  void readResponse();
  void flushBuffer();
  
  // GPRS/Network functions
  bool gprsConnect();
  bool gprsIsConnected();
  
  // MQTT functions
  void mqttConnect();
  bool checkMQTTConnection();
  bool publishMessage(const char* topic, const char* payload);
  
  // Time synchronization
  bool syncTimeUTC_viaCNTP(uint8_t cid = 1, const char* server = "time.google.com", uint32_t waitTotalMs = 12000UL);
  bool syncTimeUTC_any(uint8_t cid, uint32_t waitTotalMs = 12000UL);
  bool getNetworkTimeISO8601(char* isoOut, size_t outCap);
  bool parseCCLKToISO(const char* cclkResp, char* isoOut, size_t outCap);
  bool getNetworkHHMMSS(char* hms, size_t cap);
  
  // Signal quality
  int getCSQ_RSSI();
  int csqToDbm(int rssi);
  
  // UART activity tracking
  void noteUartActivity();
  bool canRunHealthCheck();
  
  // Connection state
  bool mqttConnected;
  
  // Counters
  volatile uint32_t cntModemRestarts;
  volatile uint32_t cntGprsConnects;
  volatile uint32_t cntMqttConnects;
  
  // Health tracking
  uint8_t atFailStreak;
  uint8_t gprsFailStreak;
  uint8_t mqttFailStreak;
  
  unsigned long lastATokMs;
  unsigned long lastGPRSokMs;
  unsigned long lastMQTTokMs;
  unsigned long lastRestartMs;
  
private:
  HardwareSerial& _serial;
  uint8_t _pwrPin;
  
  // UART activity tracking
  unsigned long _lastUartActivity;
  static const unsigned long _healthQuietMs = 2000;
  
  // Helper functions
  bool ensureATorPowerCycle(uint8_t atAttempts = 3);
  void safeFlushAfterReset();
};

#endif // SIM7070G_H
