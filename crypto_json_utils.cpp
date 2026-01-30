#include "crypto_json_utils.h"
#include "DEBUG.h"
#include "Sim7070G.h"
#include <string.h>

// External crypto key (defined in main.cpp)
extern const char* CRYPTOKEY;

// Static crypto key buffer (decoded from hex)
static uint8_t CRYPTO_KEY[32];
static bool cryptoKeyInitialized = false;

// =========================================================================
// HEX ENCODING/DECODING
// =========================================================================

void hexEncode(const uint8_t* in, size_t inLen, char* out, size_t outCap, bool uppercase) {
  static const char* TAB_U = "0123456789ABCDEF";
  static const char* TAB_L = "0123456789abcdef";
  const char* TAB = uppercase ? TAB_U : TAB_L;

  size_t need = inLen * 2 + 1;
  if (outCap < need) {
    if (outCap) out[0] = '\0';
    return;
  }

  size_t o = 0;
  for (size_t i = 0; i < inLen; i++) {
    out[o++] = TAB[(in[i] >> 4) & 0x0F];
    out[o++] = TAB[in[i] & 0x0F];
  }
  out[o] = '\0';
}

bool hexDecode(const char* hex, uint8_t* out, size_t outCap, size_t& outLen) {
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  
  size_t n = strlen(hex);
  if (n % 2) return false;
  size_t bytes = n / 2;
  if (bytes > outCap) return false;
  
  for (size_t i = 0; i < bytes; i++) {
    int hi = nib(hex[2 * i]);
    int lo = nib(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  outLen = bytes;
  return true;
}

// =========================================================================
// JSON PARSING HELPERS
// =========================================================================

bool jsonGetString(const char* json, const char* key, char* out, size_t cap) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  p++;
  const char* e = strchr(p, '"');
  if (!e) return false;
  size_t n = (size_t)(e - p);
  if (n >= cap) n = cap - 1;
  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}

bool jsonGetBool(const char* json, const char* key, bool& out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (!strncmp(p, "true", 4)) {
    out = true;
    return true;
  }
  if (!strncmp(p, "false", 5)) {
    out = false;
    return true;
  }
  return false;
}

bool jsonGetInt(const char* json, const char* key, long& out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  out = strtol(p, NULL, 10);
  return true;
}

bool jsonGetFloat(const char* json, const char* key, float& out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  out = strtof(p, NULL);
  return true;
}

// =========================================================================
// CRYPTO FUNCTIONS
// =========================================================================

static void initCryptoKey() {
  if (cryptoKeyInitialized) return;
  
  if (!CRYPTOKEY) {
    DEBUG_PRINTLN(F("[CRYPTO] ERROR: CRYPTOKEY not defined"));
    return;
  }
  
  size_t outLen = 0;
  if (!hexDecode(CRYPTOKEY, CRYPTO_KEY, sizeof(CRYPTO_KEY), outLen) || outLen != 32) {
    DEBUG_PRINTLN(F("[CRYPTO] ERROR: INVALID CRYPTO KEY (must be 64 hex = 32 bytes)."));
    return;
  }
  
  cryptoKeyInitialized = true;
  DEBUG_PRINTLN(F("[CRYPTO] Key loaded from HEX."));
}

void makeNonce12(uint8_t nonce[12]) {
  static uint64_t perSecondCtr = 0;  // 40-bit used
  static uint16_t lastY = 0;
  static uint8_t lastM = 0;
  static uint8_t lastD = 0;
  static uint8_t lastH = 0;
  static uint8_t lastMin = 0;
  static uint8_t lastS = 0;

  int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0;
  char iso[32] = { 0 };

  // Try to get network time from modem (requires Sim7070G instance)
  // For now, we'll use a fallback approach - this will be called from Sim7070GDevice
  // which has access to the modem instance
  bool haveNetwork = false;
  
  // Note: This function should be called with modem instance available
  // We'll need to pass modem instance or use a global/static reference
  // For now, use millis() as fallback
  uint32_t t = millis() / 1000UL;
  H = (t / 3600UL) % 24;
  Mi = (t / 60UL) % 60;
  S = (t) % 60;
  Y = 0xFFFF;
  Mo = 0;
  D = 0;

  if ((uint16_t)Y != lastY || (uint8_t)Mo != lastM || (uint8_t)D != lastD || 
      (uint8_t)H != lastH || (uint8_t)Mi != lastMin || (uint8_t)S != lastS) {
    perSecondCtr = 0;
    lastY = (uint16_t)Y;
    lastM = (uint8_t)Mo;
    lastD = (uint8_t)D;
    lastH = (uint8_t)H;
    lastMin = (uint8_t)Mi;
    lastS = (uint8_t)S;
  }

  nonce[0] = (uint8_t)((((uint16_t)Y) >> 8) & 0xFF);
  nonce[1] = (uint8_t)(((uint16_t)Y) & 0xFF);
  nonce[2] = (uint8_t)Mo;
  nonce[3] = (uint8_t)D;
  nonce[4] = (uint8_t)H;
  nonce[5] = (uint8_t)Mi;
  nonce[6] = (uint8_t)S;

  uint64_t c = perSecondCtr++;
  c &= 0x000000FFFFFFFFFFULL;
  nonce[7] = (uint8_t)((c >> 32) & 0xFF);
  nonce[8] = (uint8_t)((c >> 24) & 0xFF);
  nonce[9] = (uint8_t)((c >> 16) & 0xFF);
  nonce[10] = (uint8_t)((c >> 8) & 0xFF);
  nonce[11] = (uint8_t)(c & 0xFF);
}

// Overloaded version that uses modem instance to get network time
void makeNonce12(uint8_t nonce[12], Sim7070G* modem) {
  static uint64_t perSecondCtr = 0;
  static uint16_t lastY = 0;
  static uint8_t lastM = 0;
  static uint8_t lastD = 0;
  static uint8_t lastH = 0;
  static uint8_t lastMin = 0;
  static uint8_t lastS = 0;

  int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0;
  char iso[32] = { 0 };

  bool haveNetwork = false;
  if (modem && modem->getNetworkTimeISO8601(iso, sizeof(iso))) {
    if (sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d", &Y, &Mo, &D, &H, &Mi, &S) == 6) {
      haveNetwork = true;
    }
  }

  if (!haveNetwork) {
    uint32_t t = millis() / 1000UL;
    H = (t / 3600UL) % 24;
    Mi = (t / 60UL) % 60;
    S = (t) % 60;
    Y = 0xFFFF;
    Mo = 0;
    D = 0;
  }

  if ((uint16_t)Y != lastY || (uint8_t)Mo != lastM || (uint8_t)D != lastD || 
      (uint8_t)H != lastH || (uint8_t)Mi != lastMin || (uint8_t)S != lastS) {
    perSecondCtr = 0;
    lastY = (uint16_t)Y;
    lastM = (uint8_t)Mo;
    lastD = (uint8_t)D;
    lastH = (uint8_t)H;
    lastMin = (uint8_t)Mi;
    lastS = (uint8_t)S;
  }

  nonce[0] = (uint8_t)((((uint16_t)Y) >> 8) & 0xFF);
  nonce[1] = (uint8_t)(((uint16_t)Y) & 0xFF);
  nonce[2] = (uint8_t)Mo;
  nonce[3] = (uint8_t)D;
  nonce[4] = (uint8_t)H;
  nonce[5] = (uint8_t)Mi;
  nonce[6] = (uint8_t)S;

  uint64_t c = perSecondCtr++;
  c &= 0x000000FFFFFFFFFFULL;
  nonce[7] = (uint8_t)((c >> 32) & 0xFF);
  nonce[8] = (uint8_t)((c >> 24) & 0xFF);
  nonce[9] = (uint8_t)((c >> 16) & 0xFF);
  nonce[10] = (uint8_t)((c >> 8) & 0xFF);
  nonce[11] = (uint8_t)(c & 0xFF);
}

size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], const uint8_t nonce12[12]) {
  initCryptoKey();
  if (!cryptoKeyInitialized) return 0;
  
  size_t len = strlen(plaintext);
  ChaChaPoly aead;
  aead.setKey(CRYPTO_KEY, sizeof(CRYPTO_KEY));
  aead.setIV(nonce12, 12);
  aead.encrypt(ciphertext, (const uint8_t*)plaintext, len);
  aead.computeTag(tag, 16);
  return len;
}

size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], Sim7070G* modem) {
  uint8_t nonce[12];
  makeNonce12(nonce, modem);
  return encryptPayload(plaintext, ciphertext, tag, nonce);
}

bool decryptPayload(const uint8_t* ciphertext, size_t ctLen, const uint8_t tag[16],
                    const uint8_t nonce12[12], char* outPlain, size_t outPlainCap) {
  initCryptoKey();
  if (!cryptoKeyInitialized) return false;
  
  if (outPlainCap < ctLen + 1) return false;
  ChaChaPoly aead;
  aead.setKey(CRYPTO_KEY, sizeof(CRYPTO_KEY));
  aead.setIV(nonce12, 12);
  aead.decrypt((uint8_t*)outPlain, ciphertext, ctLen);
  bool ok = aead.checkTag(tag, 16);
  if (ok) outPlain[ctLen] = '\0';
  return ok;
}

// =========================================================================
// NETWORK TIME HELPERS (delegated to Sim7070G library)
// =========================================================================

bool getNetworkTimeISO8601(char* isoOut, size_t outCap) {
  // This function should be called with modem instance
  // For now, return false - caller should use Sim7070G::getNetworkTimeISO8601 directly
  (void)isoOut;
  (void)outCap;
  return false;
}

bool parseCCLKToISO(const char* cclkResp, char* isoOut, size_t outCap) {
  // This function should be called with modem instance
  // For now, return false - caller should use Sim7070G::parseCCLKToISO directly
  (void)cclkResp;
  (void)isoOut;
  (void)outCap;
  return false;
}
