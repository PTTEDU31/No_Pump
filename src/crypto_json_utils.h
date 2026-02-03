#ifndef CRYPTO_JSON_UTILS_H
#define CRYPTO_JSON_UTILS_H

#include <Arduino.h>
#include <Crypto.h>
#include <ChaChaPoly.h>
#include "config.h"

// Forward declaration
class Sim7070G;

// =========================================================================
// HEX ENCODING/DECODING
// =========================================================================

/**
 * Encode binary data to hexadecimal string
 * @param in Input binary data
 * @param inLen Length of input data
 * @param out Output buffer for hex string
 * @param outCap Capacity of output buffer
 * @param uppercase Use uppercase letters (default: true)
 */
void hexEncode(const uint8_t* in, size_t inLen, char* out, size_t outCap, bool uppercase = true);

/**
 * Decode hexadecimal string to binary data
 * @param hex Input hex string
 * @param out Output buffer for binary data
 * @param outCap Capacity of output buffer
 * @param outLen Output: actual length of decoded data
 * @return true if successful, false otherwise
 */
bool hexDecode(const char* hex, uint8_t* out, size_t outCap, size_t& outLen);

// =========================================================================
// JSON PARSING HELPERS
// =========================================================================

/**
 * Get string value from JSON
 * @param json JSON string
 * @param key Key to search for
 * @param out Output buffer
 * @param cap Capacity of output buffer
 * @return true if found and extracted, false otherwise
 */
bool jsonGetString(const char* json, const char* key, char* out, size_t cap);

/**
 * Get boolean value from JSON
 * @param json JSON string
 * @param key Key to search for
 * @param out Output boolean value
 * @return true if found and extracted, false otherwise
 */
bool jsonGetBool(const char* json, const char* key, bool& out);

/**
 * Get integer value from JSON
 * @param json JSON string
 * @param key Key to search for
 * @param out Output integer value
 * @return true if found and extracted, false otherwise
 */
bool jsonGetInt(const char* json, const char* key, long& out);

/**
 * Get float value from JSON
 * @param json JSON string
 * @param key Key to search for
 * @param out Output float value
 * @return true if found and extracted, false otherwise
 */
bool jsonGetFloat(const char* json, const char* key, float& out);

// =========================================================================
// CRYPTO FUNCTIONS
// =========================================================================

/**
 * Generate 12-byte nonce from network time
 * Format: [Y(2 bytes) | Mo | D | H | Mi | S | counter(5 bytes)]
 * @param nonce Output buffer (12 bytes)
 */
void makeNonce12(uint8_t nonce[12]);

/**
 * Generate 12-byte nonce from network time (with modem instance)
 * Format: [Y(2 bytes) | Mo | D | H | Mi | S | counter(5 bytes)]
 * @param nonce Output buffer (12 bytes)
 * @param modem Sim7070G modem instance for getting network time
 */
void makeNonce12(uint8_t nonce[12], Sim7070G* modem);

/**
 * Encrypt plaintext using ChaCha20-Poly1305
 * @param plaintext Input plaintext (null-terminated string)
 * @param ciphertext Output buffer for ciphertext
 * @param tag Output buffer for authentication tag (16 bytes)
 * @param nonce12 12-byte nonce
 * @return Length of ciphertext
 */
size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], const uint8_t nonce12[12]);

/**
 * Encrypt plaintext using ChaCha20-Poly1305 (with modem instance for network time)
 * @param plaintext Input plaintext (null-terminated string)
 * @param ciphertext Output buffer for ciphertext
 * @param tag Output buffer for authentication tag (16 bytes)
 * @param modem Sim7070G modem instance for getting network time
 * @return Length of ciphertext, or 0 on error
 */
size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], Sim7070G* modem);

/**
 * Decrypt ciphertext using ChaCha20-Poly1305
 * @param ciphertext Input ciphertext
 * @param ctLen Length of ciphertext
 * @param tag Authentication tag (16 bytes)
 * @param nonce12 12-byte nonce
 * @param outPlain Output buffer for plaintext
 * @param outPlainCap Capacity of output buffer
 * @return true if decryption and tag verification successful, false otherwise
 */
bool decryptPayload(const uint8_t* ciphertext, size_t ctLen, const uint8_t tag[16], 
                    const uint8_t nonce12[12], char* outPlain, size_t outPlainCap);

// =========================================================================
// NETWORK TIME HELPERS
// =========================================================================

/**
 * Get network time in ISO-8601 format (YYYY-MM-DDTHH:MM:SSZ)
 * @param isoOut Output buffer
 * @param outCap Capacity of output buffer (should be at least 21)
 * @return true if successful, false otherwise
 */
bool getNetworkTimeISO8601(char* isoOut, size_t outCap);

/**
 * Parse CCLK response to ISO-8601 format
 * @param cclkResp CCLK response string
 * @param isoOut Output buffer for ISO-8601 string
 * @param outCap Capacity of output buffer
 * @return true if successful, false otherwise
 */
bool parseCCLKToISO(const char* cclkResp, char* isoOut, size_t outCap);

/**
 * Callback type: fill buffer with AT+CCLK? response, return true if "+CCLK:" present
 */
typedef bool (*GetCCLKResponseFn)(char* buf, size_t cap);

/**
 * Set callback used by getNetworkTimeISO8601() to obtain CCLK response (e.g. from modem)
 */
void setGetCCLKResponse(GetCCLKResponseFn fn);

#endif // CRYPTO_JSON_UTILS_H
