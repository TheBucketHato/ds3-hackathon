/*
 * lora_crypto.h — AES-256-GCM payload encryption for the LoRa link.
 *
 * Wraps the ASCII payload (everything AFTER the 4-byte RadioHead header) as:
 *     nonce(12) || ciphertext(N) || tag(16)
 * matching the Jetson side (Python `cryptography` AESGCM, which returns
 * ciphertext||tag). 12-byte nonce, 16-byte GCM tag, no AAD.
 *
 * The shared 32-byte key is defined in lora_crypto.cpp — keep it identical on
 * both ends. (The algorithm is AES-256, NSA-approved for TOP SECRET; the link's
 * real strength is only as good as keeping this key secret.)
 */
#pragma once
#include <Arduino.h>

#define LORA_NONCE_LEN 12
#define LORA_TAG_LEN   16
#define LORA_OVERHEAD  (LORA_NONCE_LEN + LORA_TAG_LEN)   // 28 bytes

void   loraCryptoBegin();

// Encrypt `plain[0..plen)` into `out` as nonce||ciphertext||tag.
// `out` must hold at least plen + LORA_OVERHEAD bytes. Returns bytes written.
size_t loraEncrypt(const uint8_t *plain, size_t plen, uint8_t *out);

// Decrypt `in[0..inLen)` (nonce||ciphertext||tag) into `out`.
// Returns plaintext length, or -1 on auth failure / malformed packet.
int    loraDecrypt(const uint8_t *in, size_t inLen, uint8_t *out);
