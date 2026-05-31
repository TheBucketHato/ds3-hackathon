#include "lora_crypto.h"
#include "src/AES.h"     // vendored rweather Crypto (AES-GCM only, see sketch/src)
#include "src/GCM.h"

// Shared 32-byte AES-256 key — MUST be identical on the Jetson. Replace before
// any real deployment; whoever can read this flash gets the key.
static const uint8_t KEY[32] = {
  0x31,0x61,0xce,0x43,0x1b,0x04,0x5a,0x53,
  0x80,0x4f,0xe8,0x0c,0xa3,0xd7,0xa9,0x06,
  0x1b,0xc1,0x82,0xde,0xd1,0x07,0xeb,0xd2,
  0x21,0x52,0xcc,0x29,0x3e,0x03,0x03,0x0e
};

static GCM<AES256> gcm;
static uint8_t  nonceSalt[4];
static uint64_t nonceCtr = 0;

void loraCryptoBegin() {
  // GCM requires a unique nonce per message under one key. The 64-bit counter
  // guarantees uniqueness within a boot; a per-boot random salt keeps nonces
  // from repeating across reboots. Seed the salt from noise + time.
  uint32_t seed = micros();
  for (int i = 0; i < 8; i++) seed = seed * 1103515245u + 12345u + analogRead(A3);
  randomSeed(seed);
  for (int i = 0; i < 4; i++) nonceSalt[i] = (uint8_t)random(0, 256);
  nonceCtr = 0;
}

static void makeNonce(uint8_t *n) {
  memcpy(n, nonceSalt, 4);
  uint64_t c = nonceCtr++;
  for (int i = 0; i < 8; i++) n[4 + i] = (uint8_t)(c >> (8 * i));
}

size_t loraEncrypt(const uint8_t *plain, size_t plen, uint8_t *out) {
  uint8_t *nonce = out;
  makeNonce(nonce);
  gcm.clear();
  gcm.setKey(KEY, sizeof(KEY));
  gcm.setIV(nonce, LORA_NONCE_LEN);
  gcm.encrypt(out + LORA_NONCE_LEN, plain, plen);
  gcm.computeTag(out + LORA_NONCE_LEN + plen, LORA_TAG_LEN);
  return LORA_NONCE_LEN + plen + LORA_TAG_LEN;
}

int loraDecrypt(const uint8_t *in, size_t inLen, uint8_t *out) {
  if (inLen < LORA_OVERHEAD) return -1;
  size_t plen = inLen - LORA_OVERHEAD;
  const uint8_t *nonce = in;
  const uint8_t *ct    = in + LORA_NONCE_LEN;
  const uint8_t *tag   = in + LORA_NONCE_LEN + plen;
  gcm.clear();
  gcm.setKey(KEY, sizeof(KEY));
  gcm.setIV(nonce, LORA_NONCE_LEN);
  gcm.decrypt(out, ct, plen);
  if (!gcm.checkTag(tag, LORA_TAG_LEN)) return -1;   // tamper / wrong key -> reject
  return (int)plen;
}
