#include <hmac_sha256.h>
// #include <poly1305-donna.h>
#include <poly1305.hpp>

#define sda PIO_PC3
#define scl PIO_PC1
#define Serial SerialUSB

#define START_TIMER() \
  do { \
    PIOC->PIO_CODR = sda; \
    PIOC->PIO_SODR = sda; \
  } while (0);

#define STOP_TIMER() \
  do { \
    PIOC->PIO_CODR = scl; \
    PIOC->PIO_SODR = scl; \
  } while (0);


const unsigned char key[] = "super-secret-key";
const uint8_t msg[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };
uint8_t mac[32] = { 0 };

// unsigned char key[32];
// unsigned char mac[16];
// unsigned char msg[73];

void setup() {
  Serial.begin(115200);
  PMC->PMC_PCER0 |= PMC_PCER0_PID13;

  PIOC->PIO_PER = PIO_PC1;
  PIOC->PIO_PER = PIO_PC3;

  PIOC->PIO_OER = PIO_PC1;
  PIOC->PIO_OER = PIO_PC3;

  PIOC->PIO_PUDR = PIO_PC1;
  PIOC->PIO_PUDR = PIO_PC3;

  PIOC->PIO_CODR = PIO_PC1;
  PIOC->PIO_CODR = PIO_PC3;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  for (int i = 0; i < 5; i++) {
    PIOC->PIO_CODR = PIO_PC1;
    PIOC->PIO_CODR = PIO_PC3;

    PIOC->PIO_SODR = PIO_PC1;
    // delayMicroseconds(10);

    poly1305_auth(mac, msg, sizeof(msg), key);
    // hmac_sha256(key, sizeof(key) - 1, msg, 8, mac, 32);
    // poly1305_auth(mac, msg, sizeof(msg), key);
    // Serial.println(mac[0], HEX);

    PIOC->PIO_SODR = PIO_PC3;

    delayMicroseconds(10);
  }
  delay(100);


  // START_TIMER();
  // // poly1305
  // STOP_TIMER();
}
