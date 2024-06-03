#include <due_can-CANdy.h>

#define Serial SerialUSB

#define HAMMER_BIT_COUNT 5  // total number of bits to hammer for message frame
#define HAMMER_SIZE 1       // number of bits to hammer per data bit

#define SPEED 50

#define CANdy_Write(value) \
  do { \
    (value) ? (PIOA->PIO_SODR = PIO_PA0A_CANTX0) : (PIOA->PIO_CODR = PIO_PA0A_CANTX0); \
  } while (0)
// PIO_PA0A_CANTX0

volatile bool hammer_state = false;

volatile bool resetValue = false;

volatile uint8_t frame_queue;
volatile uint8_t bits_after_prev_stuff, frame_bits;

// data bits to hammer
uint8_t hammer_data[32] = { 0b11111111, 0, 0, 0, 0 };
const unsigned char key[] = "super-secret-key";

// current index of hammer_data
volatile uint8_t hammer_index = 0;

// bit in frame has been completely hammered
volatile bool frameBitHammered = false;

volatile uint8_t data_length = 0;

static unsigned long lastTime = 0;

// runs hammering at ~20% for first data bit; helps synchronize later hammering
void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  hammer_index = 0;
  if (hammer_index < HAMMER_BIT_COUNT) {
    // starts TC4 at next data bit (~0% mark)
    startTimer(TC1, 1, TC4_IRQn, SPEED * 1000);

    // turn on multiplexing
    PIOA->PIO_OER = PIO_PA0A_CANTX0;
    PIOA->PIO_PER = PIO_PA0A_CANTX0;

    resetValue = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frameBitHammered = false;

    // start TC7 to fire 5 times in a bit (~20% mark)
    startTimer(TC2, 0, TC6_IRQn, SPEED * 1000 * 5);
  }

  // stops first data bit hammering timer
  stopTimer(TC1, 0, TC3_IRQn);
}

// handles hammering bits for all data bits
void TC4_Handler() {
  TC_GetStatus(TC1, 1);

  if (hammer_index < HAMMER_BIT_COUNT) {
    // if bits left to hammer, setup hammering for one data bit
    resetValue = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frameBitHammered = false;

    // turn on multiplexing
    PIOA->PIO_OER = PIO_PA0A_CANTX0;
    PIOA->PIO_PER = PIO_PA0A_CANTX0;

    // start TC7 to fire 5 times in a bit (~20% mark)
    startTimer(TC2, 0, TC6_IRQn, SPEED * 1000 * 5);
  } else {
    // stop timer when no more bits to hammer
    stopTimer(TC1, 1, TC4_IRQn);
  }
}

// handles hammering bits for one data bit
void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  if (!frameBitHammered) {
    // if bit in frame has not been completely hammered, continue with hammering
    CANdy_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);

    hammer_index++;

    frameBitHammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    // reset value
    CANdy_Write(resetValue);

    // stop TC6
    stopTimer(TC2, 0, TC6_IRQn);

    // turn off multiplexing
    // PIOA->PIO_ODR = PIO_PA0A_CANTX0;
    PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Can0.begin(SPEED * 1000);

  PMC->PMC_PCER0 |= PMC_PCER0_PID11;  // PIOA power ON

  PIOA->PIO_PDR = PIO_PA0A_CANTX0;  // unable to send data w/ PER so changed to PDR (2024-03-13)
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of digital pin 22
  PIOB->PIO_PER = PIO_PB26;
  PIOB->PIO_OER = PIO_PB26;

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);

  Can0.watchFor();
}

void sendData(const uint8_t* data, const int dataLength) {
  CAN_FRAME outgoing;
  outgoing.id = 0x400;
  outgoing.extended = false;
  outgoing.priority = 4;
  outgoing.length = dataLength;

  for (int i = 0; i < dataLength; i++) {
    outgoing.data.byte[i] = data[i];
  }

  if (Can0.sendFrame(outgoing)) {
    // if frame successfully sent, flip LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void loop() {
  const int dataLength = 8;
  // const uint8_t data[dataLength] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
  const uint8_t data[dataLength] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  // const uint8_t data[dataLength] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  // NOTE: avoid use of delay since it does not work well w/ interrupts
  if (millis() - lastTime > 1000) {
    lastTime = millis();
    sendData(data, dataLength);
  }
}
