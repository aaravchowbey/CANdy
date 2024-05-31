#include <hmac_sha256.h>

#define Serial SerialUSB

#define SPEED CAN_BPS_50K

#define HAMMER_BIT_COUNT 4  // total number of bits to hammer for message frame
#define HAMMER_SIZE 2       // number of bits to hammer per data bit

#define SET_LED(state) \
  do { \
    (state) ? (PIOB->PIO_SODR = PIO_PB27) : (PIOB->PIO_CODR = PIO_PB27); \
    (state) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26); \
  } while (0)

#define CANdy_Write(value) \
  do { \
    (value) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26); \
  } while (0)
// PIO_PA0A_CANTX0

volatile bool hammer_state = false;

volatile bool resetValue = false;

volatile uint32_t frame_queue;
volatile uint8_t bits_after_prev_stuff, frame_bits;

// data bits to hammer
uint8_t hammer_data[32] = { 0b01010101, 0, 0, 0, 0 };
const char key[] = "super-secret-key";

// current index of hammer_data
volatile uint8_t hammer_index = 0;

// bit in frame has been completely hammered
volatile bool frameBitHammered = false;

union {
  uint16_t val;
  uint8_t arr[2];
} frame_id;

void CAN0_Handler() {
  // runs on second frame bit at ~70%
  if (CAN0->CAN_SR & CAN_SR_TSTP) {
    // fires on third frame bit at ~70%
    startTimer(TC0, 0, TC0_IRQn, SPEED * 1000);

    // set frame_queue
    bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frame_queue = 0b11111111111111111111111111111100 | value;

    // reset values
    bits_after_prev_stuff = 2;
    frame_bits = 2;
    hammer_index = 0;
  }
}

// samples bits to determine when data bits start
void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  if (bits_after_prev_stuff >= 5 && ((value == 0 && (frame_queue & 0b11111) == 0b11111) || (value == 1 && (frame_queue & 0b11111) == 0))) {
    // if stuff bit, reset frame_samples
    bits_after_prev_stuff = 0;
  } else {
    // if not stuff bit, add to frame_queue
    frame_queue = ((frame_queue & 0x7FFFFFFF) << 1) | value;

    // increment counters
    bits_after_prev_stuff++;
    frame_bits++;

    if (frame_bits == 19) {
      // if 19 bits sent, check if correct and start hammering
      if ((frame_queue & 0b1111) != 0 &&     // DLC (must be > 0)
          (frame_queue & 0b1110000) == 0 &&  // reserved bit + IDE + RTR (must be 0)
          (frame_queue & 0x40000) == 0       // start of frame
      ) {
        // hammer_data = get_hmac((frame_queue & 0x3FF80) >> 7);
        startTimer(TC1, 0, TC3_IRQn, (uint32_t)(SPEED * 1000 * 3.333));

        frame_id.val = (uint16_t)((frame_queue & 0x3FF80) >> 7);
        hmac_sha256(key, 16, frame_id.arr, 2, hammer_data, 32);
      }

      // stops frame timer
      stopTimer(TC0, 0, TC0_IRQn);
    }
  }
}

// runs hammering at ~20% for first data bit; helps synchronize later hammering
void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  if (hammer_index < HAMMER_BIT_COUNT) {
    // starts TC4 at next data bit (~0% mark)
    startTimer(TC1, 1, TC4_IRQn, SPEED * 1000);

    // turn on multiplexing
    // PIOA->PIO_PER = PIO_PA0A_CANTX0;
    // PIOA->PIO_OER = PIO_PA0A_CANTX0;

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
    // PIOA->PIO_PER = PIO_PA0A_CANTX0;
    // PIOA->PIO_OER = PIO_PA0A_CANTX0;

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
    // PIOA->PIO_PDR = PIO_PA0A_CANTX0;
    // PIOA->PIO_ODR = PIO_PA0A_CANTX0;
  }
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  // enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  // enable the specified peripheral clock.
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency;

  TC_SetRA(tc, channel, rc / 2);
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void stopTimer(Tc *tc, uint32_t channel, IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}

void setup() {
  // start serial port at 115200 bps
  Serial.begin(115200);

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of LEDs
  PIOB->PIO_PER = PIO_PB27;
  PIOB->PIO_OER = PIO_PB27;

  // enable control of digital pin 22
  PIOB->PIO_PER = PIO_PB26;
  PIOB->PIO_OER = PIO_PB26;

  // initialize CAN0
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, SystemCoreClock, SPEED);

  // enable only timestamp interrupt and set to fire at SOF
  can_disable_interrupt(CAN0, CAN_DISABLE_ALL_INTERRUPT_MASK);
  can_enable_interrupt(CAN0, CAN_SR_TSTP);
  can_set_timestamp_capture_point(CAN0, 0);

  // enable CAN0 interrupt handler
  NVIC_EnableIRQ(CAN0_IRQn);

  // PIOA power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID11;

  // multiplex CAN_RX to GPIO (Peripheral Enable Register)
  PIOA->PIO_PER = PIO_PA1A_CANRX0;

  // set CAN_RX as input (Output Disable Register)
  PIOA->PIO_ODR = PIO_PA1A_CANRX0;

  // disable pull-up on both pins (Pull-Up Disable Register)
  PIOA->PIO_PUDR = PIO_PA1A_CANRX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;
}

void loop() {
}
