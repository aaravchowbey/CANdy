#include <due_can.h>

#define Serial SerialUSB

#define SPEED              CAN_BPS_50K
#define HAMMER_BIT_COUNT   2           // total number of bits to hammer for message frame
#define HAMMER_SIZE        2           // number of bits to hammer per data bit
#define SAMPLING_RATE      10          // sampling rate of CAN bus

#define SET_LED(state)                                                         \
  do {                                                                         \
    (state) ? (PIOB->PIO_SODR = PIO_PB27) : (PIOB->PIO_CODR = PIO_PB27);       \
    (state) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26);       \
  } while (0)

#define ONE_BITS(x) (((uint64_t)1 << (x)) - 1)
#define BITS_AT_POS(queue, pos, num_bits)                                      \
  (queue[pos * SAMPLING_RATE / 64] &                                           \
   (ONE_BITS(num_bits * SAMPLING_RATE) << (SAMPLING_RATE * pos % 64)))

volatile bool resetValue = false;

uint64_t bus_queue[2] = {UINT64_MAX, UINT64_MAX};
uint64_t frame_queue[3] = {UINT64_MAX, UINT64_MAX, UINT64_MAX};

// data bits to hammer
const bool hammerData[HAMMER_BIT_COUNT] = {0, 1};

// current index of hammerData
volatile int hammerIndex = 0;

// bit in frame has been completely hammered
volatile bool frameBitHammered = false;

volatile bool sof = false;
volatile uint8_t stuff_bit_counter = 10;
volatile uint8_t frame_samples = 0;

void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  bool value = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;

  if (!sof) {
    // SET_LED(false);
    // if not sof, add to bus_queue
    bus_queue[0] = ((bus_queue[0] & 0x7FFFFFFFFFFFFFFF) << 1) | ((bus_queue[1] & 0x8000000000000000) >> 63);
    bus_queue[1] = ((bus_queue[1] & 0x7FFFFFFFFFFFFFFF) << 1) | value;

    // check if bus_queue contains sof
    if ((bus_queue[0] & 0x3FFFFFFFFFFF) == 0x3FFFFFFFFFFF &&
        bus_queue[1] == 0xFFFFFFFFFFFFFC00) {
      sof = true;
      frame_queue[0] = UINT64_MAX;
      frame_queue[1] = UINT64_MAX;
      frame_queue[2] = 0xFFFFFFFFFFFFFC00;
      frame_samples = 10;
      stuff_bit_counter = 10;
    }
  } else {
    // else in frame

    // FIX: below shows CANdy_Hammer does not work properly
    // SET_LED(true);
    // sof = false;
    // CANdy_Hammer();
    // return;
    // Serial.println((frame_queue[2] & 0x3FF) == 0x3FF);

    // FIX: below shows that value is broken!
    // if (frame_samples == 10) {
    //   SET_LED(true);
    // } else if ((frame_queue[2] & 0x3FF) == 0x3FF) {
    //   SET_LED(false);
    //   sof = false;
    // }
    // return;
    // Serial.println(value);
    // Serial.print((uint32_t)(frame_queue[2] & UINT32_MAX));
    // Serial.print(" ");
    // Serial.print(frame_samples);
    // Serial.print("\n");

    // check if frame_queue has a stuff bit
    uint64_t result = (frame_queue[2] & 0x3FFFFFFFFFFFF);
    if (frame_samples >= 50 && ((!value && result == 0x3FFFFFFFFFFFF) || (value && result == 0))) {
      // SET_LED(false);
      stuff_bit_counter--;

      if (stuff_bit_counter == 0) {
        stuff_bit_counter = 10;
        frame_samples = 0;
      }
    } else {
      // if not stuff bit, add to frame_queue
      frame_queue[0] = ((frame_queue[0] & 0x7FFFFFFFFFFFFFFF) << 1) |
                       ((frame_queue[1] & 0x8000000000000000) >> 63);
      frame_queue[1] = ((frame_queue[1] & 0x7FFFFFFFFFFFFFFF) << 1) |
                       ((frame_queue[2] & 0x8000000000000000) >> 63);
      frame_queue[2] = ((frame_queue[2] & 0x7FFFFFFFFFFFFFFF) << 1) | value;

      frame_samples++;

      if (frame_samples >= 190) {
        sof = false;
        // SET_LED(false);
        // CANdy_Hammer();
      } else if ((frame_queue[2] & 0xFFFFFFFFFF) != 0 && // DLC (must be > 0)
                 (frame_queue[2] & 0xFFFFFF0000000000) == 0 && // reserved bit (must be 0)
                 (frame_queue[1] & 0x3F) == 0 && // RTR (0 for data frames) (first 6 samples)
                 (frame_queue[0] & 0x3FF0000000000000) == 0 // start of frame
      ) {
        sof = false;
        // SET_LED(false);
        // CANdy_Hammer();
      }
    }
  }
}

// void TC3_Handler() {
//   TC_GetStatus(TC1, 0);
//
//   bool value = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;
//
//   bus_queue[2] = ((bus_queue[2] & ONE_BITS(63)) << 1) | ((bus_queue[1] & ((uint64_t)1 << 63)) >> 63);
//   bus_queue[1] = ((bus_queue[1] & ONE_BITS(63)) << 1) | ((bus_queue[0] & ((uint64_t)1 << 63)) >> 63);
//   bus_queue[0] = ((bus_queue[0] & ONE_BITS(63)) << 1) | value;
//
//   if (!sof && (bus_queue[2] & ONE_BITS(22)) == ONE_BITS(22) && bus_queue[1] == UINT64_MAX && bus_queue[0] == (ONE_BITS(54) << 10)) {
//     // check if 140 ones followed by 10 zeros is found in last 150 bits
//     // ([2] has 22 ones, [1] has 64 ones, [0] has 54 ones & 10 zeros)
//     sof = true;
//     frame_queue[0] = ONE_BITS(54) << 10;
//   } else if (sof) {
//     if ((frame_queue[2] & ((uint64_t)1 << 63)) == 0) {
//       sof = false;
//       return;
//     }
//
//     // if sof is true, continue until data bits in frame reached
//     if (!(value == 0 && (frame_queue[0] & ONE_BITS(50)) == ONE_BITS(50) &&  // value is 1 and previous 50 samples are 0s (and is not in default state)
//           frame_queue[0] != UINT64_MAX) &&
//         !(value == 1 && (frame_queue[0] & ONE_BITS(50)) == 0)               // value is 0 and previous 50 samples are 1s
//     ) {
//       // if not stuff bit, add to frame_queue
//       frame_queue[2] = ((frame_queue[2] & ONE_BITS(63)) << 1) | ((frame_queue[1] & ((uint64_t)1 << 63)) >> 63);
//       frame_queue[1] = ((frame_queue[1] & ONE_BITS(63)) << 1) | ((frame_queue[0] & ((uint64_t)1 << 63)) >> 63);
//
//       // FIX: requires explicit conversion to uint64_t for some reason; why?
//       frame_queue[0] = ((frame_queue[0] & ONE_BITS(63)) << 1) | ((uint64_t)(value ? 1 : 0));
//     }
//
//     if (
//         BITS_AT_POS(frame_queue, 18, 1) == 0 &&        // start-of-frame
//         (frame_queue[1] & ONE_BITS(6)) == 0 &&         // RTR (must be 0 for data frame) (first 6 samples)
//         (frame_queue[0] & (ONE_BITS(4) << 60)) == 0 && //                                (last 4 samples)
//         BITS_AT_POS(frame_queue, 5, 1) == 0 &&         // IDE
//         BITS_AT_POS(frame_queue, 4, 1) == 0 &&         // reserved bit
//         BITS_AT_POS(frame_queue, 0, 4) != 0            // DLC (must be > 0)
//         // (frame_queue[2] & (ONE_BITS(10) << 52)) == 0 &&
//         // (frame_queue[1] & ONE_BITS(6)) == 0 &&         // RTR (must be 0 for data frame) (first 6 samples)
//         // (frame_queue[0] & (ONE_BITS(4) << 60)) == 0 && //                                (last 4 samples)
//         // (frame_queue[0] & (ONE_BITS(10) << 50)) == 0 &&
//         // (frame_queue[0] & (ONE_BITS(10) << 40)) == 0 &&
//         // (frame_queue[0] & ONE_BITS(40)) != 0
//     ) {
//       frame_queue[2] = frame_queue[1] = frame_queue[0] = UINT64_MAX;
//       sof = false;
//
//       SET_LED(!ledState);
//
//       CANdy_Hammer();
//     }
//   }
//   SET_LED(sof);
// }

// starts hammering
void CANdy_Hammer() {
  hammerIndex = 0;

  startTimer(TC2, 0, TC6_IRQn, SPEED);
}

// handles hammering bits for all data bits
void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  SET_LED(true);

  if (hammerIndex < HAMMER_BIT_COUNT) {
    Serial.println(hammerIndex);
    // if bits left to hammer, setup hammering for one data bit
    resetValue = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;
    frameBitHammered = false;

    // turn on multiplexing
    // PIOA->PIO_PER = PIO_PA0A_CANTX0;
    // PIOA->PIO_OER = PIO_PA0A_CANTX0;

    // start TC7 to fire 5 times in a bit
    startTimer(TC2, 1, TC7_IRQn, SPEED * 5);
  } else {
    // stop timer when no more bits to hammer
    stopTimer(TC2, 0, TC6_IRQn);
  }
}

// handles hammering bits for one data bit
void TC7_Handler() {
  TC_GetStatus(TC2, 1);

  if (!frameBitHammered) {
    // if bit in frame has not been completely hammered, continue with hammering
    CANdy_Write(hammerData[hammerIndex]);

    hammerIndex++;

    if (hammerIndex % HAMMER_SIZE == 0 || hammerIndex == HAMMER_BIT_COUNT) {
      frameBitHammered = true;
    }
  } else {
    // stop TC7
    stopTimer(TC2, 1, TC7_IRQn);

    // reset value
    CANdy_Write(resetValue);

    // turn off multiplexing
    // PIOA->PIO_PDR = PIO_PA0A_CANTX0;
    // PIOA->PIO_ODR = PIO_PA0A_CANTX0;
  }
}

void CANdy_Write(bool value) {
  (value) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26);
  // if (value) {
  //   PIOA->PIO_SODR = PIO_PA0A_CANTX0;
  // } else {
  //   PIOA->PIO_CODR = PIO_PA0A_CANTX0;
  // }
}

void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  //Enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  //Enable the specified peripheral clock.
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

void stopTimer(Tc* tc, uint32_t channel, IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}

void setup() {
  // start serial port at 115200 bps
  Serial.begin(115200);

  // CAN0 and CAN1 initialization
  Can0.begin(SPEED);
  Can1.begin(SPEED);

  // PIOA power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID11;

  // multiplex CAN_RX to GPIO (Peripheral Enable Register)
  PIOA->PIO_PER = PIO_PA1A_CANRX0;

  // set CAN_RX as input (Output Disable Register)
  PIOA->PIO_ODR = PIO_PA1A_CANRX0;

  // disable pull-up on both pins (Pull-Up Disable Register)
  PIOA->PIO_PUDR = PIO_PA1A_CANRX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of LEDs
  PIOB->PIO_PER = PIO_PB27;
  PIOB->PIO_OER = PIO_PB27;

  // enable control of digital pin 22
  PIOB->PIO_PER = PIO_PB26;
  PIOB->PIO_OER = PIO_PB26;

  SET_LED(true);
  delay(500);
  SET_LED(false);
  delay(1000);

  Can0.watchFor();

  startTimer(TC1, 0, TC3_IRQn, SPEED * SAMPLING_RATE);
}

void loop() {
}
