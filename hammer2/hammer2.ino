
#include <due_can-CANdy.h>

#define Serial SerialUSB

#define SPEED              CAN_BPS_50K
#define HAMMER_BIT_COUNT   2           // total number of bits to hammer for message frame
#define HAMMER_SIZE        2           // number of bits to hammer per data bit
#define SAMPLING_RATE      5           // sampling rate of CAN bus

#define SET_LED(state)                                                         \
  do {                                                                         \
    (state) ? (PIOB->PIO_SODR = PIO_PB27) : (PIOB->PIO_CODR = PIO_PB27);       \
    (state) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26);       \
  } while (0)

#define CANdy_Write(value) \
  do {                                                                         \
    (value) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26);       \
  } while (0)
// PIO_PA0A_CANTX0

volatile bool resetValue = false;

volatile uint64_t bus_queue = UINT64_MAX;
volatile uint32_t frame_queue = 0b11111111111111111111111111111110;

// data bits to hammer
const uint64_t hammerData = 0b01;

// current index of hammerData
volatile uint8_t hammerIndex = 0;

// bit in frame has been completely hammered
volatile bool frameBitHammered = false;

volatile bool sof = false;
volatile uint8_t frame_samples = 1;

void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  // check if bus_queue contains sof
  if (!sof) {
    // if not sof, add to bus_queue
    bus_queue = ((bus_queue & 0x7FFFFFFFFFFFFFFF) << 1) | value;

    sof = (bus_queue & 0x7FFFFFFFFFFFFF) == 0x7FFFFFFFFFFFE0;
  } else {
    frame_queue = 0xFFFFFFFC | value;
    frame_samples = 2;
    hammerIndex = 0;

    startTimer(TC1, 2, TC5_IRQn, SPEED);
    stopTimer(TC0, 0, TC0_IRQn);
  }
}

void TC5_Handler() {
  TC_GetStatus(TC1, 2);

  bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  if (frame_samples >= 5 && ((value == 0 && (frame_queue & 0b11111) == 0b11111) || (value == 1 && (frame_queue & 0b11111) == 0))) {
    // if stuff bit, reset frame_samples
    frame_samples = 0;
  } else {
    // if not stuff bit, add to frame_queue
    frame_queue = ((frame_queue & 0x7FFFFFFF) << 1) | value;

    SET_LED(frame_samples % 2 == 0);

    frame_samples++;

    if (hammerIndex == 0 &&
        (frame_queue & 0b1111) != 0 && // DLC (must be > 0)
        (frame_queue & 0b1110000) == 0 && // reserved bit + IDE + RTR (must be 0)
        (frame_queue & 0x40000) == 0 // start of frame
    ) {
      // if DLC complete, start hammering timer for incoming data bit
      // startTimer(TC2, 0, TC6_IRQn, SPEED);
    } else if (frame_samples >= 8 && (frame_queue & 0b11111111) == 0b11111111) {
      // if end of frame, stop frame timer and starting sof detection timer
      frame_queue = 0b11111111111111111111111111111110;
      frame_samples = 1;
      // startTimer(TC0, 0, TC0_IRQn, SPEED * 5);
      stopTimer(TC1, 2, TC5_IRQn);
    }
  }
}

// handles hammering bits for all data bits
void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  if (hammerIndex < HAMMER_BIT_COUNT) {
    // Serial.println(hammerIndex);
    // if bits left to hammer, setup hammering for one data bit
    resetValue = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
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
    CANdy_Write((hammerData & (1 << (HAMMER_SIZE - hammerIndex - 1))) >> hammerIndex);

    hammerIndex++;

    frameBitHammered = hammerIndex % HAMMER_SIZE == 0 || hammerIndex == HAMMER_BIT_COUNT;
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

// void CAN0_Handler() {
//   Can0.interruptHandler();
//   uint32_t status = CAN0->CAN_SR;
//   // if (status & CAN_SR_RXRDY) {
//   //   // Frame received successfully, possible EOF
//   //   uint16_t timestamp = CAN0->CAN_TIM;
//   //   // Process EOF
//   //   Serial.print("EOF detected at: ");
//   //   Serial.println(timestamp);
//   // }
//   // if (status & CAN_SR_TXOK) {
//   //   // Frame transmitted successfully, possible SOF
//   //   uint16_t timestamp = CAN0->CAN_TIM;
//   //   // Process SOF
//   //   Serial.print("SOF detected at: ");
//   //   Serial.println(timestamp);
//   // }
// }
//
// void CAN1_Handler() {
//   Can1.interruptHandler();
//   uint32_t status = CAN1->CAN_SR;
//   // if (status & CAN_SR_RXOK) {
//   //   // Frame received successfully, possible EOF
//   //   uint16_t timestamp = CAN1->CAN_TIM;
//   //   // Process EOF
//   //   Serial.print("EOF detected at: ");
//   //   Serial.println(timestamp);
//   // }
//   // if (status & CAN_SR_TXOK) {
//   //   // Frame transmitted successfully, possible SOF
//   //   uint16_t timestamp = CAN1->CAN_TIM;
//   //   // Process SOF
//   //   Serial.print("SOF detected at: ");
//   //   Serial.println(timestamp);
//   // }
// }

// void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
//   //Enable or disable write protect of PMC registers.
//   pmc_set_writeprotect(false);
//   //Enable the specified peripheral clock.
//   pmc_enable_periph_clk((uint32_t)irq);
//
//   TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
//   uint32_t rc = VARIANT_MCK / 128 / frequency;
//
//   TC_SetRA(tc, channel, rc / 2);
//   TC_SetRC(tc, channel, rc);
//   TC_Start(tc, channel);
//
//   tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
//   tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
//   NVIC_EnableIRQ(irq);
// }
//
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

  Can0.enable_time_triggered_mode();
  Can0.set_timestamp_capture_point(0);

  Can1.enable_time_triggered_mode();
  Can1.set_timestamp_capture_point(1);

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

  // startTimer(TC0, 0, TC0_IRQn, SPEED * 5);
}

void loop() {
}
