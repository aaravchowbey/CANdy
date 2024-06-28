#define Serial SerialUSB

#define SPEED CAN_BPS_50K

#define HAMMER_BIT_COUNT 8       // total number of bits to hammer for message frame
#define HAMMER_SIZE 1            // number of bits to hammer per data bit
#define HAMMER_START 20.f        // percentage
#define HAMMER_BIT_WIDTH 30.f    // percentage

#define FRAME_DELAY_AMOUNT 0.0f

const uint32_t speed_freq = SPEED * 1000;
const uint32_t hammer_freq = SPEED * 1000 * (100.f / HAMMER_BIT_WIDTH);
const uint32_t frame_sync_delayed_freq = SPEED * 1000 * (100.f / (HAMMER_START + HAMMER_BIT_WIDTH * 0.7f + FRAME_DELAY_AMOUNT));

volatile bool frame_value = false;
volatile uint8_t frame_queue = 0;
volatile uint8_t bits_after_prev_stuff = 0, frame_bits = 0;
uint8_t hammer_data[32] = { 0 };
uint8_t hammer_index = 0;

// bit in frame has been completely hammered
volatile bool frame_bit_hammered = false;

volatile uint8_t data_length = 8;

// uint64_t bus_queue = UINT64_MAX;
uint64_t bus_queue[2] = {UINT64_MAX, UINT64_MAX};
bool sof = false;

// #define SAMPLING_SPEED 11

// #define IS_MODEM

#define PIN_WRITE(value) \
  do { \
    (value) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26); \
  } while (0);

#define PIN_ON \
  do { \
    PIOB->PIO_SODR = PIO_PB26; \
  } while (0);
#define PIN_OFF \
  do { \
    PIOB->PIO_CODR = PIO_PB26; \
  } while (0);

void mailbox_int_handler(uint8_t mb) {
  if (CAN0->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) {       // mailbox signals it is ready
    switch (((CAN0->CAN_MB[mb].CAN_MMR >> 24) & 7)) {  // what sort of mailbox is it?
      case 1:                                          // receive
      case 2:                                          // receive w/ overwrite
      case 4:                                          // consumer - technically still a receive buffer
        can_mailbox_send_transfer_cmd(CAN0, mb);
        break;
      case 3:  // transmit
        can_disable_interrupt(CAN0, 0x01u << mb);
        break;
      case 5:  // producer - technically still a transmit buffer
        break;
    }
  }
}

void CAN0_Handler() {
  // get status of interrupts
  uint32_t ul_status = CAN0->CAN_SR;

  // if (ul_status & CAN_SR_TSTP) {  // timestamp - start of frame (70% of second bit)
  //   startTimer(TC0, 0, TC0_IRQn, speed_freq);
  //   // set frame_queue
  //   frame_value = 0;
  //   frame_queue = 0b11111100 | (uint8_t)((PIOA->PIO_PDSR & PIO_PA1A_CANRX0) ? 1 : 0);
  //
  //   // reset values
  //   bits_after_prev_stuff = 2;
  //   frame_bits = 2;
  //   hammer_index = 0;
  // }
  if (ul_status & CAN_SR_MB0) {  // mailbox 0 event
    mailbox_int_handler(0);
  }
  if (ul_status & CAN_SR_MB1) {  // mailbox 1 event
    mailbox_int_handler(1);
  }
  if (ul_status & CAN_SR_MB2) {  // mailbox 2 event
    mailbox_int_handler(2);
  }
  if (ul_status & CAN_SR_MB3) {  // mailbox 3 event
    mailbox_int_handler(3);
  }
  if (ul_status & CAN_SR_MB4) {  // mailbox 4 event
    mailbox_int_handler(4);
  }
  if (ul_status & CAN_SR_MB5) {  // mailbox 5 event
    mailbox_int_handler(5);
  }
  if (ul_status & CAN_SR_MB6) {  // mailbox 6 event
    mailbox_int_handler(6);
  }
  if (ul_status & CAN_SR_MB7) {  // mailbox 7 event
    mailbox_int_handler(7);
  }
}

void TC1_Handler() {
  TC_GetStatus(TC0, 1);

  bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  // check if bus_queue contains sof
  if (!sof) {
    // if not sof, add to bus_queue
    #if SAMPLING_SPEED < 6
    bus_queue = ((bus_queue & 0x7FFFFFFFFFFFFFFF) << 1) | (uint64_t)(value ? 1 : 0);
    #else
    bus_queue[0] = ((bus_queue[0] & 0x7FFFFFFFFFFFFFFF) << 1) | ((bus_queue[1] & 0x8000000000000000) >> 63);
    bus_queue[1] = ((bus_queue[1] & 0x7FFFFFFFFFFFFFFF) << 1) | (uint64_t)(value ? 1 : 0);
    #endif;

    #if SAMPLING_SPEED == 11
    if ((bus_queue[0] & 0b111111111111111111111111111111111111111111111111111111111) == 0b111111111111111111111111111111111111111111111111111111111 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111100000000000) {
    #elif SAMPLING_SPEED == 10
    if ((bus_queue[0] & 0b1111111111111111111111111111111111111111111111) == 0b1111111111111111111111111111111111111111111111 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111110000000000) {
    #elif SAMPLING_SPEED == 9
    if ((bus_queue[0] & 0b11111111111111111111111111111111111) == 0b11111111111111111111111111111111111 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111111000000000) {
    #elif SAMPLING_SPEED == 8
    if ((bus_queue[0] & 0b111111111111111111111111) == 0b111111111111111111111111 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111111100000000) {
    #elif SAMPLING_SPEED == 7
    if ((bus_queue[0] & 0b1111111111111) == 0b1111111111111 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111111110000000) {
    #elif SAMPLING_SPEED == 6
    if ((bus_queue[0] & 0b11) == 0b11 &&
        bus_queue[1] == 0b1111111111111111111111111111111111111111111111111111111111000000) {
    #elif SAMPLING_SPEED == 5
    if ((bus_queue & 0x7FFFFFFFFFFFFF) == 0x7FFFFFFFFFFFE0) {
    #endif
      sof = true;
    }
  } else {
    startTimer(TC0, 0, TC0_IRQn, speed_freq);
    stopTimer(TC0, 1, TC1_IRQn);
    sof = false;

    // set frame_queue
    frame_value = 0;
    frame_queue = 0b11111110;// | (uint8_t)(value ? 1 : 0);

    // reset values
    bits_after_prev_stuff = 1;
    frame_bits = 1;
    hammer_index = 0;
  }
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  frame_value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  if (bits_after_prev_stuff >= 5 && ((frame_value == 0 && (frame_queue & 0b11111) == 0b11111) || (frame_value == 1 && (frame_queue & 0b11111) == 0))) {
    // if stuff bit, reset frame_samples
    bits_after_prev_stuff = 0;
  } else {
    // if not stuff bit, add to frame_queue
    frame_queue = ((frame_queue & 0b1111111) << 1) | frame_value;

    // increment counters
    bits_after_prev_stuff++;
    frame_bits++;

#ifdef IS_MODEM
    if (frame_bits == 15 && (frame_value & 0b111) != 0) {
      // stopTimer(TC0, 0, TC0_IRQn);
    } else if (frame_bits == 19) {
      if ((frame_queue & 0b1111) != 0) {
        data_length = frame_queue & 0b1111;
      } else {
        data_length = 0;
        // TODO: figure out why stopTimer fails to work here
        // stopTimer(TC0, 0, TC0_IRQn);
      }
    } else if (data_length != 0 && frame_bits > 20 && frame_bits <= 20 + data_length * 8) {
      if (frame_bits == 20 + data_length * 8) {
        startTimer(TC1, 0, TC3_IRQn, frame_sync_delayed_freq);
        stopTimer(TC0, 0, TC0_IRQn);
      }
    }
#else
    if (frame_bits == 15 && (frame_value & 0b111) != 0) {
      stopTimer(TC0, 0, TC0_IRQn);
    } else if (frame_bits == 19) {
      startTimer(TC1, 0, TC3_IRQn, frame_sync_delayed_freq);
      stopTimer(TC0, 0, TC0_IRQn);
    }
#endif
  }
}

void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  if (hammer_index < HAMMER_BIT_COUNT) {
    startTimer(TC1, 1, TC4_IRQn, speed_freq);
    startTimer(TC2, 0, TC6_IRQn, hammer_freq);
    // PIN_ON;
    hammer_data[hammer_index / 8] |= (uint8_t)((PIOA->PIO_PDSR & PIO_PA1A_CANRX0) ? 1 : 0) << (hammer_index % 8);
    hammer_index++;
    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    printData();
  }

  stopTimer(TC1, 0, TC3_IRQn);
}

// handles hammering bits for all data bits
void TC4_Handler() {
  TC_GetStatus(TC1, 1);

  if (hammer_index < HAMMER_BIT_COUNT) {
    startTimer(TC2, 0, TC6_IRQn, hammer_freq);
    // PIN_ON;
    hammer_data[hammer_index / 8] |= (uint8_t)((PIOA->PIO_PDSR & PIO_PA1A_CANRX0) ? 1 : 0) << (hammer_index % 8);
    hammer_index++;
    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    printData();
    stopTimer(TC1, 1, TC4_IRQn);
  }
}

void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  if (!frame_bit_hammered) {
    // if bit in frame has not been completely hammered, continue with hammering
    hammer_data[hammer_index / 8] |= (uint8_t)((PIOA->PIO_PDSR & PIO_PA1A_CANRX0) ? 1 : 0) << (hammer_index % 8);
    hammer_index++;
    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    // PIN_OFF;
    stopTimer(TC2, 0, TC6_IRQn);
  }
}

void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  // disable write protect of PMC registers
  pmc_set_writeprotect(false);
  // enable the specified peripheral clock
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR | TC_CMR_TCCLKS_TIMER_CLOCK4);
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

  // PIOA power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID11;

  // multiplex CANTX0
  PIOA->PIO_OER = PIO_PA0A_CANTX0;
  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of digital pin 22
  PIOB->PIO_OER = PIO_PB26;
  PIOB->PIO_PER = PIO_PB26;

  // enable control of LED
  PIOB->PIO_OER = PIO_PB27;
  PIOB->PIO_PER = PIO_PB27;

  // init CAN0
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, SystemCoreClock, SPEED);

  // enable only timestamp interrupt and set to fire at SOF
  can_disable_interrupt(CAN0, CAN_DISABLE_ALL_INTERRUPT_MASK);
  // can_enable_interrupt(CAN0, CAN_SR_TSTP);
  // can_set_timestamp_capture_point(CAN0, 0);

  // enable CAN0 interrupt handler
  NVIC_EnableIRQ(CAN0_IRQn);

  // init RX boxes
  uint8_t c;
  for (c = 0; c < 8 - 1; c++) {
    // set mode to CAN_MB_RX_MODE
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_RX_MODE << CAN_MMR_MOT_Pos);

    // set id to 0x0
    CAN0->CAN_MB[c].CAN_MID = CAN_MID_MIDvA(0x0);

    // set accept mask to 0x7ff
    CAN0->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN0->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  // init TX boxes
  for (; c < 8; c++) {
    // set mode to CAN_MB_TX_MODE
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_TX_MODE << CAN_MMR_MOT_Pos);

    // set priority to 10
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (10 << CAN_MMR_PRIOR_Pos);

    // set accept mask to 0x7ff
    CAN0->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN0->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  // set up mailbox 0 for standard IDs
  CAN0->CAN_MB[0].CAN_MAM = CAN_MAM_MIDvA(0);
  CAN0->CAN_MB[0].CAN_MID &= ~CAN_MAM_MIDE;
  CAN0->CAN_MB[0].CAN_MID = CAN_MID_MIDvA(0);
  can_enable_interrupt(CAN0, CAN_IER_MB0);

  startTimer(TC0, 1, TC1_IRQn, speed_freq * SAMPLING_SPEED);
  NVIC_SetPriority(TC1_IRQn, 0);
}

void printData() {
  for (uint8_t i = 0; i < (HAMMER_BIT_COUNT + 1) / 8; i++) {
    Serial.println(hammer_data[i], HEX);
  }

  for (uint8_t i = 0; i < 32; i++) {
    hammer_data[i] = 0;
  }

  (PIOB->PIO_PDSR & PIO_PB27) ? PIOB->PIO_CODR = PIO_PB27 : PIOB->PIO_SODR = PIO_PB27;

  startTimer(TC0, 1, TC1_IRQn, speed_freq * SAMPLING_SPEED);
}

void loop() {
}
