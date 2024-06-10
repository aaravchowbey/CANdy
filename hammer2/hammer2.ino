// TODO: slight shift of hammering from last few data bytes to CRC
#define Serial SerialUSB

#define SPEED CAN_BPS_50K

#define HAMMER_BIT_COUNT 5  // total number of bits to hammer for message frame
#define HAMMER_SIZE 1       // number of bits to hammer per data bit

#define PIN_WRITE(value) \
  do { \
    (value) ? (PIOB->PIO_SODR = PIO_PB26) : (PIOB->PIO_CODR = PIO_PB26); \
  } while (0);

#define CAN1_Write(value) \
  do { \
    (value) ? (PIOB->PIO_SODR = PIO_PB14A_CANTX1) : (PIOB->PIO_CODR = PIO_PB14A_CANTX1); \
  } while (0);

#define CAN0_Write(value) \
  do { \
    (value) ? (PIOA->PIO_SODR = PIO_PA0A_CANTX0) : (PIOA->PIO_CODR = PIO_PA0A_CANTX0); \
  } while (0);

volatile bool hammer_state = false;

volatile bool frame_value = false;
volatile bool reset_value = false;

volatile uint8_t frame_queue = 0;
volatile uint8_t bits_after_prev_stuff = 0, frame_bits = 0;

// data bits to hammer
uint8_t hammer_data[32] = { 0b01010101, 0, 0, 0, 0 };
const unsigned char key[] = "super-secret-key";

// current index of hammer_data
volatile uint8_t hammer_index = 0;

// bit in frame has been completely hammered
volatile bool frame_bit_hammered = false;

volatile bool checking_for_sof = true;

volatile bool checking_for_can1_sof = true;
volatile bool replicate_frame_value = false;

volatile bool writing_message_to_ecu = false;
volatile bool sending_hammered_frame = false;

union {
  volatile uint64_t val;
  uint8_t arr[8];
} frame_data;

volatile uint8_t data_length = 8;

void mailbox_int_handler(Can* can, uint8_t mb) {
  if (can->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) {       // mailbox signals it is ready
    switch (((can->CAN_MB[mb].CAN_MMR >> 24) & 7)) {  // what sort of mailbox is it?
      case 1:                                         // receive
      case 2:                                         // receive w/ overwrite
      case 4:                                         // consumer - technically still a receive buffer
        can_mailbox_send_transfer_cmd(can, mb);
        break;
      case 3:  // transmit
        can_disable_interrupt(can, 0x01 << mb);
        break;
      case 5:  // producer - technically still a transmit buffer
        break;
    }
  }
}

void CAN0_Handler() {
  uint32_t ul_status = CAN0->CAN_SR;  // get status of interrupts

  if (ul_status & CAN_SR_MB0) {  // mailbox 0 event
    mailbox_int_handler(CAN0, 0);
  }
  if (ul_status & CAN_SR_MB1) {  // mailbox 1 event
    mailbox_int_handler(CAN0, 1);
  }
  if (ul_status & CAN_SR_MB2) {  // mailbox 2 event
    mailbox_int_handler(CAN0, 2);
  }
  if (ul_status & CAN_SR_MB3) {  // mailbox 3 event
    mailbox_int_handler(CAN0, 3);
  }
  if (ul_status & CAN_SR_MB4) {  // mailbox 4 event
    mailbox_int_handler(CAN0, 4);
  }
  if (ul_status & CAN_SR_MB5) {  // mailbox 5 event
    mailbox_int_handler(CAN0, 5);
  }
  if (ul_status & CAN_SR_MB6) {  // mailbox 6 event
    mailbox_int_handler(CAN0, 6);
  }
  if (ul_status & CAN_SR_MB7) {  // mailbox 7 event
    mailbox_int_handler(CAN0, 7);
  }

  // runs on second frame bit at ~70%
  if ((ul_status & CAN_SR_TSTP) && !writing_message_to_ecu) {
    sending_hammered_frame = checking_for_sof;

    if (checking_for_sof) {
      // fires instantly? TODO: find out why it fires instantly
      startTimer(TC0, 0, TC0_IRQn, SPEED * 1000);

      PIOB->PIO_PER = PIO_PB14A_CANTX1;

      // set frame_queue
      frame_value = 0;
      frame_queue = 0b11111110;
      CAN1_Write(0);

      // reset values
      bits_after_prev_stuff = 1;
      frame_bits = 1;
      hammer_index = 0;
    } else {
      stopTimer(TC0, 0, TC0_IRQn);
      PIOB->PIO_PDR = PIO_PB14A_CANTX1;
    }

    can_set_timestamp_capture_point(CAN0, (checking_for_sof ? 1 : 0));
    checking_for_sof = !checking_for_sof;
  }
}

void CAN1_Handler() {
  uint32_t ul_status = CAN1->CAN_SR;  // get status of interrupts

  if (ul_status & CAN_SR_MB0) {  // mailbox 0 event
    mailbox_int_handler(CAN1, 0);
  }
  if (ul_status & CAN_SR_MB1) {  // mailbox 1 event
    mailbox_int_handler(CAN1, 1);
  }
  if (ul_status & CAN_SR_MB2) {  // mailbox 2 event
    mailbox_int_handler(CAN1, 2);
  }
  if (ul_status & CAN_SR_MB3) {  // mailbox 3 event
    mailbox_int_handler(CAN1, 3);
  }
  if (ul_status & CAN_SR_MB4) {  // mailbox 4 event
    mailbox_int_handler(CAN1, 4);
  }
  if (ul_status & CAN_SR_MB5) {  // mailbox 5 event
    mailbox_int_handler(CAN1, 5);
  }
  if (ul_status & CAN_SR_MB6) {  // mailbox 6 event
    mailbox_int_handler(CAN1, 6);
  }
  if (ul_status & CAN_SR_MB7) {  // mailbox 7 event
    mailbox_int_handler(CAN1, 7);
  }

  // runs on second frame bit at ~70%
  if ((ul_status & CAN_SR_TSTP) && !sending_hammered_frame) {
    writing_message_to_ecu = checking_for_can1_sof;

    if (checking_for_can1_sof) {
      // start separate timer for CANTX; fires instantly
      startTimer(TC0, 1, TC1_IRQn, SPEED * 1000);

      // turn on multiplexing
      PIOA->PIO_PER = PIO_PA0A_CANTX0;

      // set replicate_frame_queue
      replicate_frame_value = 0;
      CAN0_Write(0);
    } else {
      stopTimer(TC0, 1, TC1_IRQn);
      PIOA->PIO_PDR = PIO_PA0A_CANTX0;
    }

    can_set_timestamp_capture_point(CAN1, (checking_for_can1_sof ? 1 : 0));
    checking_for_can1_sof = !checking_for_can1_sof;
  }
}

// samples bits to determine when data bits start
void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  CAN1_Write(frame_value);
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

    if (frame_bits == 15 && (frame_value & 0b111) != 0) {
      // TODO: figure out why stopTimer fails to work here
      // stopTimer(TC0, 0, TC0_IRQn);
    } else if (frame_bits == 19) {
      if ((frame_queue & 0b1111) != 0) {
        data_length = frame_queue & 0b1111;
        frame_data.val = 0;
      } else {
        data_length = 0;
        // TODO: figure out why stopTimer fails to work here
        // stopTimer(TC0, 0, TC0_IRQn);
      }
    } else if (data_length != 0 && frame_bits > 19 && frame_bits <= 19 + data_length * 8) {
      frame_data.val = (frame_data.val << 1) | frame_value;

      if (frame_bits == 19 + data_length * 8) {
        // hammer_data[0] = 4;
        // hmac_sha256(key, sizeof(key) - 1, frame_data.arr, data_length, hammer_data, 32);
        startTimer(TC1, 0, TC3_IRQn, (uint32_t)(SPEED * 1000 * 5));
      }
    }
  }
}

// used to copy values to CAN0 from CAN1 by sampling CAN1
void TC1_Handler() {
  TC_GetStatus(TC0, 1);

  CAN0_Write(replicate_frame_value);
  replicate_frame_value = PIOB->PIO_PDSR & PIO_PB15A_CANRX1;
}

// runs hammering at ~20% for first data bit; helps synchronize later hammering
void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  if (hammer_index < HAMMER_BIT_COUNT) {
    // starts TC4 at next data bit (~0% mark)
    startTimer(TC1, 1, TC4_IRQn, SPEED * 1000);

    reset_value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frame_bit_hammered = false;

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
    reset_value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frame_bit_hammered = false;

    // start TC6 to fire 5 times in a bit (~20% mark)
    startTimer(TC2, 0, TC6_IRQn, SPEED * 1000 * 5);
  } else {
    // stop timer when no more bits to hammer
    stopTimer(TC1, 1, TC4_IRQn);
  }
}

// handles hammering bits for one data bit
void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  // write first hammered bit for data bit
  CAN1_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
  hammer_index++;

  frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;

  // start timer to hammer remaining bits for data bit
  startTimer(TC2, 1, TC7_IRQn, SPEED * 1000 * 10);

  stopTimer(TC2, 0, TC6_IRQn);
}

void TC7_Handler() {
  TC_GetStatus(TC2, 1);

  if (!frame_bit_hammered) {
    // if bit in frame has not been completely hammered, continue with hammering
    CAN1_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
    hammer_index++;

    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    // reset value
    CAN1_Write(reset_value);

    // stop TC7 (stops hammering for the data bit)
    stopTimer(TC2, 1, TC7_IRQn);
  }
}

void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  // disable write protect of PMC registers.
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
  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of LED
  PIOB->PIO_OER = PIO_PB27;
  PIOB->PIO_PER = PIO_PB27;

  // enable control of digital pin 22
  PIOB->PIO_OER = PIO_PB26;
  PIOB->PIO_PER = PIO_PB26;

  // multiplex CANTX1
  PIOB->PIO_OER = PIO_PB14A_CANTX1;
  PIOB->PIO_PDR = PIO_PB14A_CANTX1;
  PIOB->PIO_PUDR = PIO_PB14A_CANTX1;

  // init CAN0
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, SystemCoreClock, SPEED);

  // enable only timestamp interrupt and set to fire at SOF
  can_disable_interrupt(CAN0, CAN_DISABLE_ALL_INTERRUPT_MASK);
  can_enable_interrupt(CAN0, CAN_SR_TSTP);
  can_set_timestamp_capture_point(CAN0, 0);

  // enable CAN0 interrupt handler
  NVIC_EnableIRQ(CAN0_IRQn);

  // init RX0 boxes
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

  // init TX0 boxes
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

  // init CAN1
  pmc_enable_periph_clk(ID_CAN1);
  can_init(CAN1, SystemCoreClock, SPEED);

  // enable only timestamp interrupt and set to fire at SOF
  can_disable_interrupt(CAN1, CAN_DISABLE_ALL_INTERRUPT_MASK);
  can_enable_interrupt(CAN1, CAN_SR_TSTP);
  can_set_timestamp_capture_point(CAN1, 0);

  // enable CAN1 interrupt handler
  NVIC_EnableIRQ(CAN1_IRQn);

  // init RX1 boxes
  for (c = 0; c < 8 - 1; c++) {
    // set mode to CAN_MB_RX_MODE
    CAN1->CAN_MB[c].CAN_MMR = (CAN1->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_RX_MODE << CAN_MMR_MOT_Pos);

    // set id to 0x0
    CAN1->CAN_MB[c].CAN_MID = CAN_MID_MIDvA(0x0);

    // set accept mask to 0x7ff
    CAN1->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN1->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  // init TX1 boxes
  for (; c < 8; c++) {
    // set mode to CAN_MB_TX_MODE
    CAN1->CAN_MB[c].CAN_MMR = (CAN1->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_TX_MODE << CAN_MMR_MOT_Pos);

    // set priority to 10
    CAN1->CAN_MB[c].CAN_MMR = (CAN1->CAN_MB[c].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (10 << CAN_MMR_PRIOR_Pos);

    // set accept mask to 0x7ff
    CAN1->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN1->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  // set up mailbox 0 for standard IDs
  CAN1->CAN_MB[0].CAN_MAM = CAN_MAM_MIDvA(0);
  CAN1->CAN_MB[0].CAN_MID &= ~CAN_MAM_MIDE;
  CAN1->CAN_MB[0].CAN_MID = CAN_MID_MIDvA(0);
  can_enable_interrupt(CAN1, CAN_IER_MB0);
}

void loop() {
}
