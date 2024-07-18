// TODO: slight shift of hammering from last few data bytes to CRC
#define Serial SerialUSB

#define SPEED CAN_BPS_50K

#define HAMMER_BIT_COUNT 8        // total number of bits to hammer for message frame
#define HAMMER_SIZE 1             // number of bits to hammer per data bit
#define HAMMER_START 20.f         // percentage
#define HAMMER_BIT_WIDTH 30.f     // percentage

#define FRAME_DELAY_AMOUNT 29.0f  // percentage

#define CPU_CLOCK_SPEED 108

#define ROUND(num) ((uint32_t)((num) + 0.5f))

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

const uint32_t speed_freq = SPEED * 1000;
const uint32_t frame_sync_delayed_freq = SPEED * 1000 * (100.f / (HAMMER_START + FRAME_DELAY_AMOUNT));
const uint32_t hammer_freq = SPEED * 1000 * (100.f / HAMMER_BIT_WIDTH);

volatile bool hammer_state = false;

volatile bool frame_value = false;
volatile bool old_frame_value = false;
volatile bool reset_value = false;

volatile uint8_t frame_queue = 0;
volatile uint8_t bits_after_prev_stuff = 0, frame_bits = 0;

// data bits to hammer
uint8_t hammer_data[32] = { 0xff, 0, 0, 0, 0 };
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

// union {
//   volatile uint64_t val;
//   uint8_t arr[8];
// } frame_data;

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

  // runs on second frame bit at ~70%
  if ((ul_status & CAN_SR_TSTP) && !writing_message_to_ecu) {
    sending_hammered_frame = checking_for_sof;

    if (checking_for_sof) {
      // store value initially!
      frame_value = (bool)(PIOA->PIO_PDSR & PIO_PA1A_CANRX0);
      frame_queue = 0b11111100 | (uint8_t)(frame_value ? 1 : 0);

      startTimer(TC0, 0, TC0_IRQn, speed_freq, TC_CMR_TCCLKS_TIMER_CLOCK2);
      // NVIC_SetPriority(TC0_IRQn, -1);

      // write start-of-frame bit
      PIOB->PIO_PER = PIO_PB14A_CANTX1;
      CAN1_Write(0);

      // reset values
      bits_after_prev_stuff = 1;
      frame_bits = 1;
      hammer_index = 0;
      can_set_timestamp_capture_point(CAN0, 1);
    } else {
      stopTimer(TC0, 0, TC0_IRQn);
      PIOB->PIO_PDR = PIO_PB14A_CANTX1;
      PIN_WRITE(0);
      can_set_timestamp_capture_point(CAN0, 0);
    }

    checking_for_sof = !checking_for_sof;
  }
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
}

// samples bits to determine when data bits start
void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  CAN1_Write(frame_value);
  old_frame_value = frame_value;
  // PIN_WRITE(frame_bits % 2);

  frame_value  = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
  bool nonvol_frame_value = frame_value;
  if (bits_after_prev_stuff >= 5 && ((nonvol_frame_value && ((frame_queue & 0x1F) == 0)) || (!nonvol_frame_value && ((frame_queue & 0x1F) == 0x1F)))) {
    // if stuff bit, reset frame_samples
    bits_after_prev_stuff = 0;
  } else {
    // if not stuff bit, add to frame_queue
    frame_queue = ((frame_queue & 0b1111111) << 1) | (uint8_t)nonvol_frame_value;

    // increment counters
    bits_after_prev_stuff++;
    frame_bits++;

    // if (frame_bits == 15 && (frame_value & 0b111) != 0) {
    //   // TODO: figure out why stopTimer fails to work here
    //   // stopTimer(TC0, 0, TC0_IRQn);
    // } else if (frame_bits == 19) {
    //   // FIX: storing data length results in error!
    //   // data_length = frame_queue & 0b1111;
    //   // if (data_length == 0) {
    //   //   stopTimer(TC0, 0, TC0_IRQn);
    //   // }
    // } else
    if (frame_bits >= 20 && frame_bits < 20 + data_length * 8) {
      // if (frame_bits == 20) {
      //   frame_data.val = 0;
      // }
      // uint8_t frame_data_ind = frame_bits - 20;
      // frame_data.arr[frame_data_ind / 8] |= (frame_data_ind % 8) << (7 - frame_data_ind);
      // frame_data.val = (frame_data.val << 1) | (uint8_t)frame_val;

      if (frame_bits == 20 + data_length * 8 - 1) {
        // hammer_data[0] = 4;
        // hmac_sha256(key, sizeof(key) - 1, frame_data.arr, data_length, hammer_data, 32);

        // start one bit after!
        startTimer(TC1, 0, TC3_IRQn, frame_sync_delayed_freq, TC_CMR_TCCLKS_TIMER_CLOCK1);
        NVIC_SetPriority(TC3_IRQn, 0);
      }
    }
  }
}

// runs hammering at ~20% for first data bit; helps synchronize later hammering
void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  startTimer(TC1, 1, TC4_IRQn, speed_freq, TC_CMR_TCCLKS_TIMER_CLOCK2);
  // PIN_WRITE(1);

  stopTimer(TC1, 0, TC3_IRQn);
}

// handles hammering bits for all data bits
void TC4_Handler() {
  TC_GetStatus(TC1, 1);

  if (hammer_index < HAMMER_BIT_COUNT) {
    // start TC6 to fire 5 times in a bit (~20% mark)
    // startTimer(TC2, 0, TC6_IRQn, speed_freq * 3, TC_CMR_TCCLKS_TIMER_CLOCK2);

    // PIN_WRITE(hammer_index % 2);
    // CAN1_Write(1);
    CAN1_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
    hammer_index++;
    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    // stop timer when no more bits to hammer
    stopTimer(TC1, 1, TC4_IRQn);
  }
}

void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  // if (!frame_bit_hammered) {
  //   // if bit in frame has not been completely hammered, continue with hammering
  //   CAN1_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
  //   // CAN1_Write(true);
  //   hammer_index++;
  //   frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  // } else {
  //   // reset value
    // CAN1_Write(0);
    CAN1_Write(old_frame_value);

    // stop TC6 (stops hammering for the data bit)
    stopTimer(TC2, 0, TC6_IRQn);
  // }
}

void CAN1_Handler() {
  uint32_t ul_status = CAN1->CAN_SR;  // get status of interrupts

  // runs on second frame bit at ~70%
  // if ((ul_status & CAN_SR_TSTP) && !sending_hammered_frame) {
  //   writing_message_to_ecu = checking_for_can1_sof;
  //
  //   if (checking_for_can1_sof) {
  //     // start separate timer for CANTX; fires instantly
  //     startTimer(TC2, 2, TC8_IRQn, speed_freq, TC_CMR_TCCLKS_TIMER_CLOCK2);
  //
  //     // turn on multiplexing
  //     PIOA->PIO_PER = PIO_PA0A_CANTX0;
  //
  //     // set replicate_frame_queue
  //     replicate_frame_value = 0;
  //     CAN0_Write(0);
  //     can_set_timestamp_capture_point(CAN1, 1);
  //   } else {
  //     stopTimer(TC2, 2, TC8_IRQn);
  //     PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  //     can_set_timestamp_capture_point(CAN1, 0);
  //   }
  //
  //   checking_for_can1_sof = !checking_for_can1_sof;
  // }
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
}

// used to copy values to CAN0 from CAN1 by sampling CAN1
void TC8_Handler() {
  TC_GetStatus(TC2, 2);

  CAN0_Write(replicate_frame_value);
  replicate_frame_value = PIOB->PIO_PDSR & PIO_PB15A_CANRX1;
}

void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency, uint32_t clock) {
  // disable write protect of PMC registers.
  pmc_set_writeprotect(false);
  // enable the specified peripheral clock.
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR | clock);
  uint32_t rc = (uint32_t)((float)CPU_CLOCK_SPEED * 1e6 / (1 << (2 * clock + 1)) / (float)frequency);

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

  uint8_t cpuMult = CPU_CLOCK_SPEED / 6 - 1;
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4);
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4);
  PMC->CKGR_PLLAR = (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(cpuMult) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(1UL));
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) {}
  PMC->PMC_MCKR = ( PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK);
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {}
  SystemCoreClockUpdate();

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
