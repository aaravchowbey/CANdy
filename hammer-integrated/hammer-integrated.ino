#include <hmac_sha256.h>

#define Serial SerialUSB

#define HAMMER_BIT_COUNT 5  // total number of bits to hammer for message frame
#define HAMMER_SIZE 1       // number of bits to hammer per data bit

#define SPEED CAN_BPS_50K

#define CANdy_Write(value) \
  do { \
    (value) ? (PIOA->PIO_SODR = PIO_PA0A_CANTX0) : (PIOA->PIO_CODR = PIO_PA0A_CANTX0); \
  } while (0)

// data bits to hammer
uint8_t hammer_data[32] = { 0b11111111, 0, 0, 0, 0 };
const unsigned char key[] = "super-secret-key";

// current index of hammer_data
volatile uint8_t hammer_index = 0;

// bit in frame has been completely hammered
volatile bool frame_bit_hammered = false;

volatile bool reset_value = false;

static unsigned long last_time = 0;

uint8_t total_bits;
bool is_frame_processed;

#define DATA_LENGTH 8
const uint8_t data[DATA_LENGTH] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };

struct {
  uint32_t id;
  uint8_t length;
  union {
    uint64_t value;
    uint8_t bytes[8];
  } data;
} outgoing;

void mailbox_int_handler(uint8_t mb) {
  if (CAN0->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) {       //mailbox signals it is ready
    switch (((CAN0->CAN_MB[mb].CAN_MMR >> 24) & 7)) {  //what sort of mailbox is it?
      case 1:                                          //receive
      case 2:                                          //receive w/ overwrite
      case 4:                                          //consumer - technically still a receive buffer
        can_mailbox_send_transfer_cmd(CAN0, mb);
        break;
      case 3:  //transmit
        can_disable_interrupt(CAN0, 0x01u << mb);
        break;
      case 5:  //producer - technically still a transmit buffer
        break;
    }
  }
}

void CAN0_Handler() {
  uint32_t ul_status = CAN0->CAN_SR;  //get status of interrupts

  if (ul_status & CAN_SR_MB0) {  //mailbox 0 event
    mailbox_int_handler(0);
  }
  if (ul_status & CAN_SR_MB1) {  //mailbox 1 event
    mailbox_int_handler(1);
  }
  if (ul_status & CAN_SR_MB2) {  //mailbox 2 event
    mailbox_int_handler(2);
  }
  if (ul_status & CAN_SR_MB3) {  //mailbox 3 event
    mailbox_int_handler(3);
  }
  if (ul_status & CAN_SR_MB4) {  //mailbox 4 event
    mailbox_int_handler(4);
  }
  if (ul_status & CAN_SR_MB5) {  //mailbox 5 event
    mailbox_int_handler(5);
  }
  if (ul_status & CAN_SR_MB6) {  //mailbox 6 event
    mailbox_int_handler(6);
  }
  if (ul_status & CAN_SR_MB7) {  //mailbox 7 event
    mailbox_int_handler(7);
  }
  if (ul_status & CAN_SR_ERRA) {  //error active
  }
  if (ul_status & CAN_SR_WARN) {  //warning limit
  }
  if (ul_status & CAN_SR_ERRP) {  //error passive
  }
  if (ul_status & CAN_SR_BOFF) {  //bus off
  }
  if (ul_status & CAN_SR_SLEEP) {  //controller in sleep mode
  }
  if (ul_status & CAN_SR_WAKEUP) {  //controller woke up
  }
  if (ul_status & CAN_SR_TOVF) {  //timer overflow
  }
  if (ul_status & CAN_SR_TSTP) {  //timestamp - start or end of frame
    if (!is_frame_processed) {
      startTimer(TC1, 0, TC3_IRQn, (uint32_t)(SPEED * 1000 / (total_bits - 0.8)));
    }
  }
  if (ul_status & CAN_SR_CERR) {  //CRC error in mailbox
  }
  if (ul_status & CAN_SR_SERR) {  //stuffing error in mailbox
  }
  if (ul_status & CAN_SR_AERR) {  //ack error
  }
  if (ul_status & CAN_SR_FERR) {  //form error
  }
  if (ul_status & CAN_SR_BERR) {  //bit error
  }
}

// runs hammering at ~20% for first data bit; helps synchronize later hammering
void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  is_frame_processed = true;
  hammer_index = 0;
  if (hammer_index < HAMMER_BIT_COUNT) {
    // starts TC4 at next data bit (~0% mark)
    startTimer(TC1, 1, TC4_IRQn, SPEED * 1000);

    // turn on multiplexing
    PIOA->PIO_PER = PIO_PA0A_CANTX0;

    reset_value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;
    frame_bit_hammered = false;

    // start TC6 to fire at ~20% mark
    startTimer(TC2, 0, TC6_IRQn, SPEED * 1000 * 5);
  }

  // stops first data bit hammering timer
  stopTimer(TC1, 0, TC3_IRQn);
}

// handles hammering bits for all other data bits
void TC4_Handler() {
  TC_GetStatus(TC1, 1);

  if (hammer_index < HAMMER_BIT_COUNT) {
    // if bits left to hammer, setup hammering for one data bit
    reset_value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

    // start TC6 to fire at ~20% mark
    startTimer(TC2, 0, TC6_IRQn, SPEED * 1000 * 5);
  } else {
    // stop timer when no more bits to hammer
    stopTimer(TC1, 1, TC4_IRQn);
  }
}

// handles hammering one bit and sets up hammering for one data bit
void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  // turn on multiplexing
  PIOA->PIO_PER = PIO_PA0A_CANTX0;

  // write first hammered bit for data bit
  CANdy_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
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
    CANdy_Write((hammer_data[hammer_index / 8] >> (hammer_index % 8)) & 1);
    hammer_index++;

    frame_bit_hammered = hammer_index % HAMMER_SIZE == 0 || hammer_index == HAMMER_BIT_COUNT;
  } else {
    // reset value
    CANdy_Write(reset_value);

    // stop TC7 (stops hammering for the data bit)
    stopTimer(TC2, 1, TC7_IRQn);

    // turn off multiplexing
    PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  }
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
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize CAN0
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, SystemCoreClock, SPEED);
  can_disable_interrupt(CAN0, CAN_DISABLE_ALL_INTERRUPT_MASK);
  can_enable_interrupt(CAN0, CAN_SR_TSTP);
  can_set_timestamp_capture_point(CAN0, 0);

  // init RX boxen
  int c;
  for (c = 0; c < 8 - 1; c++) {
    // set mode to CAN_MB_RX_MODE
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_RX_MODE << CAN_MMR_MOT_Pos);

    // set id to 0x0
    CAN0->CAN_MB[c].CAN_MID = CAN_MID_MIDvA(0x0);

    // set accept mask to 0x7ff
    CAN0->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN0->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  //Initialize TX boxen
  for (c = 8 - 1; c < 8; c++) {
    // set mode to CAN_MB_TX_MODE
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_MOT_Msk) | (CAN_MB_TX_MODE << CAN_MMR_MOT_Pos);

    // set priority to 10
    CAN0->CAN_MB[c].CAN_MMR = (CAN0->CAN_MB[c].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (10 << CAN_MMR_PRIOR_Pos);

    // set accept mask to 0x7ff
    CAN0->CAN_MB[c].CAN_MAM = CAN_MAM_MIDvA(0x7ff);
    CAN0->CAN_MB[c].CAN_MID &= ~CAN_MAM_MIDE;
  }

  // enable CAN0 interrupt handler
  NVIC_SetPriority(CAN0_IRQn, 12);
  NVIC_EnableIRQ(CAN0_IRQn);

  PMC->PMC_PCER0 |= PMC_PCER0_PID11;  // PIOA power ON

  PIOA->PIO_PDR = PIO_PA0A_CANTX0;  // unable to send data w/ PER so changed to PDR (2024-03-13)
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // PIOB power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;

  // enable control of digital pin 22
  PIOB->PIO_PER = PIO_PB26;
  PIOB->PIO_OER = PIO_PB26;

  // sets up mailbox 0 for standard IDs
  CAN0->CAN_MB[0].CAN_MAM = CAN_MAM_MIDvA(0);
  CAN0->CAN_MB[0].CAN_MID &= ~CAN_MAM_MIDE;
  CAN0->CAN_MB[0].CAN_MID = CAN_MID_MIDvA(0);
  can_enable_interrupt(CAN0, CAN_IER_MB0);

  outgoing.id = 0x400;
  outgoing.length = DATA_LENGTH;

  for (int i = 0; i < DATA_LENGTH; i++) {
    outgoing.data.bytes[i] = data[i];
  }
}

void sendFrame() {
  is_frame_processed = false;
  for (int mb = 0; mb < 8; mb++) {
    if (((CAN0->CAN_MB[mb].CAN_MMR >> 24) & 7) == CAN_MB_TX_MODE) {  //is this mailbox set up as a TX box?
      if (CAN0->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) {                 //is it also available (not sending anything?)
        // set id to outgoing frame's id
        CAN0->CAN_MB[mb].CAN_MID = CAN_MID_MIDvA(outgoing.id);

        // set data length to outgoing frame's length
        CAN0->CAN_MB[mb].CAN_MCR = (CAN0->CAN_MB[mb].CAN_MCR & ~CAN_MCR_MDLC_Msk) | CAN_MCR_MDLC(outgoing.length);

        // set priority to 4
        CAN0->CAN_MB[mb].CAN_MMR = (CAN0->CAN_MB[mb].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (4 << CAN_MMR_PRIOR_Pos);

        // set data bytes
        CAN0->CAN_MB[mb].CAN_MDL = (uint32_t)(outgoing.data.value & 0xffffffff);
        CAN0->CAN_MB[mb].CAN_MDH = (uint32_t)(outgoing.data.value >> 32);

        // calculate bits in first part of frame
        total_bits = 17;
        uint32_t frame_head = outgoing.length | (uint32_t)(outgoing.id << 7);  // 0b0[ID]000[DLC]

        for (int8_t i = 14; i >= 0; i--) {
          if ((frame_head & (0b11111 << i)) == 0) {
            total_bits++;
            i -= 4;
          }
        }

        hmac_sha256(key, sizeof(key) - 1, outgoing.data.bytes, outgoing.length, hammer_data, 32);

        can_enable_interrupt(CAN0, 0x01u << mb);  //enable the TX interrupt for this box
        can_global_send_transfer_cmd(CAN0, mb);

        // message sent
        return;
      }
    }
  }

  // TODO: reimplement queue for sending messages
  // //if execution got to this point then no free mailbox was found above
  // //so, queue the frame if possible. But, don't increment the
  // //tail if it would smash into the head and kill the queue.
  // uint8_t temp;
  // temp = (tx_buffer_tail + 1) % SIZE_TX_BUFFER;
  // if (temp == tx_buffer_head) return false;
  // tx_frame_buff[tx_buffer_tail].id = txFrame.id;
  // tx_frame_buff[tx_buffer_tail].extended = txFrame.extended;
  // tx_frame_buff[tx_buffer_tail].length = txFrame.length;
  // tx_frame_buff[tx_buffer_tail].data.value = txFrame.data.value;
  // tx_buffer_tail = temp;
}


void loop() {
  if (millis() - last_time > 1000) {
    last_time = millis();
    sendFrame();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
