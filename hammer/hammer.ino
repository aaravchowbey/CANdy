#include "variant.h"
#include <due_can.h>

#define Serial SerialUSB
#define HAMMER_POINT 85e-6
#define SPEED CAN_BPS_250K

#define LED_HIGH() do { PIOB->PIO_SODR = PIO_PB27; ledState = true; } while(0)
#define LED_LOW()  do { PIOB->PIO_CODR = PIO_PB27; ledState = false; } while(0)
#define LED_FLIP() do { if (ledState) LED_LOW(); else LED_HIGH(); } while(0)

volatile bool ledState = false;
volatile bool interruptAdded = false;

volatile bool sof = false;

const int hammerSize = 4;
const int hammerBits[hammerSize] = {1, 1, 1, 1};
int hammerIndex = 0;

void setup() {
  Serial.begin(115200);

  Can0.begin(SPEED);
  Can1.begin(SPEED);

  PMC->PMC_PCER0 |= PMC_PCER0_PID11; // PIOA power ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID12; // PIOB power ON

  // multiplex LED and enable output
  PIOB->PIO_PER = PIO_PB27;
  PIOB->PIO_OER = PIO_PB27;

  // set CAN_RX as input (Output Disable Register) and
  PIOA->PIO_PER = PIO_PA1A_CANRX0;
  PIOA->PIO_OER = PIO_PA1A_CANRX0;

  // disable pull-up on both pins (Pull Up Disable Register)
  PIOA->PIO_PUDR = PIO_PA1A_CANRX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  LED_HIGH();
  delay(500);
  LED_LOW();
  delay(1000);

  Can0.watchFor();

  // attachInterrupt(PIO_PA1A_CANRX0, CANdy_Sync, FALLING);
}

void startTimer(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) { //  DO NOT TOUCH
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

/**
 * Synchronizes hammering with message by starting timer for hammering; runs TC3_Handler
 */
void CANdy_Sync() {
  // used to check if CANdy_Sync is running
  LED_HIGH();

  detachInterrupt(PIO_PA1A_CANRX0);

  if (sof) {
    return;
  }

  sof = true;

  // NVIC_DisableIRQ(TC3_IRQn);

  // TC3_Handler will run after time specified in HAMMER_POINT (middle of first bit in data bytes)
  // startTimer(TC1, 0, TC3_IRQn, 1 / HAMMER_POINT);

  delayMicroseconds(100000);
  sof = false;
  interruptAdded = false;

  LED_LOW();
}

/**
  * Samples value of bit being sent, starts CANdy_Hammer, and resets value of bit
  */
void TC3_Handler() {
  if (!sof) {
    // disable IRQn for TC3 to "stop" timer
    NVIC_DisableIRQ(TC3_IRQn);
    return;
  }

  TC_GetStatus(TC1, 0);

  // PIO_PA1A_CANRX0 = If there is a bit
  // PIO_PDSR = Time efficency (digial inputs)
  bool value = PIOA->PIO_PDSR & PIO_PA1A_CANRX0; // We read/sample the bit here (changed to CANRX instead of CANTX)

  // sof = Start of Frame (goes through each frame) (Should be 0)
  // Stuff bit = If there is a row of 5 consistant bits, then add a stuff bit of the opposite bit
  // Stuff Bit contains no useful information!
  // 0 0 0 0 0 1 (STUFF BIT) 0 0...  (+2 ms) (Each bit is 2 ms)
  // No stuff bits after Acknowledgement bit!

  // disable output of CANTX
  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_ODR = PIO_PA0A_CANTX0;

  attachInterrupt(40, CANdy_Hammer, HIGH);
  digitalWrite(40, HIGH);

  CANdy_Write(value);

  // turn off multiplexing
  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_ODR = PIO_PA0A_CANTX0;
}

void CANdy_Hammer() { // Now every bit time, we will run TC3_Handler
  NVIC_DisableIRQ(TC6_IRQn);

  // This allows us to sample exactly at 20% (SAMPLING_POINT == 5) of the bit time. This is where the ringing will be done.
  startTimer(TC2, 0, TC6_IRQn, hammerSize * SPEED);
  hammerIndex = 0;
  detachInterrupt(40);
}

void CANdy_Write(bool value) {
  // multiplex CANTX to GPIO and define as output
  PIOA->PIO_PER = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  // TODO: check if this is meant to be PIOC or PIOA
  (value ? PIOA->PIO_SODR : PIOA->PIO_CODR) = PIO_PA0A_CANTX0;

  // turn off multiplexing
  // PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  // PIOA->PIO_ODR = PIO_PA0A_CANTX0;
}

void TC6_Handler() {
  // turn off multiplexing
  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_ODR = PIO_PA0A_CANTX0;

  TC_GetStatus(TC2, 0);

  if (hammerIndex < hammerSize) {
    CANdy_Write(hammerBits[hammerIndex]);
    hammerIndex++;
  } else {
    hammerIndex = 0;
    NVIC_DisableIRQ(TC6_IRQn);
  }
}

void loop() {
  // attach multiple interrupts may lead to problem??? <https://forum.arduino.cc/t/attaching-interrupt-in-loop/1031388/3>
  bool locSof, locInterruptAdded;
  noInterrupts();
  locSof = sof;
  locInterruptAdded = interruptAdded;
  if (!sof && !interruptAdded) {
    interruptAdded = true;
  }
  interrupts();

  if (!locSof && !locInterruptAdded) {
    attachInterrupt(PIO_PA1A_CANRX0, CANdy_Sync, LOW);
  }
}
