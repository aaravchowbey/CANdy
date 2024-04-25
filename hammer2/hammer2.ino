#include "variant.h"
#include <due_can.h>

#define Serial SerialUSB

#define SPEED        CAN_BPS_250K
#define HAMMER_POINT 84.6e-6
#define HAMMER_SIZE 2

#define LED_HIGH() do { PIOB->PIO_SODR = PIO_PB27; ledState = true; } while(0)
#define LED_LOW()  do { PIOB->PIO_CODR = PIO_PB27; ledState = false; } while(0)
#define LED_FLIP() do { if (ledState) LED_LOW(); else LED_HIGH(); } while(0)

volatile bool ledState = false;
volatile bool sof = false;
volatile bool resetValue = false;

unsigned short int queue = 0b111111111111111;

const bool hammerBits[HAMMER_SIZE] = {1, 0};
volatile int hammerIndex = 0;

void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  queue = queue << 1 | (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) & 0b111111111111111;

  if (queue == 0b111111111111110) {
    sof = true;
    NVIC_DisableIRQ(TC6_IRQn);
    startTimer(TC2, 0, TC6_IRQn, 1 / HAMMER_POINT);
  }
}

void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  if (!sof) {
    stopTimer(TC6_IRQn);

    return;
  }

  sof = false;

  resetValue = PIOA->PIO_PDSR & PIO_PA1A_CANRX0;

  // start hammering
  hammerIndex = 0;

  PIOA->PIO_PER = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  NVIC_DisableIRQ(TC7_IRQn);
  startTimer(TC2, 1, TC7_IRQn, HAMMER_SIZE * SPEED);
}

void TC7_Handler() {
  TC_GetStatus(TC2, 1);

  if (hammerIndex < HAMMER_SIZE) {
    CANdy_Write(hammerBits[hammerIndex]);
    hammerIndex++;
  } else {
    CANdy_Write(resetValue);

    // turn off multiplexing
    PIOA->PIO_PDR = PIO_PA0A_CANTX0;
    PIOA->PIO_ODR = PIO_PA0A_CANTX0;

    LED_FLIP();

    stopTimer(TC7_IRQn);
  }
}

void CANdy_Write(bool value) {
  // TODO: fix this!!!
  if (value) {
    PIOA->PIO_SODR = PIO_PA0A_CANTX0;
  } else {
    PIOA->PIO_CODR = PIO_PA0A_CANTX0;
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

void stopTimer(IRQn_Type irq) {
	NVIC_DisableIRQ(irq);
	pmc_disable_periph_clk((uint32_t)irq);
}

void setup() {
  // start serial port at 115200 bps:
  Serial.begin(115200);

  // Verify CAN0 and CAN1 initialization, baudrate is 1Mb/s:
  Can0.begin(SPEED);
  Can1.begin(SPEED);
 
  PMC->PMC_PCER0 |= PMC_PCER0_PID11; // PIOA power ON

  //Multiplex CAN_RX to GPIO (Peripheral Enable Register)
  PIOA->PIO_PER = PIO_PA1A_CANRX0;

  //Set CAN_RX as input (Ouput Disable Register)
  PIOA->PIO_ODR = PIO_PA1A_CANRX0;

  //Disable pull-up on both pins (Pull Up Disable Register)
  PIOA->PIO_PUDR = PIO_PA1A_CANRX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  // enable control of LEDs
  PMC->PMC_PCER0 |= PMC_PCER0_PID12; // PIOB power ON
  PIOB->PIO_PER = PIO_PB27;
  PIOB->PIO_OER = PIO_PB27;

  LED_HIGH();
  delay(500);
  LED_LOW();
  delay(1000);

  Can0.watchFor();

  NVIC_DisableIRQ(TC3_IRQn);
  startTimer(TC1, 0, TC3_IRQn, SPEED);
}

void loop() {
}
