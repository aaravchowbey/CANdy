#include "variant.h"
#include <due_can.h>

#define Serial SerialUSB

#define SPEED        CAN_BPS_250K
#define HAMMER_POINT 84.0e-6
#define HAMMER_SIZE 2

#define LED_HIGH() do { PIOB->PIO_SODR = PIO_PB27; ledState = true; } while(0)
#define LED_LOW()  do { PIOB->PIO_CODR = PIO_PB27; ledState = false; } while(0)
#define LED_FLIP() do { if (ledState) LED_LOW(); else LED_HIGH(); } while(0)

volatile bool ledState = false;
volatile bool sof = false;
volatile bool resetValue = false;

unsigned short int bus_queue = 0b111111111111111;
unsigned long int frame_queue = 0;

const bool hammerBits[HAMMER_SIZE] = {1, 0};
volatile int hammerIndex = 0;

void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  bool value = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0);

  bus_queue = (bus_queue << 1 | value) & 0b111111111111111;

  if (bus_queue == 0b111111111111110) {
    sof = true;

    // startTimer(TC2, 0, TC6_IRQn, (uint32_t)(1 / HAMMER_POINT));
  }

  if (sof) {
    if (!(value == 0 && (bus_queue & 0b11111) == 0b11111) &&
        !(value == 1 && (bus_queue & 0b11111) == 0b00000)) {
      frame_queue = frame_queue << 1 | value;
    }

    if ((frame_queue & (1 << 0)) == 0 &&            // start-of-frame
        (frame_queue & (1 << 12)) == 0 &&           // RTR (data frame = 0)
        (frame_queue & (1 << 13)) == 0 &&           // IDE
        (frame_queue & (1 << 14)) == 0 &&           // reserved bit
        (frame_queue & 0b1111000000000000000) != 0  // DLC (> 0)
    ) {
      frame_queue = 0;
      sof = false;

      CANdy_Hammer();
    }
  }
}

void CANdy_Hammer() {
  resetValue = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;

  // start hammering
  hammerIndex = 0;

  LED_FLIP();
  PIOA->PIO_PER = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  startTimer(TC2, 1, TC7_IRQn, 1 / 2.0e-6);

  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_ODR = PIO_PA0A_CANTX0;
}

void TC6_Handler() {
  TC_GetStatus(TC2, 0);

  if (!sof) {
    stopTimer(TC2, 0, TC6_IRQn);

    return;
  }

  sof = false;

  resetValue = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;

  // start hammering
  hammerIndex = 0;

  LED_FLIP();
  PIOA->PIO_PER = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  startTimer(TC2, 1, TC7_IRQn, 4000000);

  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  PIOA->PIO_ODR = PIO_PA0A_CANTX0;
}

void TC7_Handler() {
  TC_GetStatus(TC2, 1);

  if (hammerIndex < HAMMER_SIZE) {
    CANdy_Write(hammerBits[hammerIndex]);
    hammerIndex++;
  } else {
    stopTimer(TC2, 1, TC7_IRQn);
    CANdy_Write(resetValue);

    LED_FLIP();
  }
}

void CANdy_Write(bool value) {
  if (value) {
    PIOA->PIO_SODR = PIO_PA0A_CANTX0;
  } else {
    PIOA->PIO_CODR = PIO_PA0A_CANTX0;
  }

  // // turn off multiplexing
  // PIOA->PIO_PDR = PIO_PA0A_CANTX0;
  // PIOA->PIO_ODR = PIO_PA0A_CANTX0;
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

void TC8_Handler() {

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

  startTimer(TC1, 0, TC3_IRQn, SPEED);
}

void loop() {
}
