#include "variant.h"
#include <due_can.h>

#define Serial SerialUSB

#define SPEED              CAN_BPS_50K
#define HAMMER_BIT_COUNT   2    //total number of bits to hammer for frame
#define HAMMER_SIZE        2    //number of bits to hammer per bit 
#define SAMPLING_RATE      10   

#define LED_HIGH() do { PIOB->PIO_SODR = PIO_PB27; ledState = true; } while(0)
#define LED_LOW()  do { PIOB->PIO_CODR = PIO_PB27; ledState = false; } while(0)
#define LED_FLIP() do { if (ledState) LED_LOW(); else LED_HIGH(); } while(0)

#define ONE_BITS(x) ((1UL << (x)) - 1)
#define BITS_AT_POS(queue, pos, num_bits)                                 \
  (queue[pos * SAMPLING_RATE / 64] &                                      \
   (ONE_BITS(num_bits * SAMPLING_RATE) << (SAMPLING_RATE * pos % 64)))

volatile bool ledState = false;
volatile bool sof = false;
volatile bool resetValue = false;

uint64_t bus_queue[3] = {UINT64_MAX, UINT64_MAX, UINT64_MAX};
uint64_t frame_queue[3] = {0};

const bool hammerData[HAMMER_BIT_COUNT] = {0, 1};
// const int hammerBits = 0b10;
volatile int hammerIndex = 0;
int hammerAmount = 0;

void TC3_Handler() {
  TC_GetStatus(TC1, 0);

  bool value = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;

  bus_queue[2] = (bus_queue[2] << 1) | ((bus_queue[1] & (1UL << 63)) >> 63);
  bus_queue[1] = (bus_queue[1] << 1) | ((bus_queue[0] & (1UL << 63)) >> 63);
  bus_queue[0] = (bus_queue[0] << 1) | value;

  if ((bus_queue[2] & ONE_BITS(22)) == ONE_BITS(22) && bus_queue[1] == UINT64_MAX && bus_queue[0] == (ONE_BITS(54) << 10)) { 
    // check if 140 ones followed by 10 zeros is found in last 150 bits
    // ([2] has 22 ones, [1] has 64 ones, [0] has 54 ones & 10 zeros)
    LED_FLIP();
    sof = true;
    // startTimer(TC2, 0, TC6_IRQn, (uint32_t)(1 / HAMMER_POINT));
  }

  if (sof) {
    if (!(value == 0 && (bus_queue[0] & ONE_BITS(50)) == ONE_BITS(50)) &&
        !(value == 1 && (bus_queue[0] & ONE_BITS(50)) == 0)) {
      // if value is 0/1 and bus_queue has 50 1s/0s (5 bits), ignore
      frame_queue[2] = (frame_queue[2] << 1) | ((frame_queue[1] & (1UL << 63)) >> 63);
      frame_queue[1] = (frame_queue[1] << 1) | ((frame_queue[0] & (1UL << 63)) >> 63);
      frame_queue[0] = (frame_queue[0] << 1) | value;
    }

    // if ((frame_queue[1] & (ONE_BITS(10) << 52)) == 0 &&   // start-of-frame
    //     (frame_queue[1] & ONE_BITS(6)) == 0 &&      // RTR (must be 0 for data frame)
    //     (frame_queue[0] & (ONE_BITS(4) << 60)) == 0 &&
    //     (frame_queue[0] & (ONE_BITS(10) << 50)) == 0 &&      // IDE
    //     (frame_queue[0] & (ONE_BITS(10) << 40)) == 0 &&      // reserved bit
    //     (frame_queue[0] & (ONE_BITS(10) << 0)) != 0         // DLC (must be >0)

    if (
        BITS_AT_POS(frame_queue, 18, 1) == 0 &&     // start-of-frame
        (frame_queue[1] & ONE_BITS(6)) == 0 &&      // RTR (must be 0 for data frame)
        (frame_queue[0] & (ONE_BITS(4) << 60)) == 0 &&
        BITS_AT_POS(frame_queue, 5, 1) == 0 &&      // IDE
        BITS_AT_POS(frame_queue, 4, 1) == 0 &&      // reserved bit
        BITS_AT_POS(frame_queue, 0, 4) != 0         // DLC (must be >0)
    ) {
      frame_queue[2] = frame_queue[1] = frame_queue[0] = 0;
      sof = false;

      CANdy_Hammer();
    }
  }
}

void CANdy_Hammer() {
  // start hammering
  hammerIndex = 0;
  
  startTimer(TC2, 2, TC8_IRQn, SPEED);
}

void TC8_Handler() {
  TC_GetStatus(TC2, 2);

  if (hammerIndex < HAMMER_BIT_COUNT) {          //Will stop when there are no more bits to hammer in
    resetValue = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;

    hammerAmount = 0;                           //Number of data bits stuffed into CAN bit so far
    // turn on multiplexing
    PIOA->PIO_PER = PIO_PA0A_CANTX0;
    PIOA->PIO_OER = PIO_PA0A_CANTX0;

    startTimer(TC2, 1, TC7_IRQn, SPEED * 5);    //start timer to fire 5 times a bit
  } else {
    stopTimer(TC2, 2, TC8_IRQn);
  }
}

void TC7_Handler() {
  TC_GetStatus(TC2, 1); 

  if (hammerAmount < HAMMER_SIZE) {
    CANdy_Write(hammerData[hammerIndex]);
    hammerIndex++;    //move one higher on array
    hammerAmount++;   //+1 amount of bits that have been stuffed into current CAN bit
  } else {
    stopTimer(TC2, 1, TC7_IRQn);
    CANdy_Write(resetValue);

    // turn off multiplexing
    PIOA->PIO_PDR = PIO_PA0A_CANTX0;
    PIOA->PIO_ODR = PIO_PA0A_CANTX0;

  }
}

// void TC6_Handler() {
//   TC_GetStatus(TC2, 0);
//
//   if (!sof) {
//     stopTimer(TC2, 0, TC6_IRQn);
//
//     return;
//   }
//
//   sof = false;
//
//   resetValue = (PIOA->PIO_PDSR & PIO_PA1A_CANRX0) != 0;
//
//   // start hammering
//   hammerIndex = 0;
//
//   LED_FLIP();
//   PIOA->PIO_PER = PIO_PA0A_CANTX0;
//   PIOA->PIO_OER = PIO_PA0A_CANTX0;
//
//   startTimer(TC2, 1, TC7_IRQn, 4000000);
//
//   PIOA->PIO_PDR = PIO_PA0A_CANTX0;
//   PIOA->PIO_ODR = PIO_PA0A_CANTX0;
// }

void CANdy_Write(bool value) {
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

void stopTimer(Tc* tc, uint32_t channel, IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
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

  startTimer(TC1, 0, TC3_IRQn, SPEED * SAMPLING_RATE);
}

void loop() {
}
