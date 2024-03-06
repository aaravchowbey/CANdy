//Reads all traffic on CAN0 and forwards it to CAN1 (and in the reverse direction) but modifies some frames first.
// Required libraries
#include "variant.h"
#include <due_can.h>

//Leave defined if you use native port, comment if using programming port
#define Serial SerialUSB
#define HAMMER_POINT 89e-6
#define SPEED CAN_BPS_250K

volatile bool sof = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);

  if (Can0.begin(SPEED)) {  }
  else {
    Serial.println("CAN initialization (sync) ERROR");
    
  }

  PMC->PMC_PCER0 |= PMC_PCER0_PID11; // PIOA power ON
  

//  //Multiplex CAN_TX to GPIO (Peripheral Enable Register)
//  PIOA->PIO_PER = PIO_PA0A_CANTX0;

  //Set CAN_RX as input (Ouput Disable Register) and
  //CAN_TX as output (Output Enable Register)
  PIOA->PIO_PER = PIO_PA1A_CANRX0;
  PIOA->PIO_ODR = PIO_PA1A_CANRX0;

//  PIOA->PIO_PER = PIO_PA0A_CANTX0;
//  PIOA->PIO_OER = PIO_PA0A_CANTX0;

  //Disable pull-up on both pins (Pull Up Disable Register)
  PIOA->PIO_PUDR = PIO_PA1A_CANRX0;
  PIOA->PIO_PUDR = PIO_PA0A_CANTX0;

  attachInterrupt(PIO_PA0A_CANTX0, CANdy_Sync, FALLING);
  Can0.watchFor();
  
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

void CANdy_Sync() { // We will run TC3_Handler
  
  sof = true;
  NVIC_DisableIRQ(TC3_IRQn);
  startTimer(TC1, 0, TC3_IRQn, 1 / HAMMER_POINT); // This allows us to sample exactly at 20% (SAMPLING_POINT == 5) of the bit time (1/f = T). This is where the ringing will be done.
  detachInterrupt(PIO_PA0A_CANTX0);
  digitalWrite(LED_BUILTIN, HIGH);
}

void TC3_Handler() {
  
  TC_GetStatus(TC1, 0);

//   PIO_PA1A_CANRX0 = If there is a bit
//   PIO_PDSR = Time efficency (digial inputs)
  bool value = PIOA->PIO_PDSR & PIO_PA0A_CANTX0;    // We read/sample the bit here

  // sof = Start of Frame (goes through each frame) (Should be 0)
  // Stuff bit = If there is a row of 5 consistant bits, then add a stuff bit of the opposite bit
  // Stuff Bit contains no useful information!
  // 0 0 0 0 0 1 (STUFF BIT) 0 0...  (+2 ms) (Each bit is 2 ms)
  // No stuff bits after Acknlowedgement bit!

  PIOA->PIO_PER = PIO_PA0A_CANTX0;
  PIOA->PIO_OER = PIO_PA0A_CANTX0;
  PIOA->PIO_CODR = PIO_PA0A_CANTX0;
//  PIOA->PIO_ODR = PIO_PA0A_CANTX0;
//  PIOA->PIO_PDR = PIO_PA0A_CANTX0;
//  digitalWrite(LED_BUILTIN,  HIGH);

}

void sendData() {
  
  CAN_FRAME outgoing;
  outgoing.id = 0x400;
  outgoing.extended = false;
  outgoing.priority = 4;
  outgoing.length = 8;

  outgoing.data.byte[0] = 0x00;
  outgoing.data.byte[1] = 0x11;
  outgoing.data.byte[2] = 0x22;
  outgoing.data.byte[3] = 0x33;
  outgoing.data.byte[4] = 0x44;
  outgoing.data.byte[5] = 0x55;
  outgoing.data.byte[6] = 0x66;
  outgoing.data.byte[7] = 0x77;

  Can0.sendFrame(outgoing);
  
}

void loop() {
//  static unsigned long lastTime = 0;
//  
//  if ((millis() - lastTime) > 1000) {
//    lastTime = millis();
//    sendData();
//  }
  
  delay(500);
  PIOA->PIO_SODR = PIO_PA0A_CANTX0;
  
}
