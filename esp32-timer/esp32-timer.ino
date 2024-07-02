#include "esp_system.h"
#include "rom/ets_sys.h"

#define SIZE 5

hw_timer_t* timer = NULL;

volatile uint16_t ind = 0;
uint64_t timer_vals[SIZE] = {0};

ICACHE_RAM_ATTR void reset_timer() {
  timerWrite(timer, 0);
}

ICACHE_RAM_ATTR void store_time() {
  uint64_t val = timerRead(timer);
  timer_vals[ind % SIZE] = val;

  ind++;
}

void setup() {
  Serial.begin(115200);

  pinMode(4, INPUT);
  pinMode(5, INPUT);

  // start timer w/ period = 1μs
  timer = timerBegin(1000000);

  attachInterrupt(digitalPinToInterrupt(4), reset_timer, RISING);
  attachInterrupt(digitalPinToInterrupt(5), store_time, RISING);
}

void loop() {
  if (ind != 0 && ind % SIZE == 0) {
    for (uint8_t i = 0; i < ind; i++) {
      Serial.println(timer_vals[i]);
    }
  }
}
