#include <due_can.h>

#define Serial SerialUSB
#define SPEED CAN_BPS_50K

static uint32_t lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Can0.begin(SPEED);
  Can0.watchFor();
}

void sendData(const uint8_t* data, const int dataLength) {
  CAN_FRAME outgoing;
  outgoing.id = 0x400;
  outgoing.extended = false;
  outgoing.priority = 4;
  outgoing.length = dataLength;

  for (int i = 0; i < dataLength; i++) {
    outgoing.data.byte[i] = data[i];
  }

  if (Can0.sendFrame(outgoing)) {
    // if frame successfully sent, toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void loop() {
  const int dataLength = 8;
  const uint8_t data[dataLength] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (millis() - lastTime > 1000) {
    lastTime = millis();
    sendData(data, dataLength);
  }
}
