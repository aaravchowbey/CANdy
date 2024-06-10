#include <due_can.h>

#define Serial SerialUSB
#define SPEED CAN_BPS_50K

static uint32_t lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Can0.begin(SPEED);
  Can0.watchFor();
  for (int filter = 3; filter < 7; filter++) {
    // receives standard frames
    Can0.setRXFilter(filter, 0, 0, false);
  }
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
  const uint8_t data[dataLength] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#ifndef SENDER2
  // sender
  if (millis() - lastTime > 1300) {
    lastTime = millis();
    sendData(data, dataLength);
  }
  //
  // CAN_FRAME incoming;
  // if (Can0.available() > 0) {
  //   Can0.read(incoming);
  // }
#else
  // sender2
  if (millis() - lastTime > 1700) {
    lastTime = millis();
    sendData(data, dataLength);
  }
  //
  // CAN_FRAME incoming;
  // if (Can0.available() > 0) {
  //   Can0.read(incoming);
  // }
#endif
}
