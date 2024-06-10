#include <due_can.h>

#define Serial SerialUSB
#define SPEED CAN_BPS_50K

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Can0.begin(CAN_BPS_50K);
  Can0.watchFor();
  // for (int filter = 3; filter < 7; filter++) {
  //   // receives standard frames
  //   Can0.setRXFilter(filter, 0, 0, false);
  // }
}

void printFrame(CAN_FRAME &frame) {
  Serial.print("ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data: 0x");
  for (int count = 0; count < frame.length; count++) {
    Serial.print(frame.data.bytes[count], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\n");

  // if frame received, toggle LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void loop() {
  CAN_FRAME incoming;

  if (Can0.available() > 0) {
    Can0.read(incoming);
    printFrame(incoming);
  }
}
