#include <SoftwareSerial.h>

#if defined(__AVR_ATmega328P__)
SoftwareSerial Serial1(13,12); //(RX, TX)
#endif

void setup() {
  // put your setup code here, to run once:

  Serial1.begin(19200);
  Serial.begin(19200);
}

void loop() {
  // put your main code here, to run repeatedly:

  while(Serial.available()){
    uint8_t t = Serial.read();
    Serial1.write(t);
  }

  while(Serial1.available()){
    uint8_t r = Serial1.read();
    Serial.write(r);
  }
}
