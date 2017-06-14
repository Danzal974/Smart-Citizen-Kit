//External Sensors for SCK with atmega 48
// i2c slave based on Code : Wire Slave Sender by Nicholas Zambetti <http://www.zambetti.com>


#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x35);
  Wire.onRequest(requestEvent);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);

}

void requestEvent(){
  Wire.write("PH");
  Wire.write(7);
  }
