#include <Arduino.h>
#include <Serial.h>
#include <uart.h>


void setup() {
  Serial.begin(115200);  

}

void loop() {
  Serial.println(123);// put your main code here, to run repeatedly:
  delay(1000);
}
