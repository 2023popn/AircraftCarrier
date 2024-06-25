#include <ESP32Servo.h>

Servo servo1;

void setup() {
  // Set pin mode
  servo1.attach(4);
  servo1.write(0);
}

void loop() {
  delay(1000);
  servo1.write(180);
  delay(1000);
  servo1.write(0);
}