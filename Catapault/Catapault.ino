#include <ESP32Servo.h>

Servo servo1;

void setup() {
  // Set pin mode
  servo1.attach(4);
  servo1.write(0);
}

void loop() {
  delay(1000);
  setspeed(180);
  delay(1000);
  setspeed(0);
}


void setspeed(int speed) {
  signal = map(speed, 0, 100, 0, 180); // Remaps a percentage of max speed (0-100) to a servo PWM signal (0-180)
  
  servo1.write(signal);
}
