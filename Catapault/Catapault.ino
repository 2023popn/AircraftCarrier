#include <ESP32Servo.h>

Servo servo1;

void setup() {
  // Set pin mode
  servo1.attach(4);
  servo1.write(0);

  // Speed control variables
  const int maxPWM = 180;
  const float targetVelocity = 17;
}

void loop() {
  delay(1000);
  setspeed(180);
  delay(1000);
  setspeed(0);
}


void setspeed(int speed) {
  int signal = map(speed, 0, targetVelocity, 0, maxPWM); // Remaps a percentage of max speed (0-100) to a servo PWM signal (0-180)
  
  servo1.write(signal);
}


