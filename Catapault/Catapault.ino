#include <ESP32Servo.h>
#include <GeneralMap.h>

Servo servo1;

#define targetVelocity 17
#define maxPWM 180

void setup() {
  // Set pin mode
  servo1.attach(4);
  servo1.write(0);
}

void loop() {
  delay(1000);
  setSpeed(17);
  delay(1000);
  setSpeed(0);
}


void setSpeed(float speed) {
  int signal = mapGeneric(speed, 0, targetVelocity, 0, maxPWM); // Remaps a percentage of max speed (0-100) to a servo PWM signal (0-180)
  
  servo1.write(signal);
}

float getCurrentSpeed(unsigned long elapsedTime){
  if(elapsedTime <= 1375){
    return elapsedTime * 0.00581;
  } else {
    return -1
  }
}
