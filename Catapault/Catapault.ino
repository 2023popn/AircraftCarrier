#include <ESP32Servo.h>
#include <GeneralMap.h>

Servo servo1;

#define targetVelocity 17
#define maxPWM 180

#define button 2

void setup() {
  // Servo initialization
  servo1.attach(4);
  servo1.write(0);

  // Button initialization
  pinMode(button, INPUT);

  // Stage Control Variable
  // stage 0: launch prep
  // stage 1: launch
  // stage 2: reset
  int stage = 0;
}

void loop() {
  if(stage == 1){
    delay(1000);
    setSpeed(17);
    delay(1000);
    setSpeed(0);
  }

  if(stage == 2){
    
  }
}


void setSpeed(float speed) {
  int signal = mapGeneric(speed, 0, targetVelocity, 0, maxPWM); // Remaps a percentage of max speed (0-100) to a servo PWM signal (0-180)
  
  servo1.write(signal);
}

float calculateVelocity(unsigned long elapsedTime){
  return elapsedTime * 0.00581;
}
