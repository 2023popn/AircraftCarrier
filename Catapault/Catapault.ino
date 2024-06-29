#include "Motor.h"

#define servoPin 4

// #define targetVelocity 17
#define button 2

enum Stage {launchPrep, launch, reset};

#define button 2

void setup() {
  Motor motor(servoPin, maxPWM);
  
  // Button initialization
  pinMode(button, INPUT);

  enum Stage stage = launchPrep;

  float currentSpeed = 0;
  unsigned long beginTime = 0;
  unsigned long currentTime = 0;
}

void loop() {
  if(digitalRead(button)){
    stage += 1;
  }


  if(stage == launchPrep){
    setBeginTime();
  }

  if(stage == launch){
    currentTime = getTime();

    currentSpeed = calculateVelocity(currentTime);

    motor.setSpeed(currentSpeed);
  }

  if(stage == reset){
    motor.stop()
    // TODO: implement automatic reset where motor can go to
    // starting setpoint
  }
}

void setBeginTime(){
  beginTime = millis();
}

unsigned long getTime(){
  return millis() - beginTime;

void setBeginTime(){
  beginTime = millis();
}

unsigned long getTime(){
  return millis() - beginTime;
}

float calculateVelocity(unsigned long elapsedTime){
  return elapsedTime * 0.00581;
}

