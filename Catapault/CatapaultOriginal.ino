#include "Motor.h"

#define servoPin 4

// #define targetVelocity 17
#define button 2

enum Stage {launchPrep, launch, reset};
enum Stage stage;

Motor motor(servoPin);
float currentSpeed;
unsigned long beginTime, currentTime;

#define button 2

void setup() {
    // Button initialization
    pinMode(button, INPUT);

    stage = launchPrep;

    currentSpeed = 0;
    beginTime = 0;
    currentTime = 0;
}

void loop() {
    if(digitalRead(button)){
        stage = static_cast<Stage>((stage + 1) % 3);
        // TODO : find method to get State enum length
    }


    if(stage == launchPrep){
        beginTime = millis();
    }

    if(stage == launch){
        currentTime = millis();
        unsigned long elapsedTime = currentTime - beginTime;
        currentSpeed = calculateVelocity(elapsedTime);
        motor.setSpeed(currentSpeed);
    }

    if(stage == reset){
        motor.stop();
        // TODO: implement automatic reset where motor can go to
        // starting setpoint
    }
}

float calculateVelocity(unsigned long elapsedTime){
    return elapsedTime * 0.00581;
}

