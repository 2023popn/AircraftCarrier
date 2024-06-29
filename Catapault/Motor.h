#ifndef Motor_h
#define Motor_h

#include <ESP32Servo.h>
#include <GeneralMap.h>

class Motor {
    private:
        int motorPin;
        Servo servo;
    
    public:
        Motor(int pin);
        void setSpeed(int speed);
        void stop();
}

#endif
