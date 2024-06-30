#include "Motor.h"

Motor::Motor(int pin) {
    motorPin = pin; 
    Servo servo;
    servo.attach(pin);
    setSpeed(0);
}

void Motor::setSpeed(float speed) {
    // Remaps a percentage of max speed (-100 to 100) to a servo PWM signal (0-180)
    // 90 is 0 velocity for continous servo:
    // https://www.arduino.cc/reference/en/libraries/servo/write 
    int signal = mapGeneric(speed, -100, 100, 0, 180);
    servo.write(signal);
}

void Motor::stop(){
    setSpeed(0);
}
