#include <VescUart.h>

bool is_in_error_state = false;

int state = 0;
// 0 = idle
// 1 = armed
// 2 = launching
// 3 = retracting

const int pole_pairs = 7;
const float drum_circumference = 0.01; // in meters
const float belt_ratio = 1;

float tach_zero = 0;
float speed = 0;

VescUart UART;

void setup() {
  // UART setup
  Serial.begin(115200);
  while (!Serial) {;}
  UART.setSerialPort(&Serial);

  // Hardware setup
  

  // put your setup code here, to run once:
  float final_speed = 7.5; // m/s
  float cat_length = 12; // m
  float acceleration = sq(final_speed) / (2 * cat_length);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ( !UART.getVescValues() ) {
    Serial.println("VESC Values not returned");
    is_in_error_state = true;
    continue;
  }

  handle_errors();

  switch(state) {
    case 0:
      idle();
      break;
    case 1:
      armed();
      break;
    case 2:
      launching();
      break;
    case 3:
      retracting();
      break;
    default:
      state_error();
      break;
  }

  Serial.println(UART.data.rpm);
  Serial.println(UART.data.inpVoltage);
  Serial.println(UART.data.ampHours);
  Serial.println(UART.data.tachometerAbs);
  Serial.println(UART.data.tachometer);

  current_tach = UART.data.tachometer;
  current_position = tach_to_dist(current_tach);

  speed = calc_speed(current_position);
}

void idle() {
  // Handle not fully retracted
  if(current_tach > tach_zero) {
    state = 4;
  }

  
}

float tach_to_dist(long current_tach) {
  return ((current_tach - tach_zero) * drum_circumference) / (2 * pole_pairs * belt_ratio);
}

float calc_speed(float current_position) {
  return sqrt(2 * acceleration * current_position);
}

