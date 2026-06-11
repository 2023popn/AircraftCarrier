// =============================================================================
// Aircraft Carrier Catapult Controller
// =============================================================================
//
// This program controls an electromagnetic catapult on a scale aircraft carrier
// using an ESP32 and a VESC motor controller over UART.
//
// The system moves through four states in order:
//
//   IDLE  ->  ARMED  ->  LAUNCHING  ->  RETRACTING  ->  IDLE
//
//   IDLE:        Waiting for both arming switches to be flipped on.
//                If the shuttle is not at the home position, retracts first.
//
//   ARMED:       Both arming switches are held. The operator can dial in
//                launch speed with the potentiometer. Pressing the launch
//                button begins the launch.
//
//   LAUNCHING:   Motor accelerates the shuttle along the catapult track
//                using a constant-acceleration speed profile. Transitions
//                to RETRACTING once the shuttle reaches the end of the track.
//
//   RETRACTING:  Motor runs in reverse at a fixed speed until the shuttle
//                reaches the home limit switch, then returns to IDLE.
//
// Any error halts the system immediately and requires a hardware reset.
// Error details are printed to Serial for diagnosis.
//
// =============================================================================

#include <VescUart.h>


// =============================================================================
// PIN DEFINITIONS
// =============================================================================
// Inputs
#define CAPTAIN_ARM_SWITCH  1   // Arming switch operated by the captain
#define PILOT_ARM_SWITCH    2   // Arming switch operated by the pilot
#define LAUNCH_BUTTON       4   // Button that triggers the launch
#define HOME_LIMIT_SWITCH   6   // Limit switch at the start of the track (home)
#define END_LIMIT_SWITCH    7   // Limit switch at the end of the track (safety stop)
#define POT_PIN             34  // Potentiometer for selecting launch speed (ADC pin)

// Outputs
#define IDLE_LIGHT          0   // Indicator light: system is idle
#define ARMED_LIGHT         3   // Indicator light: system is armed
#define LAUNCH_LIGHT        5   // Indicator light: launch in progress


// =============================================================================
// ERROR CODES
// =============================================================================
// Each error is a unique bit in a bitmask so multiple errors can be recorded.
// Example: if ERR_VESC_COMMS and ERR_LOW_VOLTAGE both fire, error_history = 0x05.
#define ERR_NONE            0x00  // No error
#define ERR_VESC_COMMS      0x01  // VESC stopped responding
#define ERR_VESC_FAULT      0x02  // VESC reported an internal fault
#define ERR_LOW_VOLTAGE     0x04  // Battery voltage dropped below safe threshold
#define ERR_END_LIMIT       0x08  // Shuttle hit the end limit switch unexpectedly
#define ERR_LAUNCH_TIMEOUT  0x10  // Launch took too long (shuttle may be stalled)
#define ERR_RETRACT_TIMEOUT 0x20  // Retraction took too long (shuttle may be stalled)
#define ERR_UNKNOWN_STATE   0x40  // State machine reached an unexpected state


// =============================================================================
// TUNING CONSTANTS
// Change these to adjust system behaviour.
// =============================================================================

// --- Catapult geometry ---
// These values describe the physical dimensions of the catapult drive system.
const int   pole_pairs         = 7;     // Number of pole pairs in the motor
const float drum_circumference = 0.01;  // Circumference of the drive drum, in metres
const float belt_ratio         = 1.0;   // Gear/belt ratio between motor and drum (1.0 = direct drive)
const float cat_length         = 12.0;  // Length of the catapult track, in metres

// --- Launch speed ---
// The operator selects a target speed using the potentiometer while armed.
// MIN_LAUNCH_SPEED prevents the shuttle from being commanded to zero at
// the start of a launch, which would cause it to never begin moving.
const float MIN_SPEED         = 2.0;   // Minimum selectable final launch speed (m/s)
const float MAX_SPEED         = 15.0;  // Maximum selectable final launch speed (m/s)
const float MIN_LAUNCH_SPEED  = 0.5;   // Minimum speed output during launch profile (m/s)

// --- Retraction ---
const float RETRACT_SPEED     = 1.0;   // Fixed retraction speed, in metres/second

// --- Homing ---
const float HOMING_SPEED      = 0.5;   // Speed used to creep back to home on startup (m/s)

// --- Safety thresholds ---
const float         MIN_VOLTAGE         = 20.0;   // Halt if battery drops below this voltage
const unsigned long LAUNCH_TIMEOUT_MS   = 10000;  // Halt if launch takes longer than this (ms)
const unsigned long RETRACT_TIMEOUT_MS  = 15000;  // Halt if retraction takes longer than this (ms)

// --- Button debounce ---
// A switch must read the same value for this long before it is accepted as stable.
const unsigned long DEBOUNCE_MS = 25;


// =============================================================================
// SYSTEM STATE
// =============================================================================

int state = 0;  // Current state: 0=IDLE, 1=ARMED, 2=LAUNCHING, 3=RETRACTING

// Error tracking
uint8_t latest_error   = ERR_NONE;  // The most recently triggered error code
uint8_t error_history  = ERR_NONE;  // Bitmask of every error seen since last reset
bool    is_in_error_state = false;

// Launch profile — recalculated whenever the operator adjusts the speed dial
float final_speed   = 7.5;  // Target speed at end of launch (m/s) — set by potentiometer
float acceleration  = 0.0;  // Computed from final_speed and cat_length
float runway_length = 0.0;  // Set equal to cat_length

// Shuttle position tracking
float tach_zero      = 0.0;  // Tachometer value at the home position (calibrated on each retraction)
long  current_tach   = 0;    // Latest tachometer reading from the VESC
float current_speed  = 0.0;  // Latest commanded speed (m/s)

// Timeout tracking
unsigned long state_start_ms = 0;  // Time (ms) when the current state began

VescUart UART;


// =============================================================================
// DEBOUNCE TABLE
// =============================================================================
// Lists every input pin that should be debounced.
// is_pressed() uses this table to track the stable state of each pin independently.

struct DebounceState {
  int  pin;
  bool stable_state;    // Last accepted (stable) reading
  bool last_read;       // Most recent raw reading
  unsigned long last_change_ms;  // Time the raw reading last changed
};

DebounceState buttons[] = {
  {CAPTAIN_ARM_SWITCH, false, false, 0},
  {PILOT_ARM_SWITCH,   false, false, 0},
  {LAUNCH_BUTTON,      false, false, 0},
  {HOME_LIMIT_SWITCH,  false, false, 0},
  {END_LIMIT_SWITCH,   false, false, 0},
};
const int NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);


// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
// Tells the compiler these functions exist before their full definitions appear.
void  idle();
void  armed();
void  launching();
void  retracting();
void  state_error();
void  trigger_error(uint8_t error_code);
void  handle_errors();
void  home_shuttle();
void  update_launch_profile();
void  set_speed(float target_speed);
bool  is_pressed(int pin);
float read_target_speed();
float tach_to_dist(long tach);
float calc_speed(float position);
const char* error_name(uint8_t code);


// =============================================================================
// STARTUP
// =============================================================================

void home_shuttle() {
  Serial.println("Homing shuttle...");

  // Fetch fresh VESC data before reading the tachometer
  if (!UART.getVescValues()) {
    Serial.println("ERROR: Could not contact VESC during homing. Check wiring.");
    trigger_error(ERR_VESC_COMMS);
    return;
  }

  if (digitalRead(HOME_LIMIT_SWITCH)) {
    // Shuttle is already at home — record the current tach value as zero
    tach_zero = UART.data.tachometer;
    Serial.println("Already at home position.");
    return;
  }

  // Creep backwards until the home limit switch triggers or timeout expires
  const unsigned long HOMING_TIMEOUT_MS = 30000;  // 30 seconds
  unsigned long homing_start = millis();

  set_speed(-HOMING_SPEED);

  while (!digitalRead(HOME_LIMIT_SWITCH)) {
    if (millis() - homing_start > HOMING_TIMEOUT_MS) {
      UART.setCurrent(0);
      Serial.println("ERROR: Homing timed out. Check home limit switch and motor.");
      trigger_error(ERR_RETRACT_TIMEOUT);
      return;
    }
    delay(10);
  }

  UART.setCurrent(0);

  // Fetch fresh data again so tach_zero reflects the actual home position
  if (UART.getVescValues()) {
    tach_zero = UART.data.tachometer;
  }
  Serial.print("Homing complete. Zero set to tach value: ");
  Serial.println(tach_zero);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  UART.setSerialPort(&Serial);

  // Configure input pins
  pinMode(CAPTAIN_ARM_SWITCH, INPUT);
  pinMode(PILOT_ARM_SWITCH,   INPUT);
  pinMode(LAUNCH_BUTTON,      INPUT);
  pinMode(HOME_LIMIT_SWITCH,  INPUT);
  pinMode(END_LIMIT_SWITCH,   INPUT);

  // Configure output pins
  pinMode(IDLE_LIGHT,  OUTPUT);
  pinMode(ARMED_LIGHT, OUTPUT);
  pinMode(LAUNCH_LIGHT, OUTPUT);

  update_launch_profile();

  // Initialise the debounce table with real pin readings so the first
  // call to is_pressed() reflects the actual hardware state at startup
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool initial = digitalRead(buttons[i].pin);
    buttons[i].stable_state    = initial;
    buttons[i].last_read       = initial;
    buttons[i].last_change_ms  = millis();
  }

  home_shuttle();

  digitalWrite(IDLE_LIGHT, HIGH);
}


// =============================================================================
// MAIN LOOP
// =============================================================================
// Each iteration: (1) checks VESC health, (2) runs the current state function,
// (3) prints telemetry to Serial, (4) updates position tracking variables.

void loop() {
  if (!UART.getVescValues()) {
    trigger_error(ERR_VESC_COMMS);
    return;
  }
  if (UART.data.fault != 0) {
    Serial.print("VESC fault code: ");
    Serial.println(UART.data.fault);
    trigger_error(ERR_VESC_FAULT);
    return;
  }
  if (UART.data.inpVoltage < MIN_VOLTAGE) {
    Serial.print("Low voltage: ");
    Serial.println(UART.data.inpVoltage);
    trigger_error(ERR_LOW_VOLTAGE);
    return;
  }

  // Update tachometer before state functions run so they always use current data
  current_tach = UART.data.tachometer;

  handle_errors();

  switch (state) {
    case 0: idle();        break;
    case 1: armed();       break;
    case 2: launching();   break;
    case 3: retracting();  break;
    default: state_error(); break;
  }

  // Telemetry — printed every loop for monitoring over Serial
  Serial.print("RPM: ");           Serial.println(UART.data.rpm);
  Serial.print("Voltage: ");       Serial.println(UART.data.inpVoltage);
  Serial.print("Amp-hours: ");     Serial.println(UART.data.ampHours);
  Serial.print("Tach: ");          Serial.println(UART.data.tachometer);
  Serial.print("Tach (abs): ");    Serial.println(UART.data.tachometerAbs);
  Serial.print("Target speed: ");  Serial.print(final_speed); Serial.println(" m/s");
  Serial.print("Distance (m): ");  Serial.println(tach_to_dist(current_tach));
}


// =============================================================================
// STATE FUNCTIONS
// =============================================================================
// Each function handles one state. They are called once per loop iteration.
// To change state, set the 'state' variable to the new state number.

void idle() {
  digitalWrite(IDLE_LIGHT,   HIGH);
  digitalWrite(ARMED_LIGHT,  LOW);
  digitalWrite(LAUNCH_LIGHT, LOW);

  // If the shuttle is not at home, retract it before allowing arming
  if (!is_pressed(HOME_LIMIT_SWITCH)) {
    state_start_ms = millis();
    state = 3;  // -> RETRACTING
    return;
  }

  // Both arming switches must be held to move to ARMED
  if (is_pressed(CAPTAIN_ARM_SWITCH) && is_pressed(PILOT_ARM_SWITCH)) {
    digitalWrite(IDLE_LIGHT, LOW);
    state = 1;  // -> ARMED
  }
}

void armed() {
  digitalWrite(ARMED_LIGHT, HIGH);

  // If either arming switch is released, return to IDLE
  if (!is_pressed(CAPTAIN_ARM_SWITCH) || !is_pressed(PILOT_ARM_SWITCH)) {
    state = 0;  // -> IDLE
    return;
  }

  // Continuously read the potentiometer so the operator can adjust speed
  // before launch. Speed locks in the moment the launch button is pressed.
  final_speed = read_target_speed();
  update_launch_profile();

  if (is_pressed(LAUNCH_BUTTON)) {
    digitalWrite(ARMED_LIGHT,  LOW);
    digitalWrite(LAUNCH_LIGHT, HIGH);
    state_start_ms = millis();
    state = 2;  // -> LAUNCHING
  }
}

void launching() {
  if (millis() - state_start_ms > LAUNCH_TIMEOUT_MS) {
    trigger_error(ERR_LAUNCH_TIMEOUT);
    return;
  }

  // The end limit switch is a hard safety stop — it should never trigger
  // during a normal launch. If it does, halt immediately.
  if (is_pressed(END_LIMIT_SWITCH)) {
    trigger_error(ERR_END_LIMIT);
    return;
  }

  float current_distance = tach_to_dist(current_tach);
  current_speed = calc_speed(current_distance);

  if (current_distance < runway_length) {
    set_speed(current_speed);
  } else {
    // Shuttle has reached the end of the track — stop and retract
    UART.setCurrent(0);
    digitalWrite(LAUNCH_LIGHT, LOW);
    state_start_ms = millis();
    state = 3;  // -> RETRACTING
  }
}

void retracting() {
  if (millis() - state_start_ms > RETRACT_TIMEOUT_MS) {
    trigger_error(ERR_RETRACT_TIMEOUT);
    return;
  }

  if (is_pressed(HOME_LIMIT_SWITCH)) {
    // Shuttle is back at home — stop, re-calibrate zero, return to IDLE
    UART.setCurrent(0);
    tach_zero = UART.data.tachometer;
    Serial.print("Retraction complete. Zero updated to tach value: ");
    Serial.println(tach_zero);
    state = 0;  // -> IDLE
    return;
  }

  set_speed(-RETRACT_SPEED);  // Negative = reverse direction
}

void state_error() {
  // Should never be reached — indicates a bug in the state machine
  trigger_error(ERR_UNKNOWN_STATE);
}


// =============================================================================
// MOTOR CONTROL
// =============================================================================

// Converts a target linear speed (m/s) into a motor RPM command and sends
// it to the VESC. Positive = forward (launch), negative = reverse (retract).
//
// The conversion chain is:
//   linear speed (m/s)  ->  drum revolutions/sec  ->  motor revolutions/sec  ->  RPM
void set_speed(float target_speed) {
  float drum_rps  = target_speed / drum_circumference;
  float motor_rpm = drum_rps * 60.0 * pole_pairs * belt_ratio;
  UART.setRPM((int)motor_rpm);
}

// Computes the required speed (m/s) at a given position along the track
// to achieve a constant-acceleration launch profile.
// Clamped to MIN_LAUNCH_SPEED so the shuttle always receives a nonzero
// command at the start of a launch.
float calc_speed(float position) {
  float target = sqrt(2.0 * acceleration * position);
  return max(target, MIN_LAUNCH_SPEED);
}

// Recomputes acceleration from the current final_speed and track length.
// Must be called any time final_speed changes.
void update_launch_profile() {
  acceleration  = (final_speed * final_speed) / (2.0 * cat_length);
  runway_length = cat_length;
}


// =============================================================================
// POSITION TRACKING
// =============================================================================

// Converts a raw VESC tachometer value into a linear shuttle distance in metres,
// measured from the home position (tach_zero).
//
// The VESC tachometer counts 6 ticks per electrical revolution.
// One mechanical revolution = pole_pairs electrical revolutions.
// So ticks per mechanical revolution = pole_pairs * 6.
//
// Clamped to zero so a slight overshoot past home never produces a negative
// distance, which would cause calc_speed() to receive a negative input.
float tach_to_dist(long tach) {
  float mechanical_revs = (tach - tach_zero) / (pole_pairs * 6.0);
  float distance = mechanical_revs * drum_circumference / belt_ratio;
  return max(distance, 0.0f);
}


// =============================================================================
// INPUT READING
// =============================================================================

// Returns the stable (debounced) state of a given pin.
// Each pin in the buttons[] table is tracked independently.
// If a pin is not in the table, falls back to a raw digitalRead.
bool is_pressed(int pin) {
  unsigned long now = millis();

  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttons[i].pin != pin) continue;

    bool raw = digitalRead(pin);

    if (raw != buttons[i].last_read) {
      // The pin changed — start the stability timer
      buttons[i].last_read      = raw;
      buttons[i].last_change_ms = now;
    } else if ((now - buttons[i].last_change_ms) >= DEBOUNCE_MS) {
      // The pin has been stable long enough — accept it as the new state
      buttons[i].stable_state = raw;
    }

    return buttons[i].stable_state;
  }

  return digitalRead(pin);  // Pin not in debounce table — return raw value
}

// Reads the potentiometer and maps its value to a speed in the selectable range.
// The ESP32 ADC is 12-bit, so raw readings range from 0 to 4095.
float read_target_speed() {
  int raw = analogRead(POT_PIN);
  return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (raw / 4095.0);
}


// =============================================================================
// ERROR HANDLING
// =============================================================================

// Records an error, cuts motor power, and sets the halt flag.
// handle_errors() will catch the flag on the next loop and halt the system.
void trigger_error(uint8_t error_code) {
  UART.setCurrent(0);
  latest_error      = error_code;
  error_history    |= error_code;  // OR into history so previous errors are kept
  is_in_error_state = true;
  Serial.print("ERROR: ");
  Serial.println(error_name(error_code));
}

// If an error has been triggered, halts the system and repeatedly prints
// the error details to Serial. Requires a hardware reset to exit.
void handle_errors() {
  if (!is_in_error_state) return;

  UART.setCurrent(0);

  while (true) {
    Serial.println("=== SYSTEM HALTED ===");

    Serial.print("Last error   : ");
    Serial.print(error_name(latest_error));
    Serial.print(" (0x");
    Serial.print(latest_error, HEX);
    Serial.println(")");

    Serial.print("Error history: 0x");
    Serial.println(error_history, HEX);

    // List every error that has occurred since last reset
    uint8_t all_codes[] = {
      ERR_VESC_COMMS, ERR_VESC_FAULT, ERR_LOW_VOLTAGE, ERR_END_LIMIT,
      ERR_LAUNCH_TIMEOUT, ERR_RETRACT_TIMEOUT, ERR_UNKNOWN_STATE
    };
    for (int i = 0; i < 7; i++) {
      if (error_history & all_codes[i]) {
        Serial.print("  [x] ");
        Serial.println(error_name(all_codes[i]));
      }
    }

    Serial.println("Reset the ESP32 to restart.");
    delay(2000);
  }
}

// Returns a plain-English label for a given error code.
const char* error_name(uint8_t code) {
  switch (code) {
    case ERR_VESC_COMMS:      return "VESC not responding";
    case ERR_VESC_FAULT:      return "VESC internal fault";
    case ERR_LOW_VOLTAGE:     return "Battery voltage too low";
    case ERR_END_LIMIT:       return "End limit swi
