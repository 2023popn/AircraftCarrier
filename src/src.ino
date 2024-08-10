#include <VescUart.h>
#include <math.h>
#include <algorithm>

VescUart UART;

void setup() {
	Serial.begin(115200); // Debugging, seeing printed values
	Serial2.begin(115200, SERIAL_8N1, 16, 17); // UART to motor controller
	UART.setSerialPort(&Serial2);
	
	// Setting the debug port is helpful to debug individual UART messages.
	// UART.setDebugPort(&Serial);
}

float k_p = 0.0015;
float setpoint = 0.0;

void loop() {
	// If getVals is 0, there is a problem with UART connection
	bool getVals = UART.getVescValues();
	Serial.print("GETTING VALUES: ");
	Serial.println(getVals);
	if (getVals) {
		
		// See https://github.com/SolidGeek/VescUart/blob/master/src/VescUart.cpp#L195 for the
		// full list of printable values (UART.data.*)
		Serial.print("Current Position in deg: ");
		Serial.println(UART.data.pidPos);
		
		// calculate pid input. Using remainder function from math.h so that everything is within
		// -180 and 180. This way there ir not a jump between 0 and 360.
		float inpCurrent = k_p * remainder(setpoint - UART.data.pidPos, 360);
		Serial.print("PID output full: ");
		Serial.println(inpCurrent);
		
		// ensure there are no wildly high current readings by setting everything between -2 and 2
		inpCurrent = std::clamp(inpCurrent, -2.0f, 2.0f);
		Serial.print("PID output clamped: ");
		Serial.println(inpCurrent);
	
		// RPM doesn't seem to work, unsure why
		// UART.setRPM(60);

		// Current works, but is strange for negative values.
		// For pid, we would do UART.setCurrent(inpCurrent);, but we are just doing below for testing.
		UART.setCurrent(0.6);
	}
}
