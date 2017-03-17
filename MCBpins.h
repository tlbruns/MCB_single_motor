//=========================================================================//
/*
MCBpins Class

Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)
This class initializes and allows easy referencing to all pins


Trevor Bruns

Changelog-
2/20/2016: Initial Creation
3/13/2017: Moved from MCB.h to its own header file
		   Changed all uses of <vector> to standard arrays
		   Moved pin initializations from constructor to init()

*/
//=========================================================================//

#ifndef MCBpins_h
#define MCBpins_h

#include <stdint.h>

class MCBpins
{
public:
	MCBpins(void) {}

	void init(void) // initializes all SPI chip-select pins
	{
		// global chip-select pin for DACs (SYNC pin)
		pinMode(csDAC, OUTPUT);
		digitalWriteFast(csDAC, HIGH);

		// SD card chip-select
		pinMode(csSD, OUTPUT);
		digitalWriteFast(csSD, HIGH);

		// CTRL switch input
		pinMode(CTRL, INPUT);

		// WIZ820io
		pinMode(csWIZ, OUTPUT); // WIZ820io
		digitalWriteFast(csWIZ, HIGH);

		for (uint8_t aa = 0; aa < maxNumBoards; aa++)
		{
			// quadrature decoder chip-selects
			pinMode(csLS7366R[aa], OUTPUT);
			digitalWriteFast(csLS7366R[aa], HIGH);
			// status LEDs (green)
			pinMode(LEDG[aa], OUTPUT);
			digitalWriteFast(LEDG[aa], LOW);
			// software brakes (HIGH = amps disabled)
			pinMode(ampEnable[aa], OUTPUT);
			digitalWriteFast(ampEnable[aa], HIGH);
		}
	}

	// These pins are current as of Rev 1.2
	const uint8_t maxNumBoards = 6;
	const uint8_t csSD = 4;    // SD card chip-select
	const uint8_t csDAC = 27;  // global chip-select pin (SYNC) for DACs
	const uint8_t CTRL = 28; // CTRL switch input
	const uint8_t csWIZ = 10;// WIZ820io chip-select pin
	const uint8_t csLS7366R[6] = { 20, 17, 15, 29, 32, 30 }; // chip-select pins  !! if changed also change MCBmodule.pinEnc !!
	const uint8_t ampEnable[6] = { 21, 16, 14, 25, 33, 31 }; // motor amp enable pins (HIGH = POWER ON)
	const uint8_t LEDG[6] = { 24, 7, 6, 5, 3, 2 };   // control the green status LEDs
	const uint8_t buttonDown = 0;
	const uint8_t buttonUp = 1;
	const uint8_t buttonMenu = 23;
	const uint8_t buttons[3] = { buttonDown, buttonUp, buttonMenu };
	const float buttonThresh[3] = { 1500, 1500, 1500 }; // thresholds that constitute a key press (pF)
	bool buttonStates[3] = { 0, 0, 0 };
	//const DoubleVec buttonThresh = {1600, 1200, 1500}; // thresholds that constitute a key press (pF)
};

#endif