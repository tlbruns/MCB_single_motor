/*=========================================================================//

	MCB Class
	
	This class handles the modules plugged into the motor controller board
	
	Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)
	
	
	Trevor Bruns
	
	Changelog-
		2/20/2016: Initial Creation
		3/13/2017: Moved MCBpin struct into its own class -> MCBpins.h
				   Fixed all issues due to switching from vectors to arrays in MCBpins.h
				   Compiles and appears to run successfully		

//=========================================================================*/

#ifndef MCB_h
#define MCB_h

#include "MCBmodule.h"
#include <ArduinoSTL.h> // this contains a <vector> class
#include "AD5761R.h"
#include "si5351.h"
#include "core_pins.h"
#include "MCBpins.h"

typedef std::vector<uint8_t>  Uint8Vec;
typedef std::vector<int16_t>  Int16Vec;
typedef std::vector<int32_t>  Int32Vec;
typedef std::vector<uint32_t> Uint32Vec;
typedef std::vector<double>	  DoubleVec;
typedef std::vector<bool>	  BoolVec;

class MCB
{	
public:
	
	MCBpins pins;  // set up uC pin names for the MCB.
	MCB(uint8_t numModules);	// will initialize modules using default pins (still need to call init after)
	~MCB(void);
	std::vector<MCBmodule> modules; // each module controls a single motor
	void init(void);	// initialize all modules (to be called after all addModule commands)
	void addModule(uint8_t position);
	void disableAllAmps(void); // sets inhibit pin for motor amps
	void enableAllAmps(void);  // NOTE: only enables n = numModules_
	void setDACs(Int16Vec const &val); // manually set DAC outputs
	void stepPID(void); // PID controller performs one step (reads encoders, computes effort, updates DACs)
	void setCount(uint8_t moduleNum, int32_t countDesired); // set desired motor position
	Int32Vec getCounts(void); // returns most recent motor positions
	int32_t getCount(uint8_t moduleNum); // returns most recent motor position
	Uint32Vec readButtons(void);	 // check capacitive sensor buttons for key presses
	bool isDownPressed(void); // returns current state of down button
	bool isUpPressed(void);	 // returns current state of up button
	bool isMenuPressed(void); // returns current state of menu button

private:
	AD5761R DAC_;
	Int16Vec DACval_;
	Si5351 si5351_;
	uint8_t numModules_;
	uint8_t quadratureMode_; // counts per quadrature cycle (1x, 2x, 4x)
};

#endif
