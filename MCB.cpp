/*=========================================================================//

	MCB Class
	
	This class handles the modules plugged into the motor controller board
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.1)
	
	
	Trevor Bruns
	
	Changelog-
		2/20/2016: Initial Creation
		3/13/2017: Compiles and appears to run successfully
		
//========================================================================*/

#include "MCB.h"
#include "MCBmodule.h"
#include <ArduinoSTL.h> 
#include "si5351.h"
#include <SPI.h>

MCB::MCB(uint8_t numModules)
// will initialize modules using default pins (still need to call init after)
	: DAC_(pins.csDAC)
	, numModules_(numModules)	
{	
	// reserve memory for module components
	modules.reserve(numModules_);
	DACval_.reserve(numModules_);
	LEDG_.reserve(numModules_);
}

void MCB::init(void)
// initializes all modules (to be called only after all addModule commands)
{

	// initialize MCB pins
	pins.init();

	// initialize encoder clock for LS7366R
	si5351_.init(SI5351_CRYSTAL_LOAD_8PF, 0);
	si5351_.set_freq(3000000000ULL, 0ULL, SI5351_CLK0); // Set CLK1 to output 30 MHz
	si5351_.output_enable(SI5351_CLK1, 0); // Disable other clocks
	si5351_.output_enable(SI5351_CLK2, 0);
	
	// initialize SPI
	SPI.begin();
	
	// create and initialize each module (along with each encoder IC)
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
		addModule(aa);
	}
	
	// initialize DACs
	DAC_.beginTransfer();
	for (uint8_t bb = 0; bb < numModules_; bb++)
	{
		DAC_.reset(); // software reset
	}
	DAC_.endTransfer();
	
	DAC_.beginTransfer();
	for (uint8_t bb = 0; bb < numModules_; bb++)
	{
		DAC_.init(); // setup ctrl register
	}
	DAC_.endTransfer();

	DAC_.beginTransfer();
	for (uint8_t bb = 0; bb < numModules_; bb++)
	{
		DAC_.set(0); // Set output to 0 volts
	}
	DAC_.endTransfer();

	// enable motor amp outputs and turn on green LEDs
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
		// software brakes (HIGH = amps enabled)
		digitalWriteFast(pins.brakes[aa], HIGH);

		setLEDG(aa, HIGH);
	}

}

void MCB::addModule(uint8_t position)
{
	modules.push_back(MCBmodule(position)); // create new MCBmodule and add to storage vector
	modules.at(position).init(); // initialize module

	DACval_.push_back(0);   // initialize DAC output values to 0
	LEDG_.push_back(false); // begin with LEDs off
}

void MCB::disableAllAmps(void)
{
	for (uint8_t aa = 0; aa < pins.maxNumBoards; aa++)
	{
		// software brakes (LOW = amps disabled)
		digitalWriteFast(pins.brakes[aa], LOW);
	}
}

void MCB::enableAllAmps(void)
{
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
		// software brakes (HIGH = amps enabled)
		digitalWriteFast(pins.brakes[aa], HIGH);
	}
}

void MCB::setLEDG(uint8_t position, bool state)
{
	LEDG_.at(position) = state;
	digitalWriteFast(pins.LEDG[position], LEDG_.at(position));
}

void MCB::setDACs(Int16Vec const &val)
{

	DAC_.beginTransfer();
	
	for (uint8_t bb = 0; bb < val.size(); bb++)
	{
		DAC_.set(val.at(bb));
	}
	
	DAC_.endTransfer();
}

void MCB::stepPID(void)
{
	
	// step PID controllers
	for (uint8_t aa = 0; aa < modules.size(); aa++)
	{
		DACval_.at(aa) = modules.at(aa).step();
	}
	
	// update DACs
	setDACs(DACval_);
	//DAC_.beginTransfer();
	//for (uint8_t bb = 0; bb < numModules_; bb++)
	//{
	//	DAC_.set(DACval_.at(bb));
	//}
	//DAC_.endTransfer();

}

void MCB::setCount(uint8_t moduleNum, int32_t countDesired)
{
	
	modules.at(moduleNum -1).setCountDesired(countDesired);
}

Int32Vec MCB::getCounts(void)
{
	Int32Vec temp;
	for (uint8_t aa = 0; aa < modules.size(); aa++)
	{
		temp.at(aa) = modules.at(aa).getCountLast();
	}
	
	return temp;
}

int32_t MCB::getCount(uint8_t moduleNum)
{	
	return modules.at(moduleNum).getCountLast();
}

Uint32Vec MCB::readButtons(void)
{	
	uint8_t numButtons = sizeof(pins.buttons);
	Uint32Vec buttonValues;
	buttonValues.resize(numButtons);
	
	for (uint8_t aa = 0; aa < numButtons; aa++)
	{
		buttonValues.at(aa) = touchRead(pins.buttons[aa]);
		pins.buttonStates[aa] = (buttonValues[aa] > pins.buttonThresh[aa]);
	}
	
	return buttonValues;
}

bool MCB::isDownPressed(void)
{
	return pins.buttonStates[0];
}

bool MCB::isUpPressed(void)
{
	return pins.buttonStates[1];
}

bool MCB::isMenuPressed(void)
{
	return pins.buttonStates[2];
}

MCB::~MCB(void)
{
	// ensure all amps are disabled
	disableAllAmps();
}