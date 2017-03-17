#include <i2c_t3.h>
#include <SPI.h>
#include <ArduinoSTL.h>
#include "LS7366R.h"
#include "AD5761R.h"
#include "si5351.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "MCBpins.h"
#include "PID_f32.h"
#include <IntervalTimer.h>

// GLOBAL VARIABLES
const uint8_t MCBmodules_num = 1; // number of modules (i.e. motors) plugged into this board
MCB MotorBoard(MCBmodules_num);	// construct motor control board

long int startTime;
long int lastTime = 0;
long int loopcount = 0;

IntervalTimer PIDTimer;
void PIDTimerISR(void);
uint32_t timeStepPID = 2000;
float kp = 0.0001, ki = 0.000, kd = 0.000;
int32_t countDesired = 0;

uint32_t buttonUpdateInterval = 100; // [ms]
uint32_t buttonCountChange = 500; // [counts]

void setup()
{

	MotorBoard.init();
	
	MotorBoard.modules.at(0).setGains(kp, ki, kd);
	
	MotorBoard.modules.at(0).setCountDesired(countDesired);

	MotorBoard.enableAllAmps(); // enable amp

	startTime = millis();

	PIDTimer.begin(PIDTimerISR, timeStepPID); // attach function and call every timeStepPID [us]
}

void loop()
{
	loopcount++;

	if ((millis() - lastTime) > buttonUpdateInterval) {
		MotorBoard.readButtons();
		if (MotorBoard.isUpPressed()) {
			countDesired += buttonCountChange;
			MotorBoard.modules.at(0).setCountDesired(countDesired);
		}
		else if (MotorBoard.isDownPressed()) {
			countDesired -= buttonCountChange;
			MotorBoard.modules.at(0).setCountDesired(countDesired);
		}

		lastTime = millis();
	}
	
}

void PIDTimerISR(void)
{
	MotorBoard.stepPID();
}
