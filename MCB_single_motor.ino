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
#include <arm_math.h>
#include <IntervalTimer.h>

// GLOBAL VARIABLES
const uint8_t MCBmodules_num = 1; // number of modules (i.e. motors) plugged into this board
MCB MotorBoard(MCBmodules_num);	// construct motor control board

long int startTime;
long int lastTime = 0;
long int loopcount = 0;

bool pidLedState = false;
bool loopLedState = false;

IntervalTimer PIDTimer;
void PIDTimerISR(void);
uint32_t timeStepPID = 1000;
float kp = 0.0004, ki = 0.000002, kd = 0.01;
int32_t countDesired = 0;

uint32_t buttonUpdateInterval = 5; // [ms]
uint32_t buttonCountChange = 300; // [counts]

float phase = 0.0;
const int32_t sinAmplitude = 10000; // [counts]
const float twopi = 2 * PI;

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

	if ((millis() - lastTime) > buttonUpdateInterval) { // ~700us to run when button is pressed
		loopLedState = !loopLedState;
		digitalWriteFast(MotorBoard.pins.LEDG[1], loopLedState);

		if (digitalReadFast(MotorBoard.pins.CTRL)) {
			// generate sin wave trajectory
			countDesired = (int32_t)(arm_sin_f32(phase) * sinAmplitude);
			MotorBoard.modules.at(0).setCountDesired(countDesired);
			phase = phase + 0.02;
			if (phase >= twopi) phase = 0.0;
		}
		else {
			MotorBoard.readButtons();
			if (MotorBoard.isUpPressed()) {
				countDesired += buttonCountChange;
				MotorBoard.modules.at(0).setCountDesired(countDesired);
			}
			else if (MotorBoard.isDownPressed()) {
				countDesired -= buttonCountChange;
				MotorBoard.modules.at(0).setCountDesired(countDesired);
			}
			else if (MotorBoard.isMenuPressed()) {
				MotorBoard.disableAllAmps();
			}
		}

		loopLedState = !loopLedState;
		digitalWriteFast(MotorBoard.pins.LEDG[1], loopLedState);

		lastTime = millis();
	}
		
}

void PIDTimerISR(void)
{
	MotorBoard.stepPID();
	MotorBoard.setLEDG(0, pidLedState);
	pidLedState = !pidLedState;
}
