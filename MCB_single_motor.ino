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

// GLOBAL VARIABLES
const uint8_t MCBmodules_num = 1; // number of modules (i.e. motors) plugged into this board
MCB MotorBoard(MCBmodules_num);	// construct motor control board

long int startTime;
long int loopcount = 0;

float kp = 0.0001, ki = 0.000, kd = 0.000;
int32_t countDesired = 500;

void setup()
{

	MotorBoard.init();
	
	MotorBoard.modules.at(0).setGains(kp, ki, kd);
	
	MotorBoard.modules.at(0).setCountDesired(countDesired);

	digitalWriteFast(MotorBoard.pins.brakes[0], HIGH); // enable amp

	startTime = millis();
}

void loop()
{
	loopcount++;
	delay(100);

	MotorBoard.readButtons();
	if (MotorBoard.isUpPressed()) {
		countDesired += 100;
		MotorBoard.modules.at(0).setCountDesired(countDesired);
	}
	else if (MotorBoard.isDownPressed()) {
		countDesired -= 100;
		MotorBoard.modules.at(0).setCountDesired(countDesired);
	}

	MotorBoard.stepPID();
}
