#include <Arduino.h>
#include <Wire.h>
#include "TrapezStepper.h"

#define DEBUG			      true

// Pin definitions
#define J1_EN_PIN		    	2
#define J1_DIR_PIN	    		3
#define J1_STEPPIN		    	4

#define J2R_EN_PIN		    	5
#define J2R_DIR_PIN        		6
#define J2R_STEPPIN        		7

#define J2L_EN_PIN		    	8
#define J2L_DIR_PIN        		9
#define J2L_STEPPIN        		10


// motor parameters
#define J1_StepPerRev      		400
#define J1_ReductionRatio		5

#define J2_StepPerRev      		400
#define J2_ReductionRatio		15

// motion parameters
#define J1_MAX_VEL				6.28	/// [rad/s]
#define J1ACC					2.25		/// [rad^2/s]

#define J2_MAX_VEL			    2.25		/// [rad/s]
#define J2ACC				    3.14		/// [rad^2/s]

// Constants
#define REGULAR_DIR				1
#define INVERTED_DIR			0


/// Joint variables
int J2_EN[2] = { J2R_EN_PIN , J2L_EN_PIN };
int J2_DIR[2] = { J2R_DIR_PIN , J2L_DIR_PIN };
int J2_STEP[2] = { J2R_STEPPIN , J2L_STEPPIN };
bool J2_MOTOR_DIR[2] = { INVERTED_DIR, REGULAR_DIR };

byte inposState = false;

Stepper J1, J2;

// Misc functions
double getSerialfloat(String str)
{
	for (int i = 0; i > 1000; i++)
	{
		Serial.read();
	}
	Serial.println(str);
	while (!Serial.available());
	return (double)Serial.parseFloat();
}

// Stepper movement functions
void moveJ1(double t1)
{
	J1.inputSetpointRad(t1);
	while (J1.INPOS == false)
	{
		//unsigned long t1 = micros();
		J1.PTP_update();
		//unsigned long t2 = micros();
		//Serial.println(t2 - t1);
	}
}
void moveJ2(double t2)
{
	J2.inputSetpointRad(t2);
	while (J2.INPOS == false)
	{
		//unsigned long t1 = micros();
		J2.PTP_update();
		//unsigned long t2 = micros();
		//Serial.println(t2 - t1);
	}
}
void moveSystem()
{
	Serial.println("started run. RPOS is"+String(J1.RPOS));
	while (J1.INPOS == false || J2.INPOS == false)
	{
		if (inposState == true)
		{
			// set inpos flag to be false
			inposState = false;
		}

		// move motors to position
		J1.PTP_update();
		J2.PTP_update();

		// this loop will run until motors are in position
	}
	Serial.println("finished run. RPOS is"+String(J1.RPOS));

	if (inposState == false)
	{
		inposState = true;
	}
}
void moveSystem(double t1, double t2)
{
	J1.inputSetpointRad(t1);
	J2.inputSetpointRad(t2);
	while (J1.INPOS == false || J2.INPOS == false)
	{
		//unsigned long t1 = micros();
		J1.PTP_update();
		J2.PTP_update();
		//unsigned long t2 = micros();
		//Serial.println(t2 - t1);
	}
}
void StepperSetup()
{
	J1.init(J1_STEPPIN,J1_DIR_PIN,J1_EN_PIN,false, J1_StepPerRev,J1_ReductionRatio);
	J1.setVEL(J1_MAX_VEL);
	J1.setACC(J1ACC);
	J1.attach();

	J2.initDual(&J2_STEP[0], &J2_DIR[0], &J2_EN[0], &J2_MOTOR_DIR[0], J2_StepPerRev, J2_ReductionRatio);
	J2.setVEL(J2_MAX_VEL);
	J2.setACC(J2ACC);
	J2.detach();
}
void moveSystemJoints(double t1, double t2)
{
	J1.inputSetpointRad(t1);
	J2.inputSetpointRad(t2);
}

// I2C functions
void processCmd(int size)
{
	//Serial.println("hey");
	byte buff[10] = { 0 };
	int cmdType;
	int cmdValue[2];

	Wire.readBytes(buff, size);

	cmdType = buff[0] << 8 | buff[1];

	cmdValue[0] = buff[2] << 8 | buff[3];
	cmdValue[1] = buff[4] << 8 | buff[5];

	switch (cmdType)
	{
	default:
		if(DEBUG)
		{
			Serial.println("no such option");
		}
		break;
	case 1:
		J1.inputSetpointRad((double)cmdValue[0]);
		J2.inputSetpointRad((double)cmdValue[1]);
		break;
	}

}
void SendInPosFlag()
{
	byte package[1] = { inposState };
	Wire.write(package, 1);
}
void WireSetup()
{
	Wire.begin(0x02);
	Wire.onRequest(SendInPosFlag);
	Wire.onReceive(processCmd);
}

// Main loop functions
void SlaveLoop()
{
	moveSystem();
}
void TestLoop()
{
	double J1RequiredPosition = getSerialfloat("input J1 setpoint in rad");
	double J2RequiredPosition = getSerialfloat("input J2 setpoint in rad");
	Serial.println("Motion staring in 1 sec.");
	delay(1000);
	moveSystem(J1RequiredPosition, J2RequiredPosition);

}
void demoLoop()
{
	Serial.println("hello");
	double dist = PI/4;
	J1.inputSetpointRad(dist);
	J2.inputSetpointRad(dist);
	moveSystem();
	J1.inputSetpointRad(-dist);
	J2.inputSetpointRad(-dist);
	moveSystem();
	Serial.println("demo loop");
	delay(1000);
}

// Define some steppers and the pins the will use
void setup(){
	if (DEBUG)
		Serial.begin(115200);

	StepperSetup();
	WireSetup();
}

// Add the main program code into the continuous loop() function
void loop(){
	if (DEBUG == true)
	{
		demoLoop();
		//TestLoop();
	}
	else
	{
		SlaveLoop();
	}
}