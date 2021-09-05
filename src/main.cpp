#include <Arduino.h>
#include <Wire.h>
#include "TrapezStepper.h"

#define DEBUG			      	false

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
#define J1_ReductionRatio		10

#define J2_StepPerRev      		400
#define J2_ReductionRatio		63.333333

// motion parameters
#define J1_MAX_VEL				6.28	/// [rad/s]
#define J1ACC					2.25		/// [rad^2/s]

#define J2_MAX_VEL			    2.25		/// [rad/s]
#define J2ACC				    3.14		/// [rad^2/s]

// Constants
#define REGULAR_DIR				1
#define INVERTED_DIR			0

#define CMD_TYPE_MOVE			1


/// Joint variables
int J2_EN[2] = { J2R_EN_PIN , J2L_EN_PIN };
int J2_DIR[2] = { J2R_DIR_PIN , J2L_DIR_PIN };
int J2_STEP[2] = { J2R_STEPPIN , J2L_STEPPIN };
bool J2_MOTOR_DIR[2] = { REGULAR_DIR, INVERTED_DIR };

byte inposState = false;

Stepper J1, J2;

// Misc functions
void emptySerialBuffer(int iterations)
{
	for (int i = 0; i < iterations; i++)
	{
		Serial.read();
	}
}
double getSerialfloat(String str)
{
	emptySerialBuffer(100);
	Serial.println(str);
	while (!Serial.available());
	return (double) Serial.parseFloat();
}
void blink_led(int times, unsigned long dt)
{
  for(int i=0;i<times;i++)
  {
    digitalWrite(13,HIGH);
    delay(dt);
    digitalWrite(13,LOW);
    delay(dt);
  }
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

void process_cmd(int size)
{
	// The buffers in the message received are arranged in Big Endian configuration (MSB first)
	uint8_t buff[5];
	uint8_t _cmd_type;
	double _cmd_value[2];

	Wire.readBytes(buff, 5);

	_cmd_type = buff[0];

	for(int i=0;i<2;i++)
	{
		int16_t tmp = buff[2*i + 1] << 8 | buff[2*i + 2];
		_cmd_value[i] = (double)tmp / 1000;
	}
	switch (_cmd_type)
	{
	default:
		if(DEBUG)
		{
			Serial.println("no such option. enter 1 in cmd_type");
		}
		break;
	case 1:
		for(int i=0;i<2;i++)
		{
			if(abs(_cmd_value[i])>(2*PI))
			{
				Serial.println("Abs value of joints is too high (>2*PI)");
				return;
			}
			else;
		}
		J1.inputSetpointRad(_cmd_value[0]);
		J2.inputSetpointRad(_cmd_value[1]);
		break;
	}
}
void SendFlag()
{
	uint8_t package = 5;
	Wire.write(package);
}
void WireSetup()
{
	Wire.begin(2);
	Wire.onRequest(SendFlag);
	Wire.onReceive(process_cmd);
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
	{
		Serial.begin(115200);
	}
	
	pinMode(13,OUTPUT);
	StepperSetup();
	WireSetup();
}

// Add the main program code into the continuous loop() function
void loop(){
	if (DEBUG == true)
	{
		//demoLoop();
		TestLoop();
	}
	else
	{
		SlaveLoop();
	}
}