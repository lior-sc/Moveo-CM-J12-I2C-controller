#include <Arduino.h>
#include "TrapezStepper.h"


void Stepper::init(int stepPin, int dirPin, int enablePin, bool invert_motor, double steps_per_revolution, double reduction_ratio)
{
	DUAL_STEPPER = false;

	STEPS_PER_REVOLUTION = steps_per_revolution;
	REDUCTION_RATIO = reduction_ratio;
	STEPS_PER_RAD = steps_per_revolution * reduction_ratio / (2*PI);
	STEPS_PER_DEG = steps_per_revolution * reduction_ratio / 360;

	ENABLEPIN = enablePin;
	DIRPIN = dirPin;
	STEPPIN = stepPin;
	M1_DIR = invert_motor;

	MAX_ACC = DEFAULT_MAX_ACC;
	MAX_VEL = DEFAULT_MAX_VEL;

	MAX_STEP_VELOCITY= (long int)(MAX_VEL * STEPS_PER_RAD); // units are rad
	MAX_STEP_ACCELERATION = (long int)(MAX_ACC * STEPS_PER_RAD);
	
	pinMode(STEPPIN, OUTPUT);
	pinMode(DIRPIN, OUTPUT);
	pinMode(ENABLEPIN, OUTPUT);
	digitalWrite(DIRPIN, M1_DIR);

	Serial.println("Max velocity [microsteps/s]: " + String(MAX_STEP_VELOCITY)+"\nMin cycle time [Us]: " +String(1000000/ MAX_STEP_VELOCITY));
}
void Stepper::initDual(int *stepPin, int *dirPin, int *enablePin,  bool *motorDirection, double steps_per_revolution, double reduction_ratio)
{
	DUAL_STEPPER = true;

	STEPS_PER_REVOLUTION = steps_per_revolution;
	REDUCTION_RATIO = reduction_ratio;
	STEPS_PER_RAD = steps_per_revolution * reduction_ratio / (2*PI);
	STEPS_PER_DEG = steps_per_revolution * reduction_ratio / 360;

	M1_DIR = motorDirection[0];
	M2_DIR = motorDirection[1];
	DUAL_STEPPIN[0] = stepPin[0];
	DUAL_STEPPIN[1] = stepPin[1];
	DUAL_DIRPIN[0] = dirPin[0];
	DUAL_DIRPIN[1] = dirPin[1];
	DUAL_ENABLEPIN[0] = enablePin[0];
	DUAL_ENABLEPIN[1] = enablePin[1];

	MAX_ACC = DEFAULT_MAX_ACC;
	MAX_VEL = DEFAULT_MAX_VEL;

	MAX_STEP_VELOCITY= (long int)(MAX_VEL * STEPS_PER_RAD); 
	MAX_STEP_ACCELERATION = (long int)(MAX_ACC * STEPS_PER_RAD);

	pinMode(DUAL_STEPPIN[0], OUTPUT);
	pinMode(DUAL_STEPPIN[1], OUTPUT);

	pinMode(DUAL_DIRPIN[0], OUTPUT);
	pinMode(DUAL_DIRPIN[1], OUTPUT);

	pinMode(DUAL_ENABLEPIN[0], OUTPUT);
	pinMode(DUAL_ENABLEPIN[1], OUTPUT);

	digitalWrite(DUAL_DIRPIN[0], M1_DIR);
	digitalWrite(DUAL_DIRPIN[1], M2_DIR);

	Serial.println("Step pins: " + String(DUAL_ENABLEPIN[0]) + "  " + String(DUAL_ENABLEPIN[1]));
	Serial.println("Max velocity [microsteps/s]: " + String(MAX_STEP_VELOCITY) + "\nMin cycle time [Us]: " + String(1000000 / MAX_STEP_VELOCITY));
}

/// Stepper interface
void Stepper::attach()
{
	if (!DUAL_STEPPER)
	{
		digitalWrite(ENABLEPIN, HIGH);
	}
	else
	{
		digitalWrite(DUAL_ENABLEPIN[0], LOW);
		digitalWrite(DUAL_ENABLEPIN[1], LOW);
	}	
}
void Stepper::detach()
{
	if (!DUAL_STEPPER)
	{
		digitalWrite(ENABLEPIN, LOW);
	}
	else
	{
		digitalWrite(DUAL_ENABLEPIN[0], HIGH);
		digitalWrite(DUAL_ENABLEPIN[1], HIGH);
	}
}
void Stepper::setDirection()
{

	if (SETPOINT - RPOS >= 0)
	{
		DIR = FORWARD;
	}
	else
	{
		DIR = BACKWARD;
	}
	if (!DUAL_STEPPER)
	{
		M1_DIR == FORWARD ? digitalWrite(DIRPIN, DIR) : digitalWrite(DIRPIN, !DIR);
	}
	else
	{
		M1_DIR == FORWARD ? digitalWrite(DUAL_DIRPIN[0], DIR) : digitalWrite(DUAL_DIRPIN[0], !DIR);
		M2_DIR == FORWARD ? digitalWrite(DUAL_DIRPIN[1], DIR) : digitalWrite(DUAL_DIRPIN[1], !DIR);
	}


}
void Stepper::step()
{
	///execute step
	if (DUAL_STEPPER == false)
	{
		digitalWrite(STEPPIN, LOW);
		digitalWrite(STEPPIN, HIGH);
	}
	else
	{
		digitalWrite(DUAL_STEPPIN[0], LOW);
		digitalWrite(DUAL_STEPPIN[1], LOW);
		digitalWrite(DUAL_STEPPIN[0], HIGH);
		digitalWrite(DUAL_STEPPIN[1], HIGH);
	}

	/// log position change
	if (DIR == FORWARD)
	{
		RPOS++;
	}
	else
	{
		RPOS--;
	}
}
void Stepper::setVEL(double max_velocity)
{
	MAX_VEL = max_velocity;
	MAX_STEP_VELOCITY = MAX_VEL * STEPS_PER_RAD;
}
void Stepper::setACC(double max_acceleration)
{
	MAX_ACC = max_acceleration;
	MAX_STEP_VELOCITY = MAX_ACC * STEPS_PER_RAD;
}

/// Motion
void Stepper::PTP_update()                                             
{
	if (RPOS == SETPOINT)
	{
		//printData();
		//Serial.println("inpos");
		RACC = 0;
		RVEL = 0;
		INPOS = true;
		decelerationPhase = false;
		return;
	}

	else if (RPOS != SETPOINT)
	{
		 
		// calculate current stopping distance under constant deceleration
		double _rvel = (double)RVEL;
		double _max_step_acc = (double)MAX_STEP_ACCELERATION;

		long int stoppingDistance = (long int)(1.5*((_rvel*_rvel) / _max_step_acc));

		// acceleration loop. runs once every predetermined interval
		if (micros() - accTimer >= acc_loop_dt)		
		{
			/// check if it is time to start deceleration
			if (abs(RPOS - SETPOINT) <= stoppingDistance)
			{
				/// This boolean prevents system from toggeling between 
				/// acceleration and deceleration during deceleration phase 
				/// since deceleration distance is dependant on current velocity
				decelerationPhase = true;	
			}

		
			// acceleration phase. 
			//in this phase system is accelerated in a constant rate
			if (abs(RPOS - SETPOINT) > stoppingDistance && RVEL < MAX_STEP_VELOCITY && decelerationPhase == false)
			{
				RACC = MAX_STEP_ACCELERATION;

				double _racc = (double)RACC;
				double _acc_loop_dt = acc_loop_dt;
				
				RVEL = RVEL + (long int)(_racc * _acc_loop_dt / 1000000);
			}

			// constant velocity phase
			// in this phase system moves in constant velocity 
			else if (RVEL >= MAX_STEP_VELOCITY && decelerationPhase == false)
			{
				RACC = 0;
				RVEL = MAX_STEP_VELOCITY;
			}

			// deceleration phase
			else if (abs(SETPOINT - RPOS) <= stoppingDistance && decelerationPhase == true)
			{
				/// in this phase system decelerates. a "decelerationPhase" boolean prevents system from
				/// toggeling in and out of this phase due to velocity change
				RACC = MAX_STEP_ACCELERATION* -1;

				double _racc = (double)RACC;
				double _acc_loop_dt = acc_loop_dt;

				RVEL = RVEL + (long int)((_racc * _acc_loop_dt) / 1000000);

				/// prevention of stopping before position. slow advance up to point than 
				/// immediete stop this is due to the fact that acceleration might get a 
				/// negative value because deceleration rate is fixed and independant
				/// of position error. unlike servo control
				if (RVEL < SLOW_ADVANCE_SPEED)
				{
					RVEL = SLOW_ADVANCE_SPEED;
				}
			}
			setDirection();
			step_loop_dt = (unsigned long int)((1 / (double)RVEL) * 1000000);  /// Determine time interval between steps		
			accTimer = micros();	/// clock for next acceleration calculation iteration
		}
		
		// step scheduler. executes the stepping cycle for the motor
		if (micros() > stepTimer + step_loop_dt)
		{
			step();
			stepTimer = micros();
		}
	}
	
}

/// calculations
void Stepper::inputSetpointRad(double setpoint)
{
	INPOS = false;
	decelerationPhase = false;
	SETPOINT_RAD= setpoint;
	SETPOINT = setpoint * STEPS_PER_RAD;
	Serial.println("set point set to: " + String(SETPOINT) + "Steps" + "\n RPOS: " + String(RPOS));
}
void Stepper::inputSetpointDeg(double setpoint)
{
	INPOS = false;
	decelerationPhase = false;
	SETPOINT_RAD = setpoint * STEPS_PER_DEG;
	Serial.println("set point set to: " + String(SETPOINT) + "\n RPOS: " + String(RPOS));
}
double Stepper::rad2steps(double rad)
{
	//return (rad / (2*PI)) * REDUCTION_RATIO * STEPS_PER_REVOLUTION;
	return rad * STEPS_PER_RAD;
}

/// Debug
void Stepper::printData()
{
	Serial.print("Setpoint: " + String(SETPOINT) + "   ");
	Serial.print("RPOS: " + String(RPOS) + "   ");
	//Serial.print("RVEL: " + String(RVEL) + "   ");
	//Serial.print("RACC: " + String(RACC) + "   ");
	Serial.print("dt_step: " + String(step_loop_dt) + "   ");
	//Serial.print("DIR: "+String(DIR)+"   ");
	Serial.print("micros: " + String(micros()) + "   ");
	Serial.println();
}

