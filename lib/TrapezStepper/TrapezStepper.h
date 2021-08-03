
#ifndef TrapezStepper_h
#define TrapezStepper_h

#include "Arduino.h"

#define ACC_LOOP_FREQ		100			//	Hz

#define DEFAULT_MAX_VEL		6.28			//	deg/S
#define DEFAULT_MAX_ACC		3.14			//	deg/S^2
#define SLOW_ADVANCE_SPEED	0.05			//  Steps per second (not including reduction ratio)

class Stepper{
 protected:
	 
	 /// System specs
	 bool DUAL_STEPPER = false;
	 double STEPS_PER_REVOLUTION = 200;
	 double REDUCTION_RATIO = 1;
	 double STEPS_PER_RAD = (STEPS_PER_REVOLUTION * REDUCTION_RATIO) / (2*PI);
	 double STEPS_PER_DEG = (STEPS_PER_REVOLUTION * REDUCTION_RATIO) / 360;

	 /// motion values
	 long int MAX_STEP_VELOCITY, MAX_STEP_ACCELERATION;
	 bool decelerationPhase;
	 enum { acceleration, constantVelocity, deceleration };
	 enum { FORWARD, BACKWARD };

	 /// System pins
	 int STEPPIN, DIRPIN, ENABLEPIN;
	 int DUAL_STEPPIN[2], DUAL_DIRPIN[2], DUAL_ENABLEPIN[2];
	 int DIR;
	 int M1_DIR;
	 int M2_DIR;
	 
	 	
	 // timer values
	 unsigned long int step_loop_dt;
	 unsigned long int acc_loop_dt = (unsigned long int)(1000000 / ACC_LOOP_FREQ);
	 unsigned long int stepTimer, accTimer;



	 double rad2steps(double radian);
	 void printData();
	 void setDirection();
	 void step();
	 

 public:
	bool INPOS = false;
	long int RPOS = 0;		// units in steps
	long int RVEL = 0;		// units in steps
	long int RACC = 0;		// units in steps
	long int SETPOINT = 0;	// units in steps
	double SETPOINT_RAD, POSITION;			// units in rad
	double MAX_VEL = DEFAULT_MAX_VEL;
	double MAX_ACC = DEFAULT_MAX_ACC;
	 

	void init(int stepPin, int dirPin, int enablePin, bool invert_motor, double steps_per_revolution, double reduction_ratio);
	void initDual(int *stepPin, int *dirPin, int *enablePin,  bool *motorDirection, double steps_per_revolution, double reduction_ratio);
	void attach();
	void detach();
	void inputSetpointRad(double setpoint);
	void inputSetpointDeg(double setpoint);
	void PTP_update();
	void setVEL(double speed);
	void setACC(double acc);
};



#endif