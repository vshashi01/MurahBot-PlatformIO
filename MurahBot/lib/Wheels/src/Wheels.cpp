#include <Arduino.h>
#include <DigitalIO.h>
#include "Wheels.h"

//default constructor: upon instantiation all the pin values and absolute speeds are stored in class variables 
Wheel::Wheel(int pin1, int pin2, int pinSetSpeed, int minWheelAbsoluteSpeed, int maxWheelAbsoluteSpeed)
	:_pinForward(pin1), _pinBackward(pin2), _pinSetSpeed(pinSetSpeed), 
	_minWheelAbsoluteSpeed(minWheelAbsoluteSpeed), _maxWheelAbsoluteSpeed(maxWheelAbsoluteSpeed)  {
	initWheel();
	
}

int Wheel::_numberOfWheels = 0;

//copy constructor 
Wheel::Wheel(const Wheel& AWheel) {
	_pinForward = AWheel._pinForward;
	_pinBackward = AWheel._pinBackward;
	_pinSetSpeed = AWheel._pinSetSpeed;
	_minWheelAbsoluteSpeed = AWheel._minWheelAbsoluteSpeed;
	_maxWheelAbsoluteSpeed = AWheel._maxWheelAbsoluteSpeed;	
}

//called during instantiation automatically, sets-up the pins and state variables to appropriate states 
void Wheel::initWheel() {
	_spinState = WHEEL_NO_SPIN;
	_pinForward.mode(OUTPUT);
	_pinBackward.mode(OUTPUT);
	pinMode(_pinSetSpeed, OUTPUT); //the analog pin

	_numberOfWheels++;
}

// returns _spinState
Wheel::WheelState Wheel::getCurrentWheelState() {
	return _spinState;
}

//set spin Forward 
void Wheel::setSpinForward(int speed) {
	speed = _limitWheelSpeed(speed);
	_pinForward.high();
	_pinBackward.low();
	analogWrite(_pinSetSpeed, speed);
	_spinState = WHEEL_SPIN_FORWARD;
}

//set spin Backward 
void Wheel::setSpinBackward(int speed) {
	speed = _limitWheelSpeed(speed);
	_pinBackward.high();
	_pinForward.low();
	analogWrite(_pinSetSpeed, speed);
	_spinState = WHEEL_SPIN_BACKWARD;
}

//set spin Stop
void Wheel::setSpinStop() {
	_pinForward.low();
	_pinBackward.low();
	analogWrite(_pinSetSpeed, 0);
	_spinState = WHEEL_NO_SPIN;
}

// return _max/_min Wheel Absolute Speeds 
int Wheel::getWheelAbsoluteSpeed(MinMaxRange rangeValue) {
	if (rangeValue == MIN)
		return _minWheelAbsoluteSpeed;
	else if (rangeValue == MAX)
		return _maxWheelAbsoluteSpeed;
	else
		return -1;
}

//resets the Absolute Speed values 
void Wheel::setWheelAbsoluteSpeed(int minSpeedAbsolute, int maxSpeedAbsolute) {
	_minWheelAbsoluteSpeed = minSpeedAbsolute;
	_maxWheelAbsoluteSpeed = maxSpeedAbsolute;
}

//method to ensure that we never program the speed outside of the allowable range
int Wheel::_limitWheelSpeed(int wheelSpeed) {
	if (wheelSpeed > _maxWheelAbsoluteSpeed) wheelSpeed = _maxWheelAbsoluteSpeed;
	else if (wheelSpeed < _minWheelAbsoluteSpeed) wheelSpeed = _minWheelAbsoluteSpeed;
	else wheelSpeed = wheelSpeed;
	return wheelSpeed;
}


//default constructor for 4 wheel drives robot 
//stores the address of each wheel objects to local wheel object pointer for easy access 
//stores the speed tolerance range to set the min and max drive speed
Drive4Wheel::Drive4Wheel(Wheel& LeftFrontWheel, Wheel& RightFrontWheel,
	Wheel& LeftRearWheel, Wheel& RightRearWheel, int speedToleranceRange)
	:_LeftFrontWheel(&LeftFrontWheel), _RightFrontWheel(&RightFrontWheel),
	_LeftRearWheel(&LeftRearWheel), _RightRearWheel(&RightRearWheel), _speedToleranceRange(speedToleranceRange) {
	initDrive4Wheel();
}


//called during the instantiation pf the Drive4Wheel class 
void Drive4Wheel::initDrive4Wheel() {
	_setDriveSpeed();
}

//returns drive speed 
int Drive4Wheel::getDriveSpeed(MinMaxRange rangeValue) {
	if (rangeValue == MIN)
		return _minDriveSpeed;
	else if (rangeValue == MAX)
		return _maxDriveSpeed;
	else
		return -1;
}

//returns the tolerance value for speed ranges
int Drive4Wheel::getSpeedToleranceRange() {
	return _speedToleranceRange;
}

//sets the tolerance value for speed ranges
void Drive4Wheel::setSpeedToleranceRange(int speedTolerance) {
	_speedToleranceRange = speedTolerance;
	_setDriveSpeed();
}

//checks the input speed and ensures that the speed is capped to the max or min Drive speeds allowed for the Drive4Wheel class 
//not invoked by any other methods of Drive4Wheel class (made available for users of Drive4Wheel class)
int Drive4Wheel::limitDriveSpeed(int driveSpeed) {
	if (driveSpeed > _maxDriveSpeed)driveSpeed = _maxDriveSpeed;
	else if (driveSpeed < _minDriveSpeed)driveSpeed = _minDriveSpeed;
	else driveSpeed = driveSpeed;
	return driveSpeed;
}

//Drive4Wheel methods for driving 
void Drive4Wheel::goForward(int wheelSpeed) {
	_LeftFrontWheel->setSpinForward(wheelSpeed);
	_LeftRearWheel->setSpinForward(wheelSpeed);
	_RightFrontWheel->setSpinForward(wheelSpeed);
	_RightRearWheel->setSpinForward(wheelSpeed);

	_driveState = DRIVE_FORWARD;
}

void Drive4Wheel::goBackward(int wheelSpeed) {
	_LeftFrontWheel->setSpinBackward(wheelSpeed);
	_LeftRearWheel->setSpinBackward(wheelSpeed);
	_RightFrontWheel->setSpinBackward(wheelSpeed);
	_RightRearWheel->setSpinBackward(wheelSpeed);

	_driveState = DRIVE_BACKWARD;
}

void Drive4Wheel::goLeft(int wheelSpeed, float speedRatio) {
	_RightFrontWheel->setSpinForward(wheelSpeed);
	_RightRearWheel->setSpinForward(wheelSpeed);
	_LeftFrontWheel->setSpinBackward(wheelSpeed*speedRatio);
	_LeftRearWheel->setSpinBackward(wheelSpeed*speedRatio);

	_driveState = DRIVE_LEFT;
		
}

void Drive4Wheel::goRight(int wheelSpeed, float speedRatio) {

	_LeftFrontWheel->setSpinForward(wheelSpeed);
	_LeftRearWheel->setSpinForward(wheelSpeed);
	_RightFrontWheel->setSpinBackward(wheelSpeed*speedRatio);
	_RightRearWheel->setSpinBackward(wheelSpeed*speedRatio);

	_driveState = DRIVE_RIGHT;
	
}

void Drive4Wheel::swayLeft(int wheelSpeed, float speedRatio, bool reverse) {
	if (reverse == true) {
		_LeftFrontWheel->setSpinBackward(wheelSpeed*speedRatio);
		_LeftRearWheel->setSpinBackward(wheelSpeed*speedRatio);
		_RightFrontWheel->setSpinBackward(wheelSpeed);
		_RightRearWheel->setSpinBackward(wheelSpeed);

		_driveState = DRIVE_BACKWARD_LEFT;
	}
	else {
		_LeftFrontWheel->setSpinForward(wheelSpeed*speedRatio);
		_LeftRearWheel->setSpinForward(wheelSpeed*speedRatio);
		_RightFrontWheel->setSpinForward(wheelSpeed);
		_RightRearWheel->setSpinForward(wheelSpeed);

		_driveState = DRIVE_FORWARD_LEFT;
	}
}

void Drive4Wheel::swayRight(int wheelSpeed, float speedRatio, bool reverse) {
	if (reverse == true) {
		_LeftFrontWheel->setSpinBackward(wheelSpeed);
		_LeftRearWheel->setSpinBackward(wheelSpeed);
		_RightFrontWheel->setSpinBackward(wheelSpeed*speedRatio);
		_RightRearWheel->setSpinBackward(wheelSpeed*speedRatio);

		_driveState = DRIVE_BACKWARD_RIGHT;
	}
	else {
		_LeftFrontWheel->setSpinForward(wheelSpeed);
		_LeftRearWheel->setSpinForward(wheelSpeed);
		_RightFrontWheel->setSpinForward(wheelSpeed*speedRatio);
		_RightRearWheel->setSpinForward(wheelSpeed*speedRatio);

		_driveState = DRIVE_FORWARD_RIGHT;
	}
}

void Drive4Wheel::stop() {
	_LeftFrontWheel->setSpinStop();
	_LeftRearWheel->setSpinStop();
	_RightFrontWheel->setSpinStop();
	_RightRearWheel->setSpinStop();

	_driveState = DRIVE_STOP;
}

//Drive4Wheels method to identify the drive state of the drive systems
Drive4Wheel::DriveState Drive4Wheel::getCurrentDriveState() {
	return _driveState;	
}

//private method to update the drive speed values
void Drive4Wheel::_setDriveSpeed() {
	_minDriveSpeed = max(_LeftFrontWheel->getWheelAbsoluteSpeed(MIN), max(_RightFrontWheel->getWheelAbsoluteSpeed(MIN),
		max(_LeftRearWheel->getWheelAbsoluteSpeed(MIN), _RightRearWheel->getWheelAbsoluteSpeed(MIN))));
	_maxDriveSpeed = min(_LeftFrontWheel->getWheelAbsoluteSpeed(MAX), min(_RightFrontWheel->getWheelAbsoluteSpeed(MAX),
		min(_LeftRearWheel->getWheelAbsoluteSpeed(MAX), _RightRearWheel->getWheelAbsoluteSpeed(MAX))));
	//the minimum and maximum drivespeeds are evaluated from each absolute speed values of the wheels. 
	_minDriveSpeed = _minDriveSpeed + _speedToleranceRange;
	_maxDriveSpeed = _maxDriveSpeed - _speedToleranceRange;
	int biggerValue;
	if (_minDriveSpeed > _maxDriveSpeed) {
		biggerValue = _minDriveSpeed;
		_minDriveSpeed = _maxDriveSpeed;
		_maxDriveSpeed = biggerValue;
	}//checks if the tolerance value given causes the values to reach an incorrect range 
	 //if yes the higher and lower values are reset appropriately minDriveSpeed < maxDriveSpeed 
	else;

}

