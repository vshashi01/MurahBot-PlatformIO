


#include <Arduino.h>
#include <Wheels.h>

#define _TASK_TIMEOUT

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <Bounce2.h>
#include <DigitalIO.h>
#include <EnableInterrupt.h>

#define BLYNK_USE_DIRECT_CONNECT
#define MurahBotBT Serial3

#include <BlynkSimpleSerialBLE.h>

///////////////////////////////////////////////////////////////////////////
//Bluetooth and Blynk related declarations (if any)
char auth[] = "66390b83798e4495aa9d6c23724f2181"; //Blynk Authorization code

///////////////////////////////////////////////////////////////////////////
#define RearLeftEncoderPin 20 //test pinchange interrupt
#define RearRightEncoderPin 21
#define FrontLeftEncoderPin 18
#define FrontRightEncoderPin 19



volatile int RearLeftWheelTurns = 0;
volatile int RearRightWheelTurns = 0;
volatile int FrontLeftWheelTurns = 0;
volatile int FrontRightWheelTurns = 0;

volatile bool counting = false;
unsigned long startTime;

///////////////////////////////////////////////////////////////////////////
// Wheel class and Drive4Wheel class instantiation 
Wheel WheelFrontLeft(42, 43, 11);  //initializing each wheels with (forwardPin, backwardPin, speedPin)
Wheel WheelFrontRight(38, 39, 10);
Wheel WheelRearLeft(30, 31, 9);
Wheel WheelRearRight(34, 35, 8);

int speedTolerance = 30; //range of tolerance for drive speeds
Drive4Wheel murahDrive(WheelFrontLeft, WheelFrontRight,
	WheelRearLeft, WheelRearRight, speedTolerance); //initializing the Drive class with all drive system methods

//////////////////////////////////////////////////////////////////////////////////////////////////


// Button Pin initialization and Bounce class instantiation 
const byte buttonPinRobotStartStop = 22; //Start and Stop Robot 
DigitalPin<buttonPinRobotStartStop> pinRobotStartStop;
										 //const byte buttonPinBackward = 23;
										 //const byte buttonPinLeft = 24;
										 //const byte buttonPinRight = 25;  //phased out to be repurposed in the future 

Bounce ButtonRobotStartStop(buttonPinRobotStartStop, 10);


//////////////////////////////////////////////////////////////////////////////////////////////////////////



enum SystemStates {
	ACTIVE, PASSIVE, NO_STATE
};
//state variables
SystemStates prevSystemState = NO_STATE;
SystemStates currSystemState = PASSIVE; // 
SystemStates currBlynkState = PASSIVE;
DigitalPin<13> ledPin;

///////////////////////////////////////////////////////////////////////////////
//function prototypes 

void callbackButtonState();
void callbackButtonAction(); //callbacks for button actions

bool onEnableOfEnableDisableDrive();
void callbackEnableDisableDrive(); //callback to ON and OFF drive system

bool onEnableBlynk();
void callbackBlynk(); //callback for Blynk connection 
void callbackCheckBlynkConnection();

void callbackPrimaryJoystickDrive();
void callbackSecondaryJoystickDrive();
void callbackDisplayDriveState(); //callback to Drive system 

void callbackattachInterrupt();
void callbackdetachInterrupt();
bool onEnableInterrupt();
void onDisableInterrupt();

void RearLeftWheelTurnsCount();
void RearRightWheelTurnsCount();
void FrontLeftWheelTurnsCount();
void FrontRightWheelTurnsCount();

///////////////////////////////////////////////////////////////////////////////
// Scheduler and Tasks instantiation
Scheduler MurahBotSchedule;
Task taskUpdateButton(TASK_IMMEDIATE, TASK_FOREVER, &callbackButtonState, &MurahBotSchedule);
Task taskEnableDisableDrive(TASK_IMMEDIATE, TASK_ONCE, &callbackEnableDisableDrive, &MurahBotSchedule, false);
Task taskDrive(50, TASK_FOREVER, &callbackPrimaryJoystickDrive, &MurahBotSchedule, false);
Task taskRunBlynk(TASK_IMMEDIATE, TASK_FOREVER, &callbackBlynk, &MurahBotSchedule, false, &onEnableBlynk);

Task interruptCount(TASK_IMMEDIATE, TASK_ONCE, &callbackattachInterrupt, &MurahBotSchedule, false, &onEnableInterrupt, &onDisableInterrupt);




void setup() {
	pinMode(RearLeftEncoderPin, INPUT_PULLUP);
	pinMode(RearRightEncoderPin, INPUT_PULLUP);
	pinMode(FrontLeftEncoderPin, INPUT_PULLUP);
	pinMode(FrontRightEncoderPin, INPUT_PULLUP);

	pinRobotStartStop.mode(INPUT_PULLUP);
    //pinMode(buttonPinRobotStartStop, INPUT_PULLUP); //initialize the button
	Serial.begin(115200);
	delay(500);
	MurahBotBT.begin(115200); //starts the BLE module 
	delay(100);

	//enabling the Tasks
	taskUpdateButton.enable();

	//on board LED 
	ledPin.mode(OUTPUT);
	ledPin.low();   
}

void loop() {
    MurahBotSchedule.execute();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// updates various button states  
void callbackButtonState() {
	if (ButtonRobotStartStop.update()) {
		if (ButtonRobotStartStop.read() == HIGH) {
			if (prevSystemState == NO_STATE) {
				prevSystemState = currSystemState;
				return;
			}
			else if (prevSystemState == currSystemState) {
				taskEnableDisableDrive.enable();
				return;
			}
			else {
				taskEnableDisableDrive.restart();
				return;
			}
		}
		else;
	}
	else;
	taskUpdateButton.setCallback(&callbackButtonAction);
	//Force next iteration callbackButtonAction if needed 
}

//updates the Button Actions after the Button status are updated //if neccessary
void callbackButtonAction() {
	taskUpdateButton.setCallback(&callbackButtonState);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//ensures the callbackEnableDisableDrive run in the next iteration
bool onEnableOfEnableDisableDrive() {
	taskEnableDisableDrive.setCallback(&callbackEnableDisableDrive);
	taskEnableDisableDrive.forceNextIteration();
	return true;
}

// turns the driveSystem ON and OFF
void callbackEnableDisableDrive() {
	if (currSystemState == PASSIVE) {
		prevSystemState = currSystemState;
		currSystemState = ACTIVE; //changes the system state 
		//Serial.println(F("Bringing drive systems online...."));
		taskDrive.enable();
		if (currBlynkState == PASSIVE)taskRunBlynk.enable(); //enable only once 
	}
	else {
		prevSystemState = currSystemState;
		currSystemState = PASSIVE;
		//Serial.println(F("Shutting down drive systems..."));
		murahDrive.stop(); //force stop the robot
		taskDrive.disable();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//ensures the Blynk app is connected 
bool onEnableBlynk() {
	Blynk.begin(MurahBotBT, auth);
	currBlynkState = ACTIVE;
	return true;
}

void callbackBlynk() {
	Blynk.run();
	taskRunBlynk.setCallback(callbackCheckBlynkConnection);
}

void callbackCheckBlynkConnection() {
	if (!Blynk.connected()) {
		murahDrive.stop();
	}
	taskRunBlynk.setCallback(callbackBlynk);
}

////////////////////////////////////////////////////////////////////////////////
//drive control variables and functions 
//to be ported to a separate .ino file in the future

//constant values for jostick pad region
const byte X_THRESHOLD_LOW = 88; //X: 128 - 40
const byte X_THRESHOLD_HIGH = 168; //X: 128 + 40   
const byte Y_THRESHOLD_LOW = 88;
const byte Y_THRESHOLD_HIGH = 168;

// initialized the X and Y values to the center position 127
int joystickX = 127;
int joystickY = 127;

//ratio range for speedRatios as a multiple of 100 //should not do floating point calculation
float smallestRatio = 45;
float biggestRatio = 60;

//static variables for the Drive Speeds 
static int minSpeed = murahDrive.getDriveSpeed(MIN);
static int maxSpeed = murahDrive.getDriveSpeed(MAX);
//everytime drive Speeds are changed in runtime, 
//call these lines immediately to update the variable 

//Blynk input of joystick values 
BLYNK_WRITE(V1) {
	joystickX = param[0].asInt();
	joystickY = param[1].asInt();
}

//updates the Primary controls for Joystick in four directions: Front, Back, Left, Right. 
void callbackPrimaryJoystickDrive() {
	//boolean conditions for the threshold regions
	bool joystickXInThreshold = (joystickX > X_THRESHOLD_LOW && joystickX < X_THRESHOLD_HIGH);
	bool joystickYInThreshold = (joystickY > Y_THRESHOLD_LOW && joystickY < Y_THRESHOLD_HIGH);

	//if joystick not in the threshold region sets callback to secondaryJoystickDrive
	if (!joystickXInThreshold && !joystickYInThreshold) {
		taskDrive.setCallback(&callbackSecondaryJoystickDrive);
		return;
	}

	// temp variable 
	int speed;

	//various statement to check the appropriate Primary actions
	if (joystickXInThreshold && joystickYInThreshold) {
		murahDrive.stop();	//stop the drive if in absolute center	
	}
	else if (joystickY < Y_THRESHOLD_LOW && joystickXInThreshold) {
		speed = map(joystickY, Y_THRESHOLD_LOW, 0, minSpeed, maxSpeed); //map speed between joystick coordinate and allowable drive Speeds
		murahDrive.goBackward(speed);
	}
	else if (joystickY > Y_THRESHOLD_HIGH && joystickXInThreshold) {
		speed = map(joystickY, Y_THRESHOLD_HIGH, 255, minSpeed, maxSpeed);
		murahDrive.goForward(speed);
	}
	else if (joystickX < X_THRESHOLD_LOW && joystickYInThreshold) {
		speed = map(joystickX, X_THRESHOLD_LOW, 0, minSpeed, maxSpeed);
		murahDrive.goLeft(speed);
	}
	else if (joystickX > X_THRESHOLD_HIGH && joystickYInThreshold) {
		speed = map(joystickX, X_THRESHOLD_HIGH, 255, minSpeed, maxSpeed);
		murahDrive.goRight(speed);
	}
	else murahDrive.stop(); //just in case call stop always 

	taskDrive.setCallback(&callbackDisplayDriveState);
}

//updates the Secondary controls for Joystick: Sway Left and Right in Forward and Backward
void callbackSecondaryJoystickDrive() {
	
	//temp variables
	int speed;
	float swayRatio = 0.45;

	//various statements to check for the Secondary conditions
	if (joystickX < X_THRESHOLD_LOW && joystickY > Y_THRESHOLD_HIGH) {
		speed = map(joystickY, Y_THRESHOLD_HIGH, 255, minSpeed, maxSpeed);
		murahDrive.swayLeft(speed, swayRatio, false);  
	}
	else if (joystickX < X_THRESHOLD_LOW && joystickY < Y_THRESHOLD_LOW) {
		speed = map(joystickY, Y_THRESHOLD_LOW, 0, minSpeed, maxSpeed);
		murahDrive.swayLeft(speed, swayRatio, true);
	}
	else if (joystickX > X_THRESHOLD_HIGH && joystickY > Y_THRESHOLD_HIGH) {
		speed = map(joystickY, Y_THRESHOLD_HIGH, 255, minSpeed, maxSpeed);
		murahDrive.swayRight(speed, swayRatio, false);
	}
	else if (joystickX > X_THRESHOLD_HIGH && joystickY < Y_THRESHOLD_LOW) {
		speed = map(joystickY, Y_THRESHOLD_LOW, 0, minSpeed, maxSpeed);
		murahDrive.swayRight(speed, swayRatio, true);
	}
	else murahDrive.stop();

	taskDrive.setCallback(&callbackDisplayDriveState);

}

//update drive state to be changes to update on Blynk app in future
void callbackDisplayDriveState() {
	//Serial.print("Drive State: ");
	//Serial.println(murahDrive.getCurrentDriveState());

	
	if (counting == false && murahDrive.getCurrentDriveState() != murahDrive.DRIVE_STOP){
		
		interruptCount.enable();
	}
	taskDrive.setCallback(&callbackPrimaryJoystickDrive);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//interrupt systems
int RearLeftWheelCurrentTurnCount;
int RearRightWheelCurrentTurnCount;
int FrontLeftWheelCurrentTurnCount;
int FrontRightWheelCurrentTurnCount;

//inside the interrupt
void RearLeftWheelTurnsCount() {
	RearLeftWheelTurns++;
}

void RearRightWheelTurnsCount() {
	RearRightWheelTurns++;
}

void FrontLeftWheelTurnsCount() {
	FrontLeftWheelTurns++;
}

void FrontRightWheelTurnsCount() {
	FrontRightWheelTurns++;
}

bool onEnableInterrupt() {
	Serial.println("Enabled Task");	
	interruptCount.setCallback(&callbackattachInterrupt);
	return true;
}

void callbackattachInterrupt() {
	counting = true;
    startTime = millis();
	enableInterrupt(RearLeftEncoderPin, &RearLeftWheelTurnsCount, RISING);
	enableInterrupt(RearRightEncoderPin, &RearRightWheelTurnsCount, RISING);
	enableInterrupt(FrontLeftEncoderPin, &FrontLeftWheelTurnsCount, RISING);
	enableInterrupt(FrontRightEncoderPin, &FrontRightWheelTurnsCount, RISING);
	

	interruptCount.setCallback(&callbackdetachInterrupt);
	interruptCount.forceNextIteration();
}

void runInterrupt(unsigned long aTimeout){
	if (counting) return;

	interruptCount.setInterval(aTimeout);
	interruptCount.restartDelayed();

	
}

void callbackdetachInterrupt() {
	if ((millis() - startTime) > 1000) {
		
		disableInterrupt(RearLeftEncoderPin);
		disableInterrupt(RearRightEncoderPin);
		disableInterrupt(FrontLeftEncoderPin);
		disableInterrupt(FrontRightEncoderPin);
		
		interruptCount.disable();
	}

	interruptCount.setCallback(&callbackdetachInterrupt);
}



void  onDisableInterrupt() {
	Serial.println("Disabled Task");
	counting = false;
	RearLeftWheelCurrentTurnCount = RearLeftWheelTurns;
	RearLeftWheelTurns = 0;

	RearRightWheelCurrentTurnCount = RearRightWheelTurns;
	RearRightWheelTurns = 0;

	FrontLeftWheelCurrentTurnCount = FrontLeftWheelTurns;
	FrontLeftWheelTurns = 0;

	FrontRightWheelCurrentTurnCount = FrontRightWheelTurns;
	FrontRightWheelTurns = 0;
		
	Blynk.virtualWrite(V2, RearLeftWheelCurrentTurnCount);
	Blynk.virtualWrite(V3, RearRightWheelCurrentTurnCount);
	Blynk.virtualWrite(V4, FrontLeftWheelCurrentTurnCount);
	Blynk.virtualWrite(V5, FrontRightWheelCurrentTurnCount);

	return;
}

