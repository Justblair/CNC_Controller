#include "PID_v1.h"
#include "U8glib.h"

/*Pin Definitions*/
const int LEDPin = 13;
const int ACSPin = A3;
const int PotPin = A2;
const int VFDPin = A0;
const int AutoVFDPin = 4;
const int FeedBackPin = 2;
const int eStopPin = 7;
const int PWMPin = 9;

double input, output, setpoint;
double kp = 0.078, ki = 0.02, kd = 0.000; // 0.0002

double ulActRPM = 0;
double ulTargetRPM = 0;
double oldTargetRPM = 0;

int intPWM;

int counter = 0;

int VQ;

volatile bool resetState = 0;

static unsigned long bignum = 60000000;

char actRPM[20];
char targetRPM[10];
char strAmps[10];

volatile unsigned long lastTime = 0;
volatile unsigned long speed = 0;
volatile unsigned long lastSpeed = 0;

unsigned long resetMillis = 500;
unsigned long resetProtect = 0;

float current = 0;

bool mode = 0;

#define AUTO 1

U8GLIB_ST7920_128X64_1X u8g(10);   // CS Pin 10 (also Power)

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup(void) {

	pinMode(VFDPin, OUTPUT);
	pinMode(LEDPin, OUTPUT);
	pinMode(PotPin, INPUT);
	pinMode(AutoVFDPin, INPUT_PULLUP);
	pinMode(eStopPin, INPUT_PULLUP);
	pinMode(VFDPin, INPUT);
	pinMode(PWMPin, OUTPUT);

	myPID.SetMode(AUTOMATIC);
	// Attach interrupt to feedback sensor on motor
	attachInterrupt(digitalPinToInterrupt(FeedBackPin), countSpin, RISING);
	// aTTACH interrupt to stop the motor on E-stop
	attachInterrupt(digitalPinToInterrupt(eStopPin), shutDown, RISING);  


	// assign default color value
	if (u8g.getMode() == U8G_MODE_R3G3B2) {
		u8g.setColorIndex(255);     // white
	}
	else if (u8g.getMode() == U8G_MODE_GRAY2BIT) {
		u8g.setColorIndex(3);         // max intensity
	}
	else if (u8g.getMode() == U8G_MODE_BW) {
		u8g.setColorIndex(1);         // pixel on
	}
	else if (u8g.getMode() == U8G_MODE_HICOLOR) {
		u8g.setHiColorByRGB(255, 255, 255);
	}

	u8g.firstPage();
	do {
		splash();
	} while (u8g.nextPage());

	VQ = determineVQ(ACSPin);
	//Quiescent output voltage - the average voltage ACS712 shows with no load (0 A)

}

void loop(void) {

	// lets work out the ulActRPM

	if (lastTime + 680000 < micros()) {
		// If we have not had a pulse for a while, probably stopped
		ulActRPM = 0;
		lastTime = micros();
		speed = 0;
	}
	else {
		if (speed > 0) {
			ulActRPM = bignum / speed;
		}
		else {
			ulActRPM = 0;
		}
	}

	//myPID.SetTunings(kp,ki,kd);

	input = ulActRPM;
	setpoint = ulTargetRPM;

	myPID.Compute();

	//Serial.println(output);

	// The PID routine gets confused if ULTarget goes to zero with no feedback
	if (ulTargetRPM == 0) {
		output = 0;
	}

	if (resetState) {
		digitalWrite(PWMPin, LOW);  // If in reset set the pin low to switch off the motor
	}
	else {
		analogWrite(PWMPin, output); // Otherwise all is good, set the motor speed (result from the PID routine)
	}



	/*
	The switch routine ensures that we dont do all the non time critical behaviours on
	Each loop.  This is so that the PID routine is as responsive as posible
	*/
	switch (counter)
	{
	case 1:
		// Fetch the target rpm from the POT
		getTarget();
		break;
	case 2:
		// A few conversions for display later 
		ultoa(ulActRPM, actRPM, 10);
		dtostrf(current, 6, 2, strAmps);
		ultoa(ulTargetRPM, targetRPM, 10);

		intPWM = map(output, 0, 255, 64, 0);  // Used for the power 
		break;
	case 3:
		// picture loop
		u8g.firstPage();
		break;
	case 4:
		do {
			draw();
		} while (u8g.nextPage());
		break;
	case 5:
		current = readCurrent(ACSPin);
		break;

	default:
		counter = 0;
		break;
	}

	counter++;
}

void shutDown() {
	resetState = 1;
	digitalWrite(PWMPin, LOW);  // If in reset set the pin low to switch off the motor
}

void getTarget() {
	mode = digitalRead(AutoVFDPin);
	if (mode) {
		// Calculate the pulse width here
		int intVFDReading = analogRead(VFDPin);
	
			ulTargetRPM = map(intVFDReading, 0, 1024, 0, 14000);
			ulTargetRPM = (int)(ulTargetRPM / 100) * 100; // Set increments or else the target ends up jumpy and hard to set
	}

	else {
		// Map the reading from Analog 0
		int intPotReading = analogRead(PotPin);
		if (intPotReading < 950) {
			ulTargetRPM = map(intPotReading, 0, 950, 14000, 4000);
			ulTargetRPM = (int)(ulTargetRPM / 100) * 100; // Set increments or else the target ends up jumpy and hard to set
		}
		else {
			ulTargetRPM = 0;
		}
		if (ulTargetRPM > oldTargetRPM) {
			/*  If we ask the motor to increase its speed suddently, we may get a brief current surge
				and we dont want the circuit protection to cut in (ie switch off the motor)
				So each time the target rpm is increased we wait a bit before panicing about over
				current readings
			*/
			resetProtect = millis() + resetMillis;
		}
		oldTargetRPM = ulTargetRPM;
	}
}

void countSpin() {
	/*	This is the interupt routine, it gives me a raw time reading for a spindle revolution
		I will convert it to rpm later, for now I want the routine to be as small as possible
	*/
	lastSpeed = speed;
	speed = (micros() - lastTime);
	lastTime = micros();
	if (speed < 3000) speed = lastSpeed;
}

int determineVQ(int PIN) {
	// estimating avg. quiscent voltage
	long VQ = 0;
	//read 5000 samples to stabilise value
	for (int i = 0; i < 1000; i++) {
		VQ += analogRead(PIN);
		delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
	}
	VQ /= 1000;
	return int(VQ);
}

float readCurrent(int PIN) {
	int current = 0;
	int sensitivity = 100.0; //change this to 100 for ACS712-20A or to 66 for ACS712-30A
	//read 5 samples to stabilise value
	for (int i = 0; i < 10; i++) {
		current += analogRead(PIN) - VQ;
		delay(1);
	}
	current = map(current / 10, 0, 1023, 0, 5000);
	/*	Check to see if we have too much current, but also that we have not had an upward change in 
		Target RPM recently*/
	if (current > (5 * sensitivity) && millis() > resetProtect) {
		resetState = 1;  // Not sure if we will get surges when tool hits the work piece
	}
	return float(current) / sensitivity;
}

void splash(void) {
	// graphic commands to redraw the complete screen should be placed here  
	//u8g.setFont(u8g_font_helvB14r);
	//u8g.drawStr( 0, 22, "Initialising");

}

void draw(void) {
	// graphic commands to redraw the complete screen should be placed here  
	u8g.setFont(u8g_font_helvR08r);
	u8g.drawStr(0, 10, "Actual");
	u8g.setFont(u8g_font_helvB10r);
	u8g.drawStr(0, 20, "rpm");

	u8g.drawHLine(0, 25, 120);
	u8g.drawHLine(0, 25 + 27, 120);
	u8g.setFont(u8g_font_helvB24r);
	u8g.drawStr(30 + (5 - strlen(actRPM)) * 18, 24, actRPM);

	u8g.setFont(u8g_font_helvR08r);
	u8g.drawStr(0, 12 + 25, "Target");
	u8g.setFont(u8g_font_helvB10r);
	u8g.drawStr(0, 22 + 25, "rpm");

	u8g.setFont(u8g_font_helvB24r);
	if (resetState) {
		u8g.drawStr(30, 24 + 27, "Reset!");
	}
	else {
		u8g.drawStr(30 + (5 - strlen(targetRPM)) * 18, 24 + 27, targetRPM);
	}

	u8g.setFont(u8g_font_helvR08r);

	u8g.drawStr(0, 62, "Mode:");
	if (mode == AUTO) {
		u8g.drawStr(30, 62, "VFD");
	}
	else {
		u8g.drawStr(29, 62, "Man");
	}

	u8g.drawLine(53, 52, 53, 64);
	u8g.drawStr(60, 62, "Amps:");
	u8g.drawLine(53, 52, 53, 64);
	u8g.drawStr(84, 62, strAmps);

	u8g.drawFrame(120, 0, 8, 64);
	u8g.drawBox(120, intPWM, 8, 64 - intPWM + 1);
}
