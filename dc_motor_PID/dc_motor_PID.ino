

#include "Control.h"

Control motor;


/************************
Global Variables
*************************/
bool encAState, encBState;	//Encoder channel states
const int encA = 2;	//Pins for encoders
const int encB = 3;	//Pins for encoders

long count;	//Global variables to track position of motors (encoders)

float adcBits = 10.0;
float pwmBits = 8.0;
float encoderCPR = 4741.44;

float ang = 0;

void setup() {
  Serial.begin(9600);
  
  //Global Setup
	count = 0;	//Initialize encoder count
	pinMode(encA, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	encAState = digitalRead(encA);	//Initialize encoder state
	encBState = digitalRead(encB);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(encA), encA_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(encB), encB_changed, CHANGE);	//Setup interrupt for background proccessing
	
	motor.set_precision(pwmBits, adcBits);

}

void loop()
{

	const int motorPwmPin = 5;	//Define pins for motor PWM
	const int motorDirPin = 4;	//Define pins for motor direction
	
	unsigned long lastTime = millis();	//Initialize sampeling timing
	unsigned long lastPrintTime = millis();
	unsigned long loopTime;
	
	float deg, deg_c, deg_d1, control;	//Define all the float variables
	
	const int Ts = 10; //Sample period in milliseconds
	const float alpha = .05;	//digital LPF coefficent
	
	const float intThreshHigh = 3.0;	//Threshold for using integrator (deg)
	const float intThreshLow = .25;		//Threshold for using integrator (deg)
	
	bool flag = true;	//Is this the first time through?
	
	//Conditions for Kp calculation
	const float maxErrorDeg = 15.0;
	
	//Constant variables for PID algorithm
	const float kp = 1.0/maxErrorDeg;
	const float ki = 0.0;
	const float kd = 0.01;
	
	motor.set_gains(kp, ki, kd);
	motor.set_pins(motorPwmPin, motorDirPin);
	motor.set_antiwindup(intThreshHigh, intThreshLow);
	motor.set_time_filter(Ts/1000, alpha);
	
	while(1)
	{
		
		if (Serial.available())	//Serial Data is in the buffer...
		{
			Serial.println("Read New Command");
			serial_parser();
			Serial.println("New Command");
		}		
		
		loopTime = millis() - lastTime;
		if( loopTime >= Ts)
		{
			lastTime = millis();
			deg = (float(count)*360.0)/encoderCPR;
			deg_c = ang;
			control = motor.pid(deg, deg_c, flag);
			digitalWrite(motorDirPin, direction(control));
			analogWrite(motorPwmPin, (pow(2.0,pwmBits)-1)*abs(control));
			if(flag)
			{
				flag = !flag;
			}
			deg_d1 = deg;
		}
		
		if((millis() - lastPrintTime >= 1000))
		{
			Serial.print("Motor Command: "); Serial.println(deg_c);
			Serial.print("Motor Position: "); Serial.println(deg);
			Serial.print("Motor Control: "); Serial.println(control*100.0);
			Serial.println("");
			lastPrintTime = millis();
		}
	}	

}

void encA_changed()
{
    encAState = digitalRead(encA);
	
    if(encAState)  //A changed to HIGH
    {
        if(encBState)  //B is HIGH
        {
            count--;
        }
        else	//B is LOW
        {
            count++;
        }
    }
    else	//A changed to LOW
    {
        if(encBState)	//B is HIGH
        {
            count++;
        }
        else //B is LOW
        {
            count--;
        }
    }
}

void encB_changed()
{
    encBState = digitalRead(encB);
	
    if(encBState)  //B changed to HIGH
    {
        if(encAState)  //A is HIGH
        {
            count++;
        }
        else  //A is LOW
        {
            count--;
        }
    }
    else  //B changed to LOW
    {
        if(encAState)  //A is HIGH
        {
            count--;
        }
        else  //A is LOW
        {
            count++;
        }
    }
}

bool direction(float control)
{
	bool dir;
	
	if(control >= 0)
	{
		dir = 0;
	}
	else
	{
		dir = 1;
	}
	return(dir);
}

void serial_parser()
{
	float angle_c = Serial.parseFloat();
	
	ang = angle_c;

	Serial.flush();
}