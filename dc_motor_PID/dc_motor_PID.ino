/************************
This code implements PID control on position of a dc
 motor. State knowledge is based on quadrature encoder
 feedback.
 
 Hardware required for implementation
	Arduino
	H-bridge motor driver with PWM and DIR pins (e.g. Pololu G2 High-Power Motor Driver 24v13)
	DC motor with encoder with CPR>>360 (99:1 Metal Gear motor 25Dx54L mm HP 12V with 48 CPR Encoder)
		Note: The CPR of the recommended motor is effective 4741 after the gearbox.
	Optional: Logic level shifter. Most encoders use 5v, including the recommended. Your microcontroller
		may use 3.3v logic, and thus require a level shifter.
	
Wiring
	1)There should be four wires to each encoder (A, B, 5v, GND)
		A and B connect to the selected pins (see enc1A - enc2B)
		GND is commoned with the Arduino GND pin
		5v can come from any source (including Arduino) as long as GND of the source is commoned with logic
		
	3)Follow the H-bridge wiring scheme from the manufacturer
		see https://www.pololu.com/product/2992 for the H-bridge selected
		and https://www.pololu.com/product/3219 for the motor selected
	
Tuning
	The kp values indicated in this code were calculated to saturate (give full throttle) to the 
	motor when there is an error of 5 degrees. This is appropriate for generally continuous command
	strings provided the rate of command change is within the motor operating frequency. 
	
	ki and kd values are initially zeroed.
	
	If the rise-time (the time required for an initial error to become acceptably small, ignoring overshoot)
	is acceptably small, then kp should remain its default value.
	
	If there is an unacceptable level of overshoot or settling time increase the kd value in increments of .05
	until an the system settles quickly enough.
	
	Once kp and kd are chosen, add a disturbance force to the motor output. This can be a spring, or
	any external circumstance which causes an error in the output. Then slowly increase ki until the error is
	eliminated quickly enough. ki should be kept as small as possible as it can cause instabilities and degrade 
	the dynamic response of the system.
	
Other Notes
	The direction was arbitrarily chosen in the code. If the system appears unstable initially, change the
	initialization of "dir" within the direction function. This swaps which direction is positive and negative.
	
************************/

#include "Control.h"	//Custom work in progress class

Control motor;	//create a Control object


/************************
Global Variables
*************************/
bool encAState, encBState;	//Encoder channel states
const int encA = 2;	//Pins for encoders
const int encB = 3;	//Pins for encoders

long count;	//Global variables to track position of motors (encoders)

float ang = 0;	//This will hold the commanded angle value

void setup() 
{
	Serial.begin(9600);
  
	//Global Variable Setup
	count = 0;	//Initialize encoder count
	pinMode(encA, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	pinMode(encB, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	encAState = digitalRead(encA);	//Initialize encoder state
	encBState = digitalRead(encB);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(encA), encA_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(encB), encB_changed, CHANGE);	//Setup interrupt for background proccessing
	
	

}

void loop()
{

	const int motorPwmPin = 5;	//Define pins for motor PWM
	const int motorDirPin = 4;	//Define pins for motor direction
	
	unsigned long lastTime = millis();	//Initialize sampeling timing
	unsigned long lastPrintTime = millis();
	
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
	
	float adcBits = 10.0;	//10 is standard for most arduinos
	float pwmBits = 8.0;	//8 is standard for most arduinos
	float encoderCPR = 4741.44;	//This is the effective CPR for the pololu motor listed above
	
	motor.set_gains(kp, ki, kd);
	motor.set_pins(motorPwmPin, motorDirPin);
	motor.set_antiwindup(intThreshHigh, intThreshLow);
	motor.set_time_filter(Ts/1000, alpha);
	motor.set_precision(pwmBits, adcBits);
	
	while(1)
	{
		
		if (Serial.available())	//Serial Data is in the buffer...
		{
			Serial.println("Recieved New Command");
			serial_parser();
		}		
		
		if( millis() - lastTime >= Ts)
		{
			lastTime = millis();	//reset timer
			deg = (float(count)*360.0)/encoderCPR;	//read from encoder
			deg_c = ang;	//set command
			control = motor.pid(deg, deg_c, flag);	//run PID algorithm
			digitalWrite(motorDirPin, direction(control));	//Output direction to motor driver
			analogWrite(motorPwmPin, (pow(2.0,pwmBits)-1)*abs(control));	//Output voltage (via PWM) to motor
			if(flag)	//Reset flag
			{
				flag = !flag;
			}
		}
		
		if((millis() - lastPrintTime >= 1000))	//Print the status to the screen once a second
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