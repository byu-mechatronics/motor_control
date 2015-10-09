#define toggle   A0

#define dcMotor  3

#define dirPin A5
#define stepPin A4
#define stepNumber  50
#define stepSpeed  .25
#define delay1 250

#define led 13

int motorSpeed;  //Represents dc speed value bewteen 0 and 255
int toggleStatus; 
int getOut = 0;  //Used to control Stepper using EiBotBoard (EBB)

const int stepperTime = 5000;  //Time in milliseconds that the stepper motor will move for

unsigned long nextTime;  //Used for this particular stepper motor controller as a the duration of the move

void setup() 
{
  
 Serial.begin(9600);
 pinMode(toggle, INPUT_PULLUP);  //Set the pin as input, which is a high impedence state with a pull-up resistor. This means current is limited and signal is default pulled HIGH
 pinMode(dcMotor, OUTPUT);  //Set the motor control pin as an output
 
}

void loop() 
{
  //Serial.println("The Loop has Started");  //testing
  
  toggleStatus = digitalRead(toggle);  //Determine to control the DC motor or the Stepper motor
  
  //Serial.println(toggleStatus);  //testing
  
  switch(toggleStatus)
{
//-------------------------------------------------------------------------------------------------------------------
    case 0:  //DC Motor, switch is grounding the toggle pin
        
        Serial.println("Case 0");  //testing
        
        Serial.println("DC Motor Speed Up");  //testing  
        
        digitalWrite(led, !digitalRead(led));  //Toggle led
        
        for( motorSpeed = 0; motorSpeed <= 255; motorSpeed++)  //Speed up for around 25.5 seconds
        {
          analogWrite(dcMotor, motorSpeed);
          
          delay(25);
          
          toggleStatus = digitalRead(toggle);
          
          if(toggleStatus != 0)
            break;
        }
        
        if(toggleStatus != 1)  //If the toggle hasn't been switched, then continue to slowly occilate
        {
          
          Serial.println("DC Motor Slow Down");  //testing
          
          digitalWrite(led, !digitalRead(led));  //Toggle led
          
          for( motorSpeed = 255; motorSpeed >= 0; motorSpeed--)  //Slow down for around 25.5 seconds
          {
            analogWrite(dcMotor, motorSpeed);
            
            delay(25);
            
            toggleStatus = digitalRead(toggle);
            
            if(toggleStatus != 0)
              break;           
          }
        }
        else  //If the toggle has been switched, turn off the dc motor
          analogWrite(dcMotor,0);
        
    break;  //Break out of case 0, DC motor demo
//--------------------------------------------------------------------------------------------------------------------------------
    case 1:  //Stepper Motor, controlled via EggBotBoard rev2, toggle pin is free (not pulled to ground)
        
        Serial.println("Case 1");  //testing
        
        rotate(stepNumber*8,stepSpeed);
        
        delay(delay1);
        
        toggleStatus = digitalRead(toggle);  //Determine to control the DC motor or the Stepper motor
         
        if(toggleStatus == 1)
         {
           rotate(-stepNumber*8,stepSpeed);
           delay(delay1);   
         }
        
    /*  //Control stepper motor with EBB stepper driver
        Serial.write("EM,1,1");  //Turn on stepper motor
        Serial.write("SM,stepperTime, 200, 200");  //Command movement lasting "stepperTime" and equal to 200 steps
        
        nextTime = millis() + stepperTime;  //Determine the time at which the motor fully moved
        
        while((millis() <= nextTime) && (toggleStatus == 1))  //While the current time is not yet the ending time && the toggle is still switched to stepper
        {
          toggleStatus = digitalRead(toggle);  //Check switch status for change to DC motor demo
          if(toggleStatus == 0)  //If DC motor selected... 
          {
            Serial.write("EM,0,0");  //Turn of stepper motor
          }
        }
        
        Serial.write("SM,stepperTime, -200, -200");  //Command movement lasting "stepperTime" and equal to 200 steps
        
        nextTime = millis() + stepperTime;  //Determine the time at which the motor fully moved
        
        while((millis() <= nextTime && getOut != 1) && (toggleStatus == 1))  //While the current time is not yet the ending time && the toggle is still switched to stepper
        {
          toggleStatus = digitalRead(toggle);
          if(toggleStatus == 0)
          {
            Serial.write("EM,0,0");  //Turn on stepper motor
          }
        }
        
        Serial.write("EM,0,0");  //Turn on stepper motor
     */   
    break;  //Break out of case 1, stepper motor demo  
  
  }//End of switch statement



}//End of Void Loop


void rotate(int steps, float speed)  //Control stepper motor with EasyDriver board from Sparkfun
{ 
  //rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  
  int dir = (steps > 0)? HIGH:LOW;
  steps = abs(steps);

  digitalWrite(dirPin,dir); 
  
  digitalWrite(led, !digitalRead(led));  //Toggle led

  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++)
  { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(usDelay); 

    digitalWrite(stepPin, LOW); 
    delayMicroseconds(usDelay); 
  } 
}
