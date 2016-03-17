
#ifndef Control_h
#define Control_h

#include <Arduino.h>

class Control
{
	
	public:
	
		Control(void);
		void begin();
		void set_gains(float kp, float ki, float kd);
		void set_pins(int pwmPin, int dirPin);
		void set_precision(int pwmBits, int adcBits);
		void set_antiwindup(float intThreshHigh, float intThreshLow);
		void set_time_filter(float Ts, float alpha);
		float pid(float state, float command, bool flag);
		
		
	private:
		
		//Functions
		float saturate(float input, float highLimit, float lowLimit);
		
		//Variables
		float _integrator, _differentiator, _error_d1, _intThreshHigh, _intThreshLow, _alpha, _Ts;
		float _kp, _ki, _kd;
		
		float _contThresh = 1.0;
		int _pwmPin, _dirPin, _pwmBits, _adcBits;
		
};

#endif