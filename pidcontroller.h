#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

class pidController
{
 	public:
    	// Constructor initialization
    	pidController(float input, float output, float setpoint, float Kp, float Ki, float Kd, float T);
    
    	// Set function will set the value for the input, get function will return the value 
    	void setInput( float input);
    	float getInput();
    
    	// Set function will set the value for the output, get function will return the value 
    	void setOutput( float output);
    	float getOutput();
    
    	// Set function will set the value for the setpoint, get function will return the value 
    	void setSetpoint(float setpoint);
    	float getSetpoint();
    
    	// Set function will set the value for Kp, get function will return the value 
    	void setKp( float Kp );
    	float  getKp();
    
    	// Set function will set the value for Ki, get function will return the value 
    	void setKi( float Ki );
    	float  getKi();
    
    	// Set function will set the value for Kd, get function will return the value 
    	void setKd( float Kd );
    	float  getKd();

    	// Set function will set the value for Errorsum which is a quantity to calculate integral part in PID controller
    	//, get function will return the value 
    	void setErrorSum( float errs);
    	float getErrorSum();
    
    	// Set function will set the value for Lasterror which is a quantity to calculate differential part in PID controller
    	//, get function will return the value
    	void setLastError(float lasterror);
    	float getLastError();
    
    	// Set function will set the value for time which is a quantity to do calculation in PID controller and also the period that our 
    	// controller will be called 
    	//, get function will return the value
    	void setTime( float T);
    	float getTime();
    
    	// This function is used to set three parameters if one wants to change it
    	void setTunningParameters( float Kp, float Ki, float Kd);

    	// pidCompute function will calculate the pid value which could be put into our motor's speed control
    	float pidCompute();
    
    	// set speed for the right motor
    	void setRightMotorSpeed( int rightmotorspeed);
    	int  getRightMotorSpeed();
  
    	// set speed for the left motor
    	void setLeftMotorSpeed( int leftmotorspeed);
    	int  getLeftMotorSpeed();
    
    	// pidControl set up speed for the right and the left motor which should be limited. 
    	// If the value is larger than 255, it will be set to 255. If the value is smaller than -255, it will be set to -255
    	void pidControl( int RightBaseSpeed, int LeftBaseSpeed, int Min, int Max);

 	private:
 		float Input;
 		float Output;
 		float Setpoint;
 		float Kp;
 		float Ki;
 		float Kd; 
 		int RightMotorSpeed;
 		int LeftMotorSpeed;
 		float time;
 		float  errorSum;
 		float lastError;
};

#endif
