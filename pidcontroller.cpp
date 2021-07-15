#include "pidcontroller.h"
//#include "clockwork.h"

pidController::pidController(float input, float output, float setpoint, float kp, float ki, float kd, float T)
{
    setInput( input );
    setOutput( output );
    setSetpoint( setpoint );
    setKp( kp );
    setKi( ki );
    setKd( kd ); 
       //setErrorSum(0);
       //setLastError(0);
    setTime(T);
}

void pidController::setInput ( float input )
{
    Input = input;
}

float pidController::getInput ()
{
 	return Input;
}

void pidController::setOutput ( float output )
{
	Output = output;
}

float pidController::getOutput ()
{
 	return Output;
}

void pidController::setSetpoint ( float setpoint )
{
    Setpoint = setpoint;
}

float pidController::getSetpoint ()
{
 	return Setpoint;
}

void pidController::setKp ( float kp )
{
    Kp = kp;
}

float pidController::getKp ()
{
    return Kp;
}

void pidController::setKi ( float ki )
{
    Ki = ki;
}

float pidController::getKi ()
{
    return Ki;
}

void pidController::setKd ( float kd )
{
    Kd = kd;
}

float pidController::getKd ()
{
    return Kd;
}

void pidController::setErrorSum(float errs)
{
    errorSum = errs;
}

float pidController::getErrorSum()
{
    return errorSum;
}

void pidController::setLastError(float lasterror)
{
    lastError = lasterror;
}

float pidController::getLastError()
{
    return lastError; 
}

void pidController::setTime( float T)
{
    time = T;
}

float pidController::getTime()
{
    return time;
}

void pidController::setTunningParameters( float kp, float ki, float kd)
{
    setKp(kp);
    setKi(ki);
	setKd(kd);
}

float pidController::pidCompute()
{
  float error;
  float lastError; 
  float output;
  float Iterm;
  error = getSetpoint() - getInput();
  errorSum += error;
  lastError = error - getLastError();
  Iterm = (getKi())*getErrorSum()*getTime();
  if (Iterm > 255) Iterm = 255;
  if (Iterm < -255) Iterm = -255;
  output = (getKp())*error + (getKd())*lastError*(1.00)/(getTime()) + Iterm; 
  setOutput(output);
  setLastError(error); 
  return output;
}

void pidController::setRightMotorSpeed (int rightmotorspeed)
{
	RightMotorSpeed = rightmotorspeed;
}

int pidController::getRightMotorSpeed()
{
    return RightMotorSpeed;
}

void pidController::setLeftMotorSpeed (int leftmotorspeed)
{
	LeftMotorSpeed = leftmotorspeed;
}

int pidController::getLeftMotorSpeed()
{
    return LeftMotorSpeed;
}

void pidController::pidControl( int RightBaseSpeed, int LeftBaseSpeed, int Min, int Max)
{
      
    int rsp;
    int lsp;
       
    rsp = RightBaseSpeed - int (pidCompute());
    if( rsp > Max) rsp = Max;
    if( rsp < Min) rsp = Min;
    setRightMotorSpeed(rsp);
       
    lsp = LeftBaseSpeed  + int (pidCompute());
    if( lsp > Max) lsp = Max;
    if( lsp < Min) lsp = Min;
    setLeftMotorSpeed(lsp);
}
