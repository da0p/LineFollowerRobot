#include "BluetoothController.h"

BluetoothController::BluetoothController(int16_t lsp, int16_t rsp)
{
    leftmotorspeed  = lsp;
    rightmotorspeed = rsp;   

}

void BluetoothController::setUltraSensor(double u1, double u2, double u3)
{
    ultra1 = u1;
    ultra2 = u2;
    ultra3 = u3;
}

void BluetoothController::setCommand(unsigned char cmd)
{
	command = cmd;
}

unsigned char BluetoothController::getCommand()
{
	return command;
}

double BluetoothController::getUltra1()
{
	return ultra1;
}

double BluetoothController::getUltra2()
{
	return ultra2;
}

double BluetoothController::getUltra3()
{
	return ultra3;
}


void BluetoothController::ultraCheck(double dist1, double dist2, double dist3)
{
   double temp_u1 = getUltra1();
   double temp_u2 = getUltra2();
   double temp_u3 = getUltra3();
   unsigned char   cmd     = getCommand();
   //unsigned char temp_cmd;
  if(temp_u1 < dist1 && temp_u1 > 1 && cmd =='1') cmd = '0';
  if(temp_u2 < dist2 && temp_u2 > 1 && cmd =='1') cmd = '0';
  if(temp_u3 < dist3 && temp_u3 > 1 && cmd =='1') cmd = '0';
  setCommand(cmd);
}

void BluetoothController::ultraCheckAutoMode(double dist1, double dist2, double dist3)
{
   double temp_u1 = getUltra1();
   double temp_u2 = getUltra2();
   double temp_u3 = getUltra3();
   unsigned char   cmd;
   //unsigned char temp_cmd;
  if(temp_u1 < dist1 && temp_u1 > 1) cmd = 'l';
  else if(temp_u2 < dist2 && temp_u2 > 1 ) cmd = 'l';
  else if(temp_u3 < dist3 && temp_u3 > 1 ) cmd = 'l';
  else cmd = '0';
  setCommand(cmd);
}

void BluetoothController::increase()
{
  leftmotorspeed  = leftmotorspeed  + 10;
  rightmotorspeed = rightmotorspeed + 10;

  if ((leftmotorspeed > 255) || (rightmotorspeed > 255))
  {
     leftmotorspeed  = 255;
     rightmotorspeed = 255;
  }
}

void BluetoothController::decrease()
{
  leftmotorspeed  = leftmotorspeed  - 10;
  rightmotorspeed = rightmotorspeed - 10;

  if ((leftmotorspeed < -255) || (rightmotorspeed < -255))
  {
     leftmotorspeed  = -255;
     rightmotorspeed = -255;
  }
}
