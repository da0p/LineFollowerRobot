#ifndef _BLUETOOTHCONTROLLER_H_
#define _BLUETOOTHCONTROLLER_H_
#include <stdint.h>
class BluetoothController
{
   public:
       BluetoothController(int16_t lsp,int16_t rsp);

       void setUltraSensor(double u1, double u2, double u3);

       void setCommand(unsigned char data);
       unsigned char getCommand();
        
       void ultraCheck(double dist1, double dist2, double dist3);
       void ultraCheckAutoMode(double dist1, double dist2, double dist3);
       double getUltra1();
       double getUltra2();
       double getUltra3();
        
       void increase();
       void decrease();

       int16_t lspSpeed()
       {
        return leftmotorspeed;
       }

       int16_t rspSpeed()
       {
        return rightmotorspeed;
       }
       
   private:
       int16_t leftmotorspeed;
       int16_t rightmotorspeed;
       double ultra1;
       double ultra2;
       double ultra3;
       unsigned char command='0';
};
#endif
