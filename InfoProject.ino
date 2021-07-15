#include "clockwork.h"
#include "BluetoothController.h"
#include "pidcontroller.h"  
#include "finite_state_machine.h"
#include <Arduino.h>
#include "MeOrion.h"
#include <SoftwareSerial.h>

//  Constant Parameter Declaration
#define rightMaxSpeed 250
#define leftMaxSpeed  250
#define Time 0.5
#define EEPROM_ADDR 0x03

//  Port Declaration
MeDCMotor               MotorL(PORT_1);  
MeDCMotor               MotorR(PORT_2);
MeLineFollower          left_lineFinder(PORT_3);
MeLineFollower          right_lineFinder(PORT_4);
MeBluetooth             bluetooth(PORT_5);
MeUltrasonicSensor      ultraSensor1(PORT_6);
MeUltrasonicSensor      ultraSensor2(PORT_7);
MeUltrasonicSensor      ultraSensor3(PORT_8);

//  Variable Declaration
int16_t rightBaseSpeed = 80;
int16_t leftBaseSpeed  = 80;
float   input          = 0;
float   output         = 0;
float   setpoint       = 0;
double u1;
double u2;
double u3;
unsigned char command = 0;

// setting parameters

struct settings_t {
    float Kp;
    float Ki;
    float Kd;
};

union settings_u_t {
    settings_t settings;
    byte settings_byte[sizeof(settings)];
};

settings_u_t settings_u;
settings_t* settings = &(settings_u.settings);

// State Function Declaration
void start();
void manualControl();
void automaticControl();
void decision();
void setEEPROM();

//  State Declaration
State Init(0,start);
State ManualControl(1,manualControl);
State AutomaticControl(2,automaticControl);
State Decision(3,decision);
State Setting(4,setEEPROM);

// Class Declaration
pidController pid(input, output, setpoint, 0, 0, 0,0.0005);
Clockwork clockwork(Time);
BluetoothController bluetest(leftBaseSpeed, rightBaseSpeed);

// StateMachine Declaration
FiniteStateMachine robot = FiniteStateMachine(&Init);

void setup()
{
    Serial.begin(9600);
    bluetooth.begin(115200);
    default_settings();
    pid.setTunningParameters(settings->Kp,settings->Ki,settings->Kd);
    delay(10);
}

// the main loop
void loop()
{
    robot.update();
}

// Functions for each state
void start()
{
    delay(1);
    Serial.println("START");
    bluetoothSensorCheck();
    command = bluetest.getCommand();
    switch(command)
    {
        case('s'):
            default_settings();
            save_to_EEPROM();
            pid.setTunningParameters(settings->Kp,settings->Ki,settings->Kd);
            robot.transitionTo(&ManualControl);
            break;
        case('r'):
            read_from_EEPROM();
            pid.setTunningParameters(settings->Kp,settings->Ki,settings->Kd);
            robot.transitionTo(&ManualControl);
            break;
    }
    command = '0';
    bluetest.setCommand(command);
}

void manualControl()
{
    Serial.println("Manual Control");
    //command = bluetoothSensorCheck();
    bluetoothSensorCheck();
    bluetest.setUltraSensor(ultraSensor1.distanceCm(), ultraSensor2.distanceCm(), ultraSensor3.distanceCm());
    bluetest.ultraCheck(30,30,30);
    command = bluetest.getCommand();
    //Serial.println(command);
    switch(command)
    {
        case('1'): 
            forward();
            break;
        case('2'):
            backward();
            break;
        case('3'):
            turnleft();
            break;
        case('4'):
            turnright();
            break;
        case('5'):
            bluetest.increase();
            delay(100);
            break;
        case('6'):
            bluetest.decrease();
            delay(100);
            break;
        case('7'):
            robot.transitionTo(&AutomaticControl);
            break;
        case('s'):
            robot.transitionTo(&Setting);
            break;
        case('0'):
            stop();
            break;
    }
    command = '0';
    bluetest.setCommand(command);
}

void automaticControl()
{ 
    Serial.println("Auto Mode");
    read_from_EEPROM();
    pid.setTunningParameters(settings->Kp,settings->Ki,settings->Kd);
    bluetest.setUltraSensor(ultraSensor1.distanceCm(), ultraSensor2.distanceCm(), ultraSensor3.distanceCm());
    bluetest.ultraCheckAutoMode(30,30,30);
    bluetoothSensorCheck();
    command = bluetest.getCommand();
    //Serial.println(command);
    switch(command)
    {
        case('8'):
            robot.transitionTo(&ManualControl);
            break;
        case('l'):
            stop();
            //robot.transitionTo(&ManualControl);
            break;
        default:
            clockwork.start();
            for(int i = 0; i< 5; i++)
            {
                input += -(readPosition());
                delayMicroseconds(50);
            }
            input = input/5;
            //Serial.println(input);
            if(abs(input)>50)
            {
                MotorL.stop();
                MotorR.stop();
                robot.transitionTo(&Decision);
            }
            else
            {
                pid.setInput(input);
                pid.pidCompute();
                /*
                pid.pidControl(rightBaseSpeed, leftBaseSpeed, -255, 255);
                MotorL.run(pid.getLeftMotorSpeed());
                MotorR.run(pid.getRightMotorSpeed());
                */
                pid.pidControl(bluetest.rspSpeed(),bluetest.lspSpeed(), -255, 255); // Change
                MotorL.run(pid.getLeftMotorSpeed());                                // Change
                MotorR.run(pid.getRightMotorSpeed());                               // Change
            }
  
            input = 0;
            clockwork.stop(); 
            break;
    }
        //command = '0';
}

void decision()
{
    Serial.println("Decision");
    bluetoothSensorCheck();
    command = bluetest.getCommand();
    switch(command)
    {
        case('1'): 
            forward();
            robot.transitionTo(&AutomaticControl);
            break;
        case('2'):
            backward();
            robot.transitionTo(&AutomaticControl);
            break;
        case('3'):
            turnleft();
            robot.transitionTo(&AutomaticControl);
            break;
        case('4'):
            turnright();
            robot.transitionTo(&AutomaticControl);
            break;
        default:
            stop();
            break;
    }
    command = '0';
    bluetest.setCommand(command);
    input = 0;
}

void forward()
{
    MotorL.run(bluetest.lspSpeed());
    MotorR.run(bluetest.rspSpeed());
    delay(100);
}

void backward()
{
    MotorL.run(-bluetest.lspSpeed());
    MotorR.run(-bluetest.rspSpeed());
    delay(100); 
}
void turnright()
{
    MotorL.run(bluetest.lspSpeed());
    MotorR.run(bluetest.rspSpeed()/10);
    delay(100);  
}

void turnleft()
{
    MotorL.run(bluetest.lspSpeed()/10);
    MotorR.run(bluetest.rspSpeed());
    delay(100);
}

void stop()
{
    MotorL.stop();
    MotorR.stop();
}

void bluetoothSensorCheck()
{
    int readdata = 0;
    if (bluetooth.available())
    {
        while((readdata = bluetooth.read()) != (int)-1)
        {
        bluetest.setCommand(readdata);
        delay(1);
        }
    //   Serial.write(command);
    }
}

// EEPROM
void default_settings() 
{
    settings->Kp = 30;
    settings->Kd = 10;
    settings->Ki = 1;
}

void save_to_EEPROM() 
{
    for(int i = 0; i < sizeof(settings_t); i++) 
    {
        EEPROM.write(EEPROM_ADDR + i, settings_u.settings_byte[i]);
    }
    Serial.println("Settings saved");
}

void read_from_EEPROM() 
{
    for(int i = 0; i < sizeof(settings_t); i++) {
    settings_u.settings_byte[i] = EEPROM.read(EEPROM_ADDR + i);
    }
    Serial.println("Settings loaded");
}

void setEEPROM()
{
    static int parameter = 0;
    //Serial.println("EEPROM");
    bluetoothSensorCheck();
    command = bluetest.getCommand();
    switch(command)
    {
        case('r'):
            save_to_EEPROM();
            robot.transitionTo(&ManualControl);
            break;
    
        case('1'):
            if(parameter == 0)
            {
                settings->Kp = settings->Kp + 1;
                delay(10);
            }
        
            if(parameter == 1)
            {
                settings->Ki = settings->Ki + 1;
                delay(10);
            }
            if(parameter == 2)
            {
                settings->Kd = settings->Kd + 1;
                delay(10);
            }
            break;
      
        case('2'):
            if(parameter == 0)
            {
                settings->Kp = settings->Kp - 1;
                delay(10);
            }
            if(parameter == 1)
            {
                settings->Ki = settings->Ki - 1;
                delay(10);
            }
            if(parameter == 2)
            {
                settings->Kd = settings->Kd - 1;
                delay(10);
            }
            break;

        case('4'):
            parameter++;
            if(parameter > 2) parameter = 2;
            break;      

        case('3'):
            parameter--;
            if(parameter < 0) parameter = 0;
            break;          
    }
    command = '0';
    bluetest.setCommand(command);
    Serial.print(settings->Kp);
    Serial.print("\t");
    Serial.print(settings->Ki);
    Serial.print("\t");
    Serial.print(settings->Kd);
    Serial.print("\t");
    Serial.print("\n");
}
// Read position of robot

float readPosition()
{
    static float old_pos = 0;
    float current_pos;
    int left_sensorState = left_lineFinder.readSensors();
    int right_sensorState = right_lineFinder.readSensors();
    if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT && old_pos <0) current_pos = -3;
    if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_OUT_S2_OUT) current_pos =  -2;
    if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_OUT_S2_OUT) current_pos = -1;
    if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_IN_S2_OUT) current_pos = 0;
    if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_IN_S2_IN) current_pos = 1;
    if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_IN) current_pos = 2;
    if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT && old_pos >0) current_pos = 3;

    // impossible position => return the old position
    if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_IN_S2_OUT) current_pos =  old_pos;
    if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_OUT_S2_OUT) current_pos =  old_pos;
    if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
    if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_IN_S2_IN) current_pos =  0.5;  
    if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
    if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_IN_S2_OUT) current_pos =  old_pos;
    if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_IN_S2_IN) current_pos =  old_pos;
    if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
    if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_IN_S2_OUT) current_pos =  -0.5;  
    if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_IN_S2_IN) current_pos =  1000;   
    old_pos = current_pos;
    return current_pos;
}
