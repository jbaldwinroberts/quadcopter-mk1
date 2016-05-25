#include "mbed.h"
#include "rtos.h"
#include "FreeIMU.h"
#include "PID.h"
#include "ConfigFile.h"
#include "PPM.h"
#include <sstream>
#include <TinyGPS.h>
#include "sonar.h"
#include "MODSERIAL.h"
#include "filter.h"

#ifndef HARDWARE_H
#define HARDWARE_H

//Global constants
#define             IMU_YAW_ANGLE_MAX 180
#define             IMU_YAW_ANGLE_MIN -180
#define             IMU_ROLL_ANGLE_MAX 90
#define             IMU_ROLL_ANGLE_MIN -90
#define             IMU_PITCH_ANGLE_MAX 90
#define             IMU_PITCH_ANGLE_MIN -90
#define             IMU_YAW_RATE_MAX 360
#define             IMU_YAW_RATE_MIN -360
#define             IMU_ROLL_RATE_MAX 360
#define             IMU_ROLL_RATE_MIN -360
#define             IMU_PITCH_RATE_MAX 360
#define             IMU_PITCH_RATE_MIN -360

#define             RC_CHANNELS 8
#define             RC_THROTTLE_CHANNEL 3
#define             RC_IN_MAX 1900
#define             RC_IN_MIN 1000
#define             RC_OUT_MAX 1
#define             RC_OUT_MIN 0
#define             RC_YAW_RATE_MAX 180
#define             RC_YAW_RATE_MIN -180
#define             RC_ROLL_RATE_MAX 90
#define             RC_ROLL_RATE_MIN -90
#define             RC_PITCH_RATE_MAX 90
#define             RC_PITCH_RATE_MIN -90
#define             RC_ROLL_ANGLE_MAX 20
#define             RC_ROLL_ANGLE_MIN -20
#define             RC_PITCH_ANGLE_MAX 20
#define             RC_PITCH_ANGLE_MIN -20
#define             RC_THRUST_MAX 1
#define             RC_THRUST_MIN 0

#define             MOTORS_OFF 0
#define             MOTORS_ARMED 1000
#define             MOTORS_MIN 1060
#define             MOTORS_MAX 1860

#define             RATE_PID_CONTROLLER_OUTPUT_MAX 100
#define             RATE_PID_CONTROLLER_OUTPUT_MIN -100

#define             FLIGHT_CONTROLLER_FREQUENCY 500

//Global Functions
//void ZeroPitchRoll();
void Arm();
void Disarm();
void WriteSettingsToConfig();
void ConvertToCharArray(float number);
void ConvertToCharArray(int number);
float Map(float input, float inputMin, float inputMax, float outputMin, float outputMax);

//Global Variables
float               _yawRatePIDControllerP, _yawRatePIDControllerI, _yawRatePIDControllerD, _pitchRatePIDControllerP, _pitchRatePIDControllerI, _pitchRatePIDControllerD, _rollRatePIDControllerP, _rollRatePIDControllerI, _rollRatePIDControllerD;
float               _yawStabPIDControllerP, _yawStabPIDControllerI, _yawStabPIDControllerD, _pitchStabPIDControllerP, _pitchStabPIDControllerI, _pitchStabPIDControllerD, _rollStabPIDControllerP, _rollStabPIDControllerI, _rollStabPIDControllerD;
float               _zeroValues[3] = {0,0,0}; //Yaw, pitch, roll
float               _oldZeroValues[3] = {0,0,0}; //Yaw, pitch, roll
float               _rcCommands[8] = {0,0,0,0,0,0,0,0};
float               _rcMappedCommands[4] = {0,0,0,0}; //Yaw, pitch, roll, thrust
double              _gpsValues[5] = {0,0,0,0,0}; //Latitude, longitude, altitude, course, speed
bool                _gpsConnected = false;
bool                _armed = false;
bool                _rate = false;
bool                _stab = true;
bool                _initialised = false;
float               _motorPower [4] = {0,0,0,0};
float               _gyroRate[3] = {0,0,0}; // Yaw, Pitch, Roll
float               _ypr[3] = {0,0,0}; // Yaw, pitch, roll
float               _ratePIDControllerOutputs[3] = {0,0,0}; //Yaw, pitch, roll
float               _stabPIDControllerOutputs[3] = {0,0,0}; //Yaw, pitch, roll
bool                _levelOffset = false;
int                 _commsMode = 0;
int                 _batt = 0;
float               _yawTarget = 0;
float               _maxBotixPingAltitude = 0;
float               _barometerAltitude = 0;
    
//PID controllers
PID                 *_yawRatePIDController;
PID                 *_pitchRatePIDController;
PID                 *_rollRatePIDController;
PID                 *_yawStabPIDController;
PID                 *_pitchStabPIDController;
PID                 *_rollStabPIDController;

//Threads
Thread              *_statusThread;
Thread              *_serialPortMonitorThread;
Thread              *_flightControllerThread;
Thread              *_rcCommandMonitorThread;
Thread              *_altitudeMonitorThread;

//Config file
LocalFileSystem     local("local");
ConfigFile          _configFile;
char*               _str = new char[1024];

//RC filters
medianFilter        *_yawMedianFilter;
medianFilter        *_pitchMedianFilter;
medianFilter        *_rollMedianFilter;
medianFilter        *_thrustMedianFilter;

//HARDWARE////////////////////////////////////////////////////////////////////////////////////
// M1  M2
//  \  /
//   \/
//   /\
//  /  \
// M3  M4
 
//Motors
PwmOut              _motor1(p21);
PwmOut              _motor2(p22);
PwmOut              _motor3(p23);
PwmOut              _motor4(p24);

//USB serial
MODSERIAL           _wiredSerial(USBTX, USBRX);

//Wireless Serial
MODSERIAL           _wirelessSerial(p9, p10);

//GPS Serial
MODSERIAL           _gps(p13, p14);
TinyGPS             _tinyGPS;

//PPM in
PPM                 *_ppm;
InterruptIn         *_interruptPin = new InterruptIn(p8);

//Onboard LED's
DigitalOut          _led1(LED1);
DigitalOut          _led2(LED2);
DigitalOut          _led3(LED3);
DigitalOut          _led4(LED4);

//IMU
FreeIMU             _freeIMU;

//Buzzer
DigitalOut          _buzzer(p20);

//MaxBotix Ping sensor
Timer               _maxBotixTimer;
Sonar               _maxBotixSensor(p15, _maxBotixTimer);

//Unused analog pins
DigitalOut         _spare1(p16);
DigitalOut         _spare2(p17);
DigitalOut         _spare3(p18);
DigitalOut         _spare4(p19);

//Functions///////////////////////////////////////////////////////////////////////////////////////////////
//Zero gyro and arm
void Arm()
{
    //Don't arm unless throttle is equal to 0 and the transmitter is connected
    if(_rcMappedCommands[3] < (RC_THRUST_MIN + 0.2) && _rcMappedCommands[3] != -1 && _armed == false)
    {
        //Zero gyro
        _freeIMU.zeroGyro();
        
        //Set armed to true
        _armed = true; 
    }  
}

//Disarm
void Disarm()
{   
    if(_armed == true)
    {
        //Set armed to false
        _armed = false;  
        
        //Disable modes
        _levelOffset = false; 
        
        //Save settings
        WriteSettingsToConfig();
    }
}

//Zero pitch and roll
/*void ZeroPitchRoll()
{  
    printf("Zeroing pitch and roll\r\n");
    
    //Zero pitch and roll
    float totalPitch = 0;
    float totalRoll = 0;
    float ypr[3] = {0,0,0}; // Yaw, pitch, roll
    for(int i = 0; i < 500; i++)
    {
        _freeIMU.getYawPitchRoll(ypr);
        totalPitch += ypr[1];
        totalRoll += ypr[2];
    }
    
    _zeroValues[1] = totalPitch/500;
    _zeroValues[2] = totalRoll/500;
    printf("Pitch %f\r\n", _zeroValues[1]);
    printf("Roll %f\r\n", _zeroValues[2]);
}  */  

//Saves settings to config file
void WriteSettingsToConfig()
{
    _wiredSerial.printf("Writing settings to config file\n\r");
    
    if(_armed == false) //Not flying
    {
        _freeIMU.sample(false);
        
        //Write values
        ConvertToCharArray(_yawRatePIDControllerP);
        if (!_configFile.setValue("_yawRatePIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawRatePIDControllerP\n\r");
        }
        
        ConvertToCharArray(_yawRatePIDControllerI);
        if (!_configFile.setValue("_yawRatePIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawRatePIDControllerI\n\r");
        }
        
        ConvertToCharArray(_yawRatePIDControllerD);
        if (!_configFile.setValue("_yawRatePIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawRatePIDControllerD\n\r");
        }
        
        ConvertToCharArray(_pitchRatePIDControllerP);
        if (!_configFile.setValue("_pitchRatePIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchRatePIDControllerP\n\r");
        }
        
        ConvertToCharArray(_pitchRatePIDControllerI);
        if (!_configFile.setValue("_pitchRatePIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchRatePIDControllerI\n\r");
        }
        
        ConvertToCharArray(_pitchRatePIDControllerD);
        if (!_configFile.setValue("_pitchRatePIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchRatePIDControllerD\n\r");
        }
        
        ConvertToCharArray(_rollRatePIDControllerP);
        if (!_configFile.setValue("_rollRatePIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollRatePIDControllerP\n\r");
        }
        
        ConvertToCharArray(_rollRatePIDControllerI);
        if (!_configFile.setValue("_rollRatePIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollRatePIDControllerI\n\r");
        }
        
        ConvertToCharArray(_rollRatePIDControllerD);
        if (!_configFile.setValue("_rollRatePIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollRatePIDControllerD\n\r");
        }
    
        ConvertToCharArray(_yawStabPIDControllerP);
        if (!_configFile.setValue("_yawStabPIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawStabPIDControllerP\n\r");
        }
        
        ConvertToCharArray(_yawStabPIDControllerI);
        if (!_configFile.setValue("_yawStabPIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawStabPIDControllerI\n\r");
        }
        
        ConvertToCharArray(_yawStabPIDControllerD);
        if (!_configFile.setValue("_yawStabPIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _yawStabPIDControllerD\n\r");
        }
        
        ConvertToCharArray(_pitchStabPIDControllerP);
        if (!_configFile.setValue("_pitchStabPIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchStabPIDControllerP\n\r");
        }
        
        ConvertToCharArray(_pitchStabPIDControllerI);
        if (!_configFile.setValue("_pitchStabPIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchStabPIDControllerI\n\r");
        }
        
        ConvertToCharArray(_pitchStabPIDControllerD);
        if (!_configFile.setValue("_pitchStabPIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _pitchStabPIDControllerD\n\r");
        }
        
        ConvertToCharArray(_rollStabPIDControllerP);
        if (!_configFile.setValue("_rollStabPIDControllerP", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollStabPIDControllerP\n\r");
        }
        
        ConvertToCharArray(_rollStabPIDControllerI);
        if (!_configFile.setValue("_rollStabPIDControllerI", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollStabPIDControllerI\n\r");
        }
        
        ConvertToCharArray(_rollStabPIDControllerD);
        if (!_configFile.setValue("_rollStabPIDControllerD", _str))
        {
            _wiredSerial.printf("Failed to write value for _rollStabPIDControllerD\n\r");
        }
    
        ConvertToCharArray(_zeroValues[1]);
        if (!_configFile.setValue("_zeroPitch", _str))
        {
            _wiredSerial.printf("Failed to write value for zero pitch\n\r");
        }
        
        ConvertToCharArray(_zeroValues[2]);
        if (!_configFile.setValue("_zeroRoll", _str))
        {
            _wiredSerial.printf("Failed to write value for zero roll\n\r");
        }
        
        if (!_configFile.write("/local/config.cfg"))
        {
            _wiredSerial.printf("Failure to write settings to configuration file.\n\r");
        }
        else _wiredSerial.printf("Successfully wrote settings to configuration file.\n\r");
        
        _freeIMU.sample(true);
    }
    else
    {
        _wiredSerial.printf("Cannot write to config file whilst throttle is above 0\n\r");
    }
}

//Converts float to char array
void ConvertToCharArray(float number)
{
    sprintf(_str, "%1.8f", number );  
}

//Converts integer to char array
void ConvertToCharArray(int number)
{
    sprintf(_str, "%d", number );  
}

float Map(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}

#endif