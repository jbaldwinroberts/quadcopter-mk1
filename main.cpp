//Includes
#include "mbed.h"
#include "rtos.h"
#include "FreeIMU.h"
#include "PID.h"
#include "ConfigFile.h"
#include "hardware.h"
#include "statusLights.h"
#include "serialPortMonitor.h"
#include "flightController.h"
#include "rcCommandMonitor.h"
#include "altitudeMonitor.h"

//Declarations
void LoadSettingsFromConfig();
void InitialisePID();
void InitialisePWM();
void Setup();

//Loads settings from the config file - could tidy this a little if I can be assed
void LoadSettingsFromConfig()
{
    _wiredSerial.printf("Starting to load settings from config file\n\r");
    
    //_wiredSerial.printf("Loading settings from config file\n\r");
    char value[BUFSIZ];
    
    //Read a configuration file from a mbed.
    if (!_configFile.read("/local/config.cfg"))
    {
        _wiredSerial.printf("Config file does not exist\n\r");
    }
    else
    {    
        //Get values
        if (_configFile.getValue("_yawRatePIDControllerP", &value[0], sizeof(value))) _yawRatePIDControllerP = atof(value);
        else
        {
            _wiredSerial.printf("Failed to get value for _yawRatePIDControllerP\n\r");
        }
        if (_configFile.getValue("_yawRatePIDControllerI", &value[0], sizeof(value))) _yawRatePIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _yawRatePIDControllerI\n\r");
        }
        if (_configFile.getValue("_yawRatePIDControllerD", &value[0], sizeof(value))) _yawRatePIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _yawRatePIDControllerD\n\r");
        }
        if (_configFile.getValue("_pitchRatePIDControllerP", &value[0], sizeof(value))) _pitchRatePIDControllerP = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchRatePIDControllerP\n\r");
        }
        if (_configFile.getValue("_pitchRatePIDControllerI", &value[0], sizeof(value))) _pitchRatePIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchRatePIDControllerI\n\r");
        }
        if (_configFile.getValue("_pitchRatePIDControllerD", &value[0], sizeof(value))) _pitchRatePIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchRatePIDControllerD\n\r");
        }
        if (_configFile.getValue("_rollRatePIDControllerP", &value[0], sizeof(value))) _rollRatePIDControllerP = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollRatePIDControllerP\n\r");
        }
        if (_configFile.getValue("_rollRatePIDControllerI", &value[0], sizeof(value))) _rollRatePIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollRatePIDControllerI\n\r");
        }
        if (_configFile.getValue("_rollRatePIDControllerD", &value[0], sizeof(value))) _rollRatePIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollRatePIDControllerD\n\r");
        }
        
        if (_configFile.getValue("_yawStabPIDControllerP", &value[0], sizeof(value))) _yawStabPIDControllerP = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _yawStabPIDControllerP\n\r");
        }
        if (_configFile.getValue("_yawStabPIDControllerI", &value[0], sizeof(value))) _yawStabPIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _yawStabPIDControllerI\n\r");
        }
        if (_configFile.getValue("_yawStabPIDControllerD", &value[0], sizeof(value))) _yawStabPIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _yawStabPIDControllerD\n\r");
        }
        if (_configFile.getValue("_pitchStabPIDControllerP", &value[0], sizeof(value))) _pitchStabPIDControllerP = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchStabPIDControllerP\n\r");
        }
        if (_configFile.getValue("_pitchStabPIDControllerI", &value[0], sizeof(value))) _pitchStabPIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchStabPIDControllerI\n\r");
        }
        if (_configFile.getValue("_pitchStabPIDControllerD", &value[0], sizeof(value))) _pitchStabPIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _pitchStabPIDControllerD\n\r");
        }
        if (_configFile.getValue("_rollStabPIDControllerP", &value[0], sizeof(value))) _rollStabPIDControllerP = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollStabPIDControllerP\n\r");
        }
        if (_configFile.getValue("_rollStabPIDControllerI", &value[0], sizeof(value))) _rollStabPIDControllerI = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollStabPIDControllerI\n\r");
        }
        if (_configFile.getValue("_rollStabPIDControllerD", &value[0], sizeof(value))) _rollStabPIDControllerD = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for _rollStabPIDControllerD\n\r");
        }
        if (_configFile.getValue("_zeroPitch", &value[0], sizeof(value))) _zeroValues[1] = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for zero pitch\n\r");
        }
        if (_configFile.getValue("_zeroRoll", &value[0], sizeof(value))) _zeroValues[2] = atof(value);
            else
        {
            _wiredSerial.printf("Failed to get value for zero roll\n\r");
        }
    }
    
    _wiredSerial.printf("Finished loading settings from config file\n\r");
}

//PID initialisation
void InitialisePID()
{
    float updateTime = 1.0 / FLIGHT_CONTROLLER_FREQUENCY;
    
    _yawRatePIDController = new PID(_yawRatePIDControllerP, _yawRatePIDControllerI, _yawRatePIDControllerD, updateTime);
    _yawRatePIDController->setInputLimits(IMU_YAW_RATE_MIN, IMU_YAW_RATE_MAX);
    _yawRatePIDController->setOutputLimits(RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX);
    _yawRatePIDController->setMode(AUTO_MODE);
    _yawRatePIDController->setSetPoint(0.0);
    _yawRatePIDController->setBias(0);
    
    _pitchRatePIDController = new PID(_pitchRatePIDControllerP, _pitchRatePIDControllerI, _pitchRatePIDControllerD, updateTime);
    _pitchRatePIDController->setInputLimits(IMU_PITCH_RATE_MIN, IMU_PITCH_RATE_MAX);
    _pitchRatePIDController->setOutputLimits(RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX);
    _pitchRatePIDController->setMode(AUTO_MODE);
    _pitchRatePIDController->setSetPoint(0.0);
    _pitchRatePIDController->setBias(0);
    
    _rollRatePIDController = new PID(_rollRatePIDControllerP, _rollRatePIDControllerI, _rollRatePIDControllerD, updateTime);
    _rollRatePIDController->setInputLimits(IMU_ROLL_RATE_MIN, IMU_ROLL_RATE_MAX);
    _rollRatePIDController->setOutputLimits(RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX);
    _rollRatePIDController->setMode(AUTO_MODE);
    _rollRatePIDController->setSetPoint(0.0);
    _rollRatePIDController->setBias(0);
    
    _yawStabPIDController = new PID(_yawStabPIDControllerP, _yawStabPIDControllerI, _yawStabPIDControllerD, updateTime);
    _yawStabPIDController->setInputLimits(IMU_YAW_ANGLE_MIN, IMU_YAW_ANGLE_MAX);
    _yawStabPIDController->setOutputLimits(IMU_YAW_RATE_MIN, IMU_YAW_RATE_MAX);
    _yawStabPIDController->setMode(AUTO_MODE);
    _yawStabPIDController->setSetPoint(0.0);
    _yawStabPIDController->setBias(0);
    
    _pitchStabPIDController = new PID(_pitchStabPIDControllerP, _pitchStabPIDControllerI, _pitchStabPIDControllerD, updateTime);
    _pitchStabPIDController->setInputLimits(IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
    _pitchStabPIDController->setOutputLimits(IMU_PITCH_RATE_MIN, IMU_PITCH_RATE_MAX);
    _pitchStabPIDController->setMode(AUTO_MODE);
    _pitchStabPIDController->setSetPoint(0.0);
    _pitchStabPIDController->setBias(0);
    
    _rollStabPIDController = new PID(_rollStabPIDControllerP, _rollStabPIDControllerI, _rollStabPIDControllerD, updateTime);
    _rollStabPIDController->setInputLimits(IMU_ROLL_ANGLE_MIN, IMU_ROLL_ANGLE_MAX);
    _rollStabPIDController->setOutputLimits(IMU_ROLL_RATE_MIN, IMU_ROLL_RATE_MAX);
    _rollStabPIDController->setMode(AUTO_MODE);
    _rollStabPIDController->setSetPoint(0.0);
    _rollStabPIDController->setBias(0);
}

//PWM Initialisation
void InitialisePWM()
{
    //500Hz
    float period = 1.0 / FLIGHT_CONTROLLER_FREQUENCY;
    _motor1.period(period);
    _motor2.period(period);
    _motor3.period(period);
    _motor4.period(period);
    
    //Disable
    _motor1 = MOTORS_OFF;
    _motor2 = MOTORS_OFF;
    _motor2 = MOTORS_OFF;
    _motor2 = MOTORS_OFF;
}

//Setup
void Setup()
{ 
    //Setup wired serial coms
    _wiredSerial.baud(115200);
    
    printf("\r\n");  
    printf("*********************************************************************************\r\n");
    printf("Starting Setup\r\n");
    printf("*********************************************************************************\r\n");
    
     //Disable buzzer
    _buzzer = 0;
    
    //Setup wireless serial coms
    _wirelessSerial.baud(57600);
    
    //Read config file
    LoadSettingsFromConfig();
    
    //Set initial RC Ccommands
    _rcMappedCommands[0] = 0;
    _rcMappedCommands[1] = 0;
    _rcMappedCommands[2] = 0;
    _rcMappedCommands[3] = 0;
    
    //Setup RC median filters
    _yawMedianFilter = new medianFilter(5);
    _pitchMedianFilter = new medianFilter(5);
    _rollMedianFilter = new medianFilter(5);
    _thrustMedianFilter = new medianFilter(5);

    //Initialise PPM
    _ppm = new PPM(_interruptPin, RC_OUT_MIN, RC_OUT_MAX, RC_IN_MIN, RC_IN_MAX, RC_CHANNELS, RC_THROTTLE_CHANNEL);

    //Initialise IMU
    _freeIMU.init(true);
    
    //Initialise MaxBotix ping sensor
    _maxBotixTimer.start();
    
    //Initialise GPS
    _gps.baud(115200);
    
    //Initialise PID
    InitialisePID();
    
    //Initialise PWM
    InitialisePWM();
    
    //Set initialised flag
    _initialised = true;
       
    // Start threads
    _flightControllerThread = new Thread (FlightControllerThread);
    _flightControllerThread->set_priority(osPriorityRealtime);
    _rcCommandMonitorThread = new Thread (RcCommandMonitorThread);
    _rcCommandMonitorThread->set_priority(osPriorityHigh);
    _altitudeMonitorThread = new Thread (AltitudeMonitorThread);
    _altitudeMonitorThread->set_priority(osPriorityHigh);
    _serialPortMonitorThread = new Thread (SerialPortMonitorThread);
    _serialPortMonitorThread->set_priority(osPriorityLow);
    _statusThread = new Thread(StatusThread);
    _statusThread->set_priority(osPriorityIdle);
    
    Thread::wait(1000);
    
    printf("*********************************************************************************\r\n");
    printf("Finished Setup\r\n");
    printf("*********************************************************************************\r\n");
}

int main()
{   
    Setup(); 
    
    Thread::wait(osWaitForever);
}