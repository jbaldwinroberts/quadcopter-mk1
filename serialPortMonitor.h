#include "mbed.h"
#include "rtos.h"
#include "hardware.h"

//Declarations
void CheckSerialCommand();
void UpdatePID();
void getGPS();

//Variables
char _wirelessSerialBuffer[255];
int _wirelessSerialRxPos = 0;

// A thread to monitor the serial ports
void SerialPortMonitorThread(void const *args) 
{     
    printf("Serial port monitor thread started\r\n");
    
    while(true)
    {
        //Check comms mode and print correct data back to PC application
        switch(_commsMode)
        {
            //Motor power
            case 0:
                _wirelessSerial.printf("<M1=%1.2f:M2=%1.2f:M3=%1.2f:M4=%1.2f>",
                _motorPower[0], _motorPower[1], _motorPower[2], _motorPower[3]);
                break;
            
            //PID outputs
            case 1:
                _wirelessSerial.printf("<SYPID=%1.2f:SPPID=%1.2f:SRPID=%1.2f:RYPID=%1.2f:RPPID=%1.2f:RRPID=%1.2f>",
                _stabPIDControllerOutputs[0], _stabPIDControllerOutputs[1], _stabPIDControllerOutputs[2], _ratePIDControllerOutputs[0], _ratePIDControllerOutputs[1], _ratePIDControllerOutputs[2]);
                break;
            
            //IMU outputs 
            case 2:
                _wirelessSerial.printf("<SY=%1.2f:SP=%1.2f:SR=%1.2f:RY=%1.2f:RP=%1.2f:RR=%1.2f>",
                _ypr[0], _ypr[1], _ypr[2], _gyroRate[0], _gyroRate[1], _gyroRate[2]);
                break;
            
            //Status  
            case 3:
                _wirelessSerial.printf("<Batt=%d:Armed=%d:Init=%d:Rate=%d:Stab=%d:Lev=%d>",
                _batt, _armed, _initialised, _rate, _stab, _levelOffset);
                break;
            
            //Mapped RC commands   
            case 4:
                _wirelessSerial.printf("<MRCY=%1.2f:MRCP=%1.2f:MRCR=%1.2f:MRCT=%1.2f:RRC1=%1.2f:RRC2=%1.2f:RRC3=%1.2f:RRC4=%1.2f:RRC5=%1.2f:RRC6=%1.2f:RRC7=%1.2f:RRC8=%1.2f>",
                _rcMappedCommands[0], _rcMappedCommands[1], _rcMappedCommands[2], _rcMappedCommands[3], _rcCommands[0], _rcCommands[1], _rcCommands[2], _rcCommands[3], _rcCommands[4], _rcCommands[5], _rcCommands[6], _rcCommands[7]);
                break;
            
            //PID Tuning
            case 5:
                _wirelessSerial.printf("<RYPIDP=%1.6f:RYPIDI=%1.6f:RYPIDD=%1.6f:RPPIDP=%1.6f:RPPIDI=%1.6f:RPPIDD=%1.6f:RRPIDP=%1.6f:RRPIDI=%1.6f:RRPIDD=%1.6f:SYPIDP=%1.6f:SYPIDI=%1.6f:SYPIDD=%1.6f:SPPIDP=%1.6f:SPPIDI=%1.6f:SPPIDD=%1.6f:SRPIDP=%1.6f:SRPIDI=%1.6f:SRPIDD=%1.6f>",
                _yawRatePIDControllerP, _yawRatePIDControllerI, _yawRatePIDControllerD, _pitchRatePIDControllerP, _pitchRatePIDControllerI, _pitchRatePIDControllerD, _rollRatePIDControllerP, _rollRatePIDControllerI, _rollRatePIDControllerD, _yawStabPIDControllerP, _yawStabPIDControllerI, _yawStabPIDControllerD, _pitchStabPIDControllerP, _pitchStabPIDControllerI, _pitchStabPIDControllerD, _rollStabPIDControllerP, _rollStabPIDControllerI, _rollStabPIDControllerD);
                break;
              
            //GPS  
            case 6:
                _wirelessSerial.printf("<GLat=%1.6f:GLon=%1.6f:GAlt=%1.2f:GAng=%1.2f:GSpd=%1.2f:GInit=%d>",
                _gpsValues[0], _gpsValues[1], _gpsValues[2], _gpsValues[3], _gpsValues[4], _gpsConnected);
                break;
            
            //Zero mode  
            case 7:
                _wirelessSerial.printf("<ZY=%1.6f:ZP=%1.6f:ZR=%1.6f>",
                _zeroValues[0], _zeroValues[1], _zeroValues[2]);
                break;
            
            //Rate tuning
            case 8:
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wirelessSerial.printf("<MRCY=%1.2f:RY=%1.2f:RYPID=%1.2f:MRCP=%1.2f:RP=%1.2f:RPPID=%1.2f:MRCR=%1.2f:RR=%1.2f:RRPID=%1.2f>",
                _rcMappedCommands[0], _gyroRate[0], _ratePIDControllerOutputs[0], _rcMappedCommands[1], _gyroRate[1], _ratePIDControllerOutputs[1], _rcMappedCommands[2], _gyroRate[2], _ratePIDControllerOutputs[2]);
                break;
                
            //Stab tuning  
            case 9:
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wirelessSerial.printf("<MRCY=%1.2f:SY=%1.2f:SYPID=%1.2f:MRCP=%1.2f:SP=%1.2f:SPPID=%1.2f:MRCR=%1.2f:SR=%1.2f:SRPID=%1.2f>",
                _rcMappedCommands[0], _ypr[0], _stabPIDControllerOutputs[0], _rcMappedCommands[1], _ypr[1], _stabPIDControllerOutputs[1], _rcMappedCommands[2], _ypr[2], _stabPIDControllerOutputs[2]);
                break;
            
            //Altitude    
            case 10:
                _wirelessSerial.printf("<GAlt=%1.2f:PAlt=%1.2f:BAlt=%1.2f>",
                _gpsValues[2], _maxBotixPingAltitude, _barometerAltitude);
                
            default:
                break;                  
        }  
        
        //Check for wireless serial command
        while (_wirelessSerial.readable() > 0)
        {
            int c = _wirelessSerial.getc();
                                                
            switch (c)
            {
                case 60: // 
                    _wirelessSerialRxPos = 0;
                    break;
                
                case 10: // LF
                case 13: // CR
                case 62: // >
                    CheckSerialCommand();
                    break;
                    
                default:
                    _wirelessSerialBuffer[_wirelessSerialRxPos++] = c;
                    if (_wirelessSerialRxPos > 200)
                    {
                        _wirelessSerialRxPos = 0;
                    }
                    break;
            }
        }                                  
        
        //Check for GPS serial command
        while(_gps.readable() > 0)
        {
            int c = _gps.getc();
            if(_tinyGPS.encode(c))
            {
                getGPS();
            }
        }
        
        Thread::wait(100);
    }
}

//Checks for a valid command from the serial port and executes it
//<Command=Value>
void CheckSerialCommand()
{
    int length = _wirelessSerialRxPos;
    _wirelessSerialBuffer[_wirelessSerialRxPos] = 0;
    _wirelessSerialRxPos = 0;

    if (length < 1)
    {
        return;
    }
    
    char command = _wirelessSerialBuffer[0];
    double value = 0;
    if(length > 1)
    {
        value = atof((char*)&_wirelessSerialBuffer[2]);
    }
    
    switch (command)
    {
        //Start level offset mode to teach quad level
        case 'a':
            _levelOffset = true;
            break;
            
        //Arm disarm
        case 'b':
            if(_initialised == true && _armed == false)
            {
                Arm();
            }
            else if(_armed == true)
            {
                Disarm();
            }
            break;
            
        //Set mode
        case 'c':
            if(_rate == true)
            {
                _rate = false;
                _stab = true;
            }
            else
            {
                _rate = true;
                _stab = false;
            }
            break;
            
        //Set yaw
        case 'd':
            if(_armed == true) _rcMappedCommands[0] = value; //Yaw
            else _rcMappedCommands[0] = 0;
            break;
            
        //Set pitch
        case 'e':
            if(_armed == true) _rcMappedCommands[1] = value; //Pitch
            else _rcMappedCommands[1] = 0;
            break;
            
        //Set roll
        case 'f':
            if(_armed == true) _rcMappedCommands[2] = value; //Roll
            else _rcMappedCommands[2] = 0;
            break;
            
        //Set thrust
        case 'g':
            if(_armed == true) _rcMappedCommands[3] = value; //Thrust
            else _rcMappedCommands[3] = 0;
            break;
            
        //Set PID values
        case 'h':
            _yawRatePIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'i':
            _yawRatePIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'j':
            _yawRatePIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'k':
            _pitchRatePIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'l':
            _pitchRatePIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'm':
            _pitchRatePIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'n':
            _rollRatePIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'o':
            _rollRatePIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'p':
            _rollRatePIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'q':
            _yawStabPIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'r':
            _yawStabPIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 's':
            _yawStabPIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 't':
            _pitchStabPIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'u':
            _pitchStabPIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'v':
            _pitchStabPIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'w':
            _rollStabPIDControllerP = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'x':
            _rollStabPIDControllerI = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'y':
            _rollStabPIDControllerD = value;
            UpdatePID();
            WriteSettingsToConfig();
            break;
            
        case 'z':
            _commsMode = value;
            break;
            
        case '1':
            _levelOffset = false;
            _zeroValues[0] = 0;
            _zeroValues[1] = 0;
            _zeroValues[2] = 0;
            WriteSettingsToConfig();
            break;
            
        default:
            break;
    }
    
    return;
}

void getGPS()
{ 
    unsigned long fix_age;
    _tinyGPS.f_get_position(&_gpsValues[0], &_gpsValues[1], &fix_age);
  
    _gpsValues[2] = _tinyGPS.f_altitude();
    _gpsValues[3] = _tinyGPS.f_course();
    _gpsValues[4] = _tinyGPS.f_speed_kmph();
    
    if (fix_age == TinyGPS::GPS_INVALID_AGE)
      _gpsConnected = false;
    else if (fix_age > 5000)
      _gpsConnected = false;
    else
      _gpsConnected = true;
}

//Updates PID tunings
void UpdatePID()
{
    _yawRatePIDController->setTunings(_yawRatePIDControllerP, _yawRatePIDControllerI, _yawRatePIDControllerD);
    _pitchRatePIDController->setTunings(_pitchRatePIDControllerP, _pitchRatePIDControllerI, _pitchRatePIDControllerD);
    _rollRatePIDController->setTunings(_rollRatePIDControllerP, _rollRatePIDControllerI, _rollRatePIDControllerD);
    _yawStabPIDController->setTunings(_yawStabPIDControllerP, _yawStabPIDControllerI, _yawStabPIDControllerD);
    _pitchStabPIDController->setTunings(_pitchStabPIDControllerP, _pitchStabPIDControllerI, _pitchStabPIDControllerD);
    _rollStabPIDController->setTunings(_rollStabPIDControllerP, _rollStabPIDControllerI, _rollStabPIDControllerD);
}
